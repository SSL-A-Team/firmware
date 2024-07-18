use ateam_common_packets::{bindings_radio::BasicTelemetry, bindings_stspin::MotorCommand_MotionType, radio::TelemetryPacket};
use ateam_lib_stm32::{make_uart_queue_pair, queue_pair_register_and_spawn};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Ticker, Timer};
use nalgebra::{Vector3, Vector4};

use crate::{include_external_cpp_bin, motion::{self, params::robot_physical_params::{
        WHEEL_ANGLES_DEG, WHEEL_DISTANCE_TO_ROBOT_CENTER_M, WHEEL_RADIUS_M
    }, robot_controller::BodyVelocityController, robot_model::{RobotConstants, RobotModel}}, parameter_interface::ParameterInterface, pins::*, robot_state::SharedRobotState, stm32_interface, stspin_motor::{DribblerMotor, WheelMotor}, SystemIrqs};

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}
include_external_cpp_bin! {DRIB_FW_IMG, "dribbler.bin"}

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

make_uart_queue_pair!(FRONT_LEFT,
    MotorFLUart, MotorFLDmaRx, MotorFLDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(BACK_LEFT,
    MotorBLUart, MotorBLDmaRx, MotorBLDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(BACK_RIGHT,
    MotorBRUart, MotorBRDmaRx, MotorBRDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(FRONT_RIGHT,
    MotorFRUart, MotorFRDmaRx, MotorFRDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

make_uart_queue_pair!(DRIB,
    MotorDUart, MotorDDmaRx, MotorDDmaTx,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);

const TICKS_WITHOUT_PACKET_STOP: usize = 20;
const BATTERY_MIN_VOLTAGE: f32 = 18.0;

pub struct ControlTask<
    const MAX_RX_PACKET_SIZE: usize,
    const MAX_TX_PACKET_SIZE: usize,
    const RX_BUF_DEPTH: usize,
    const TX_BUF_DEPTH: usize> {
    shared_robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    battery_subscriber: BatteryVoltSubscriber,
    last_battery_v: f32,
    gyro_subscriber: GyroDataSubscriber,
    last_gyro_rads: f32,
    telemetry_publisher: TelemetryPublisher,
    
    motor_fl: WheelMotor<'static, MotorFLUart, MotorFLDmaRx, MotorFLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_bl: WheelMotor<'static, MotorBLUart, MotorBLDmaRx, MotorBLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_br: WheelMotor<'static, MotorBRUart, MotorBRDmaRx, MotorBRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_fr: WheelMotor<'static, MotorFRUart, MotorFRDmaRx, MotorFRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_drib: DribblerMotor<'static, MotorDUart, MotorDDmaRx, MotorDDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH> 
}

impl <
    const MAX_RX_PACKET_SIZE: usize,
    const MAX_TX_PACKET_SIZE: usize,
    const RX_BUF_DEPTH: usize,
    const TX_BUF_DEPTH: usize> 
    ControlTask<MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH> 
    {

        pub fn new(robot_state: &'static SharedRobotState,
                command_subscriber: CommandsSubscriber,
                telemetry_publisher: TelemetryPublisher,
                battery_subscriber: BatteryVoltSubscriber,
                gyro_subscriber: GyroDataSubscriber,
                motor_fl: WheelMotor<'static, MotorFLUart, MotorFLDmaRx, MotorFLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_bl: WheelMotor<'static, MotorBLUart, MotorBLDmaRx, MotorBLDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_br: WheelMotor<'static, MotorBRUart, MotorBRDmaRx, MotorBRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_fr: WheelMotor<'static, MotorFRUart, MotorFRDmaRx, MotorFRDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_drib: DribblerMotor<'static, MotorDUart, MotorDDmaRx, MotorDDmaTx, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH> 
        ) -> Self {
            ControlTask {
                shared_robot_state: robot_state,
                command_subscriber: command_subscriber,
                telemetry_publisher: telemetry_publisher,
                battery_subscriber: battery_subscriber,
                last_battery_v: 0.0,
                gyro_subscriber: gyro_subscriber,
                last_gyro_rads: 0.0,
                motor_fl: motor_fl, 
                motor_bl: motor_bl,
                motor_br: motor_br,
                motor_fr: motor_fr,
                motor_drib: motor_drib 
            }
        }

        fn do_control_update(&mut self, 
            robot_controller: &mut BodyVelocityController,
            cmd_vel: Vector3<f32>,
            gyro_rads: f32,
            controls_enabled: bool
        ) -> Vector4<f32>
        /*
            Provide the motion controller with the current wheel velocities
            and torques from the appropriate sensors, then get a set of wheel
            velocities to apply based on the controller's current state.
         */ 
        {
            let wheel_vels = Vector4::new(
                self.motor_fl.read_rads(),
                self.motor_bl.read_rads(),
                self.motor_br.read_rads(),
                self.motor_fr.read_rads()
            );

            // torque values are computed on the spin but put in the current variable
            // TODO update this when packet/var names are updated to match software
            let wheel_torques = Vector4::new(
                self.motor_fl.read_current(),
                self.motor_bl.read_current(),
                self.motor_br.read_current(),
                self.motor_fr.read_current()
            );
        
            // TODO read from channel or something

            robot_controller.control_update(&cmd_vel, &wheel_vels, &wheel_torques, gyro_rads, controls_enabled); 
            robot_controller.get_wheel_velocities()   
        }

        fn send_motor_commands_and_telemetry(&mut self,
                                            robot_controller: &mut BodyVelocityController,
                                            battery_voltage: f32) 
        {
            self.motor_fl.send_motion_command();
            self.motor_bl.send_motion_command();
            self.motor_br.send_motion_command();
            self.motor_fr.send_motion_command();
            self.motor_drib.send_motion_command();

            let err_fr = self.motor_fr.read_is_error() as u32;
            let err_fl = self.motor_fl.read_is_error() as u32;
            let err_br = self.motor_br.read_is_error() as u32;
            let err_bl = self.motor_bl.read_is_error() as u32;
            let err_drib = self.motor_drib.read_is_error() as u32;

            let hall_err_fr = self.motor_fr.check_hall_error() as u32;
            let hall_err_fl = self.motor_fl.check_hall_error() as u32;
            let hall_err_br = self.motor_br.check_hall_error() as u32;
            let hall_err_bl = self.motor_bl.check_hall_error() as u32;
            let hall_err_drib = self.motor_drib.check_hall_error() as u32;

            let basic_telem = TelemetryPacket::Basic(BasicTelemetry {
                sequence_number: 0,
                robot_revision_major: 0,
                robot_revision_minor: 0,
                battery_level: battery_voltage,
                battery_temperature: 0.,
                _bitfield_align_1: [],
                _bitfield_1: BasicTelemetry::new_bitfield_1(
                    0, 0, 0, self.shared_robot_state.ball_detected() as u32, 0, 0, 0, 0, err_fl, hall_err_fl, err_bl, hall_err_bl, err_br, hall_err_br, err_fr, hall_err_fr, err_drib, hall_err_drib, 0, 0, 0,
                ),
                motor_0_temperature: 0.,
                motor_1_temperature: 0.,
                motor_2_temperature: 0.,
                motor_3_temperature: 0.,
                motor_4_temperature: 0.,
                kicker_charge_level: 0.,
            });
            self.telemetry_publisher.publish_immediate(basic_telem);

            let mut control_debug_telem = robot_controller.get_control_debug_telem();
            control_debug_telem.motor_fl.wheel_torque = self.motor_fl.read_hall_vel();
            control_debug_telem.motor_bl.wheel_torque = self.motor_bl.read_hall_vel();
            control_debug_telem.motor_br.wheel_torque = self.motor_br.read_hall_vel();
            control_debug_telem.motor_fr.wheel_torque = self.motor_fr.read_hall_vel();

            let control_debug_telem = TelemetryPacket::Control(control_debug_telem);
            self.telemetry_publisher.publish_immediate(control_debug_telem);
        }
    
        async fn control_task_entry(&mut self) {
            defmt::info!("control task init.");

            // wait for the switch state to be read
            while !self.shared_robot_state.hw_init_state_valid() {
                Timer::after_millis(10).await;
            }
 
            self.flash_motor_firmware(
                self.shared_robot_state.hw_in_debug_mode()).await;
             
            embassy_futures::join::join5(
                self.motor_fl.leave_reset(),
                self.motor_bl.leave_reset(),
                self.motor_br.leave_reset(),
                self.motor_fr.leave_reset(),
                self.motor_drib.leave_reset(),
            ).await;


            self.motor_fl.set_telemetry_enabled(true);
            self.motor_bl.set_telemetry_enabled(true);
            self.motor_br.set_telemetry_enabled(true);
            self.motor_fr.set_telemetry_enabled(true);
            self.motor_drib.set_telemetry_enabled(true);

            self.motor_fl.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
            self.motor_bl.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
            self.motor_br.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);
            self.motor_fr.set_motion_type(MotorCommand_MotionType::OPEN_LOOP);


            Timer::after_millis(10).await;

            let robot_model = self.get_robot_model();
            let mut robot_controller = BodyVelocityController::new_from_global_params(1.0 / 100.0, robot_model);
    
            let mut loop_rate_ticker = Ticker::every(Duration::from_millis(10));

            let mut cmd_vel = Vector3::new(0.0, 0.0, 0.0);
            let mut drib_vel = 0.0;
            let mut ticks_since_control_packet = 0;

            loop {
                self.motor_fl.process_packets();
                self.motor_bl.process_packets();
                self.motor_br.process_packets();
                self.motor_fr.process_packets();
                self.motor_drib.process_packets();

                self.motor_fl.log_reset("FL");
                self.motor_bl.log_reset("BL");
                self.motor_br.log_reset("BR");
                self.motor_fr.log_reset("FR");
                self.motor_drib.log_reset("DRIB");

                if self.motor_drib.ball_detected() {
                    defmt::info!("ball detected");
                }

                ticks_since_control_packet += 1;
                while let Some(latest_packet) = self.command_subscriber.try_next_message_pure() {
                    match latest_packet {
                        ateam_common_packets::radio::DataPacket::BasicControl(latest_control) => {

                            let new_cmd_vel = Vector3::new(
                                latest_control.vel_x_linear,
                                latest_control.vel_y_linear,
                                latest_control.vel_z_angular,
                            );

                            cmd_vel = new_cmd_vel;
                            drib_vel = latest_control.dribbler_speed;
                            ticks_since_control_packet = 0;
                        },
                        ateam_common_packets::radio::DataPacket::ParameterCommand(latest_param_cmd) => {
                            let param_cmd_resp = robot_controller.apply_command(&latest_param_cmd);

                            if let Ok(resp) = param_cmd_resp {
                                defmt::info!("sending successful parameter update command response");
                                let tp_resp = TelemetryPacket::ParameterCommandResponse(resp);
                                self.telemetry_publisher.publish(tp_resp).await;
                            } else if let Err(resp) = param_cmd_resp {
                                defmt::warn!("sending failed parameter updated command response");
                                let tp_resp = TelemetryPacket::ParameterCommandResponse(resp);
                                self.telemetry_publisher.publish(tp_resp).await;
                            }
                        },
                    }
                }
                
                if ticks_since_control_packet >= TICKS_WITHOUT_PACKET_STOP {
                    cmd_vel = Vector3::new(0., 0., 0.);
                    drib_vel = 0.0;
                    defmt::warn!("ticks since packet lockout");
                }

                // now we have setpoint r(t) in self.cmd_vel
                while let Some(battery_v) = self.battery_subscriber.try_next_message_pure() {
                    self.last_battery_v = battery_v;
                }

                while let Some(gyro_rads) = self.gyro_subscriber.try_next_message_pure() {
                    self.last_gyro_rads = gyro_rads[2];
                }
                
                let controls_enabled = true;

                let wheel_vels = if !(self.shared_robot_state.get_battery_low() || self.shared_robot_state.get_battery_crit()) && !self.shared_robot_state.shutdown_requested() {
                    // TODO check order
                    self.do_control_update(&mut robot_controller, cmd_vel, self.last_gyro_rads, controls_enabled)
                } else {
                    // Battery is too low, set velocity to zero
                    drib_vel = 0.0;
                    defmt::warn!("CT - low battery / shutting down command lockout");
                    Vector4::new(0.0, 0.0, 0.0, 0.0)
                };

                self.motor_fl.set_setpoint(wheel_vels[0]);
                self.motor_bl.set_setpoint(wheel_vels[1]);
                self.motor_br.set_setpoint(wheel_vels[2]);
                self.motor_fr.set_setpoint(wheel_vels[3]);

                let drib_dc = -1.0 * drib_vel / 1000.0;
                self.motor_drib.set_setpoint(drib_dc);

                self.send_motor_commands_and_telemetry(
                    &mut robot_controller, self.last_battery_v);

                loop_rate_ticker.next().await;
            }
        }

        async fn flash_motor_firmware(&mut self, debug: bool) {
            defmt::info!("flashing firmware");
            if true {
                let mut had_motor_error = false;
                if self.motor_fl.load_default_firmware_image().await.is_err() {
                    defmt::error!("failed to flash FL");
                    had_motor_error = true;
                } else {
                    defmt::info!("FL flashed");
                }

                if self.motor_bl.load_default_firmware_image().await.is_err() {
                    defmt::error!("failed to flash BL");
                    had_motor_error = true;
                } else {
                    defmt::info!("BL flashed");
                }

                if self.motor_br.load_default_firmware_image().await.is_err() {
                    defmt::error!("failed to flash BR");
                    had_motor_error = true;
                } else {
                    defmt::info!("BR flashed");
                }

                if self.motor_fr.load_default_firmware_image().await.is_err() {
                    defmt::error!("failed to flash FR");
                    had_motor_error = true;
                } else {
                    defmt::info!("FR flashed");
                }

                if self.motor_drib.load_default_firmware_image().await.is_err() {
                    defmt::error!("failed to flash DRIB");
                    had_motor_error = true;
                } else {
                    defmt::info!("DRIB flashed");
                }

                if had_motor_error {
                    defmt::error!("one or more motors failed to flash.")
                } else {
                    defmt::debug!("all motors flashed");
                }
            } else {
                let res = embassy_futures::join::join5(
                    self.motor_fl.load_default_firmware_image(),
                    self.motor_bl.load_default_firmware_image(),
                    self.motor_br.load_default_firmware_image(),
                    self.motor_fr.load_default_firmware_image(),
                    self.motor_drib.load_default_firmware_image(),
                )
                .await;
                
                let error_mask = res.0.is_err() as u8 
                        | ((res.1.is_err() as u8) & 0x01) << 1 
                        | ((res.2.is_err() as u8) & 0x01) << 2
                        | ((res.3.is_err() as u8) & 0x01) << 3;

                self.shared_robot_state.set_wheels_inop(error_mask);
                self.shared_robot_state.set_dribbler_inop(res.4.is_err());

                if error_mask != 0 {
                    defmt::error!("failed to flash drive motor (FL, BL, BR, FR, DRIB): {}", res);
                } else {
                    defmt::debug!("motor firmware flashed");
                }
            }
        }

        fn get_robot_model(&mut self) -> motion::robot_model::RobotModel{
            let robot_model_constants: RobotConstants = RobotConstants {
                wheel_angles_rad: Vector4::new(
                    WHEEL_ANGLES_DEG[0].to_radians(),
                    WHEEL_ANGLES_DEG[1].to_radians(),
                    WHEEL_ANGLES_DEG[2].to_radians(),
                    WHEEL_ANGLES_DEG[3].to_radians(),
                ),
                wheel_radius_m: Vector4::new(
                    WHEEL_RADIUS_M,
                    WHEEL_RADIUS_M,
                    WHEEL_RADIUS_M,
                    WHEEL_RADIUS_M,
                ),
                wheel_dist_to_cent_m: Vector4::new(
                    WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                    WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                    WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                    WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
                ),
            };

            let robot_model: RobotModel = RobotModel::new(robot_model_constants);
            
            return robot_model; 
        }
    }
    
#[embassy_executor::task]
async fn control_task_entry(mut control_task: ControlTask<MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>) {
    loop {
        control_task.control_task_entry().await;
        defmt::error!("control task returned");
    }
}

pub async fn start_control_task(
    uart_queue_spawner: SendSpawner,
    control_task_spawner: Spawner,
    robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    battery_subscriber: BatteryVoltSubscriber,
    gyro_subscriber: GyroDataSubscriber,
    motor_fl_uart: MotorFLUart, motor_fl_rx_pin: MotorFLUartRxPin, motor_fl_tx_pin: MotorFLUartTxPin, motor_fl_rx_dma: MotorFLDmaRx, motor_fl_tx_dma: MotorFLDmaTx, motor_fl_boot0_pin: MotorFLBootPin, motor_fl_nrst_pin: MotorFLResetPin,
    motor_bl_uart: MotorBLUart, motor_bl_rx_pin: MotorBLUartRxPin, motor_bl_tx_pin: MotorBLUartTxPin, motor_bl_rx_dma: MotorBLDmaRx, motor_bl_tx_dma: MotorBLDmaTx, motor_bl_boot0_pin: MotorBLBootPin, motor_bl_nrst_pin: MotorBLResetPin,
    motor_br_uart: MotorBRUart, motor_br_rx_pin: MotorBRUartRxPin, motor_br_tx_pin: MotorBRUartTxPin, motor_br_rx_dma: MotorBRDmaRx, motor_br_tx_dma: MotorBRDmaTx, motor_br_boot0_pin: MotorBRBootPin, motor_br_nrst_pin: MotorBRResetPin,
    motor_fr_uart: MotorFRUart, motor_fr_rx_pin: MotorFRUartRxPin, motor_fr_tx_pin: MotorFRUartTxPin, motor_fr_rx_dma: MotorFRDmaRx, motor_fr_tx_dma: MotorFRDmaTx, motor_fr_boot0_pin: MotorFRBootPin, motor_fr_nrst_pin: MotorFRResetPin,
    motor_d_uart: MotorDUart,   motor_d_rx_pin: MotorDUartRxPin,   motor_d_tx_pin: MotorDUartTxPin,   motor_d_rx_dma: MotorDDmaRx,   motor_d_tx_dma: MotorDDmaTx,  motor_d_boot0_pin: MotorDBootPin,  motor_d_nrst_pin: MotorDResetPin,

) {
    let initial_motor_controller_uart_conifg = stm32_interface::get_bootloader_uart_config();

    //////////////////////////
    //  create motor uarts  //
    //////////////////////////

    let fl_uart = Uart::new(motor_fl_uart, motor_fl_rx_pin, motor_fl_tx_pin, SystemIrqs, motor_fl_tx_dma, motor_fl_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let bl_uart = Uart::new(motor_bl_uart, motor_bl_rx_pin, motor_bl_tx_pin, SystemIrqs, motor_bl_tx_dma, motor_bl_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let br_uart = Uart::new(motor_br_uart, motor_br_rx_pin, motor_br_tx_pin, SystemIrqs, motor_br_tx_dma, motor_br_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let fr_uart = Uart::new(motor_fr_uart, motor_fr_rx_pin, motor_fr_tx_pin, SystemIrqs, motor_fr_tx_dma, motor_fr_rx_dma, initial_motor_controller_uart_conifg).unwrap();
    let drib_uart = Uart::new(motor_d_uart, motor_d_rx_pin, motor_d_tx_pin, SystemIrqs, motor_d_tx_dma, motor_d_rx_dma, initial_motor_controller_uart_conifg).unwrap();

    //////////////////////////////////////////////
    //  register motor queues and DMA hardware  //
    //////////////////////////////////////////////

    let (fl_uart_tx, fl_uart_rx) = Uart::split(fl_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, FRONT_LEFT, fl_uart_rx, fl_uart_tx);
    let (bl_uart_tx, bl_uart_rx) = Uart::split(bl_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, BACK_LEFT, bl_uart_rx, bl_uart_tx);
    let (br_uart_tx, br_uart_rx) = Uart::split(br_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, BACK_RIGHT, br_uart_rx, br_uart_tx);
    let (fr_uart_tx, fr_uart_rx) = Uart::split(fr_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, FRONT_RIGHT, fr_uart_rx, fr_uart_tx);

    let (drib_uart_tx, drib_uart_rx) = Uart::split(drib_uart);
    queue_pair_register_and_spawn!(uart_queue_spawner, DRIB, drib_uart_rx, drib_uart_tx);

    ////////////////////////////////
    //  create motor controllers  //
    ////////////////////////////////
    
    let motor_fl = WheelMotor::new_from_pins(&FRONT_LEFT_RX_UART_QUEUE,  &FRONT_LEFT_TX_UART_QUEUE,  motor_fl_boot0_pin, motor_fl_nrst_pin, WHEEL_FW_IMG);
    let motor_bl = WheelMotor::new_from_pins(&BACK_LEFT_RX_UART_QUEUE,   &BACK_LEFT_TX_UART_QUEUE,   motor_bl_boot0_pin, motor_bl_nrst_pin, WHEEL_FW_IMG);
    let motor_br = WheelMotor::new_from_pins(&BACK_RIGHT_RX_UART_QUEUE,  &BACK_RIGHT_TX_UART_QUEUE,  motor_br_boot0_pin, motor_br_nrst_pin, WHEEL_FW_IMG);
    let motor_fr = WheelMotor::new_from_pins(&FRONT_RIGHT_RX_UART_QUEUE, &FRONT_RIGHT_TX_UART_QUEUE, motor_fr_boot0_pin, motor_fr_nrst_pin, WHEEL_FW_IMG);
    let motor_drib = DribblerMotor::new_from_pins(&DRIB_RX_UART_QUEUE,   &DRIB_TX_UART_QUEUE,        motor_d_boot0_pin,  motor_d_nrst_pin,  DRIB_FW_IMG, 1.0);

    let control_task = ControlTask::new(
        robot_state, command_subscriber, telemetry_publisher, battery_subscriber,
        gyro_subscriber, motor_fl, motor_bl, motor_br, motor_fr, motor_drib);

    control_task_spawner.spawn(control_task_entry(control_task)).unwrap();
}