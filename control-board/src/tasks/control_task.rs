use ateam_common_packets::{bindings::{BasicTelemetry, MotorCommand_MotionType}, radio::TelemetryPacket};
use ateam_lib_stm32::{idle_buffered_uart_spawn_tasks, static_idle_buffered_uart};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Ticker, Timer};
use nalgebra::{Vector3, Vector4};

use crate::{include_external_cpp_bin, motion::{self, params::robot_physical_params::{
        WHEEL_ANGLES_DEG, WHEEL_DISTANCE_TO_ROBOT_CENTER_M, WHEEL_RADIUS_M
    }, robot_controller::BodyVelocityController, robot_model::{RobotConstants, RobotModel}}, parameter_interface::ParameterInterface, pins::*, robot_state::SharedRobotState, stm32_interface, stspin_motor::WheelMotor, SystemIrqs};

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

static_idle_buffered_uart!(FRONT_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(FRONT_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);

const TICKS_WITHOUT_PACKET_STOP: usize = 20;
const BATTERY_MIN_VOLTAGE: f32 = 18.0;

// This is Gyro for all axes.
const MAX_STATIONARY_GYRO_RADS: f32 = 0.1; // rad/s
const MIN_GYRO_STABLE_COUNT: usize = 1000; // number of ticks to wait for gyro to stabilize
// This is Acceleration for x and y.
const MAX_STATIONARY_ACCEL_MS: f32 = 0.1; // m/s^2

const WHEEL_VELOCITY_STATIONARY_RADS_MAX: f32 = 0.1; // rad/s
const CURRENT_CALIBRATION_SAMPLES: usize = 1000; // number of samples to take for current calibration

const CONTROL_LOOP_RATE_MS: u64 = 10; // 100Hz
const CONTROL_LOOP_RATE_S: f32 = CONTROL_LOOP_RATE_MS as f32 / 1000.0;
const CONTROL_MOTION_TYPE: MotorCommand_MotionType::Type = MotorCommand_MotionType::OPEN_LOOP;

// Internal macro with do_motor_calibrate as a parameter
#[macro_export]
macro_rules! __create_control_task_internal {
    (
        $main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $control_command_subscriber:ident, $control_telemetry_publisher:ident,
        $battery_volt_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident, $do_motor_calibrate:expr
    ) => {
        ateam_control_board::tasks::control_task::start_control_task(
            $main_spawner, $uart_queue_spawner,
            $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $battery_volt_subscriber,
            $control_gyro_data_subscriber, $control_accel_data_subscriber,
            $p.UART7, $p.PF6, $p.PF7, $p.DMA1_CH1, $p.DMA1_CH0, $p.PF5, $p.PF4,
            $p.USART10, $p.PE2, $p.PE3, $p.DMA1_CH3, $p.DMA1_CH2, $p.PE5, $p.PE4,
            $p.USART6, $p.PC7, $p.PC6, $p.DMA1_CH5, $p.DMA1_CH4, $p.PG7, $p.PG8,
            $p.USART3, $p.PD9, $p.PD8, $p.DMA1_CH7, $p.DMA1_CH6, $p.PB12, $p.PB13,
            $do_motor_calibrate,
        ).await;
    };
}

// External macro for normal control task
#[macro_export]
macro_rules! create_control_task {
    (
        $main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $control_command_subscriber:ident, $control_telemetry_publisher:ident,
        $battery_volt_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident
    ) => {
        $crate::__create_control_task_internal!(
            $main_spawner, $uart_queue_spawner, $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $battery_volt_subscriber,
            $control_gyro_data_subscriber, $control_accel_data_subscriber,
            $p, false
        );
    };
}

// External macro for motor calibration task
#[macro_export]
macro_rules! create_motor_calibrate_task {
    (
        $main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $control_command_subscriber:ident, $control_telemetry_publisher:ident,
        $battery_volt_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident
    ) => {
        $crate::__create_control_task_internal!(
            $main_spawner, $uart_queue_spawner, $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $battery_volt_subscriber,
            $control_gyro_data_subscriber, $control_accel_data_subscriber,
            $p, true
        );
    };
}

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
    accel_subscriber: AccelDataSubscriber,
    last_gyro_x_rads: f32,
    last_gyro_y_rads: f32,
    last_gyro_z_rads: f32,
    last_accel_x_ms: f32,
    last_accel_y_ms: f32,
    last_accel_z_ms: f32,
    telemetry_publisher: TelemetryPublisher,

    motor_fl: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_bl: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_br: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
    motor_fr: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
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
                accel_subscriber: AccelDataSubscriber,
                motor_fl: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_bl: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_br: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
                motor_fr: WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>,
        ) -> Self {
            ControlTask {
                shared_robot_state: robot_state,
                command_subscriber: command_subscriber,
                telemetry_publisher: telemetry_publisher,
                battery_subscriber: battery_subscriber,
                last_battery_v: 0.0,
                gyro_subscriber: gyro_subscriber,
                accel_subscriber: accel_subscriber,
                last_gyro_x_rads: 0.0,
                last_gyro_y_rads: 0.0,
                last_gyro_z_rads: 0.0,
                last_accel_x_ms: 0.0,
                last_accel_y_ms: 0.0,
                last_accel_z_ms: 0.0,
                motor_fl: motor_fl,
                motor_bl: motor_bl,
                motor_br: motor_br,
                motor_fr: motor_fr,
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

            let err_fr = self.motor_fr.read_is_error() as u32;
            let err_fl = self.motor_fl.read_is_error() as u32;
            let err_br = self.motor_br.read_is_error() as u32;
            let err_bl = self.motor_bl.read_is_error() as u32;

            let hall_err_fr = self.motor_fr.check_hall_error() as u32;
            let hall_err_fl = self.motor_fl.check_hall_error() as u32;
            let hall_err_br = self.motor_br.check_hall_error() as u32;
            let hall_err_bl = self.motor_bl.check_hall_error() as u32;

            let basic_telem = TelemetryPacket::Basic(BasicTelemetry {
                sequence_number: 0,
                robot_revision_major: 0,
                robot_revision_minor: 0,
                battery_level: battery_voltage,
                battery_temperature: 0.,
                _bitfield_align_1: [],
                _bitfield_1: BasicTelemetry::new_bitfield_1(
                    0, 0, 0, self.shared_robot_state.ball_detected() as u32, 0, 0, 0, 0, err_fl, hall_err_fl, err_bl, hall_err_bl, err_br, hall_err_br, err_fr, hall_err_fr, 0, 0, 0, 0, 0,
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

            control_debug_telem.motor_fl = self.motor_fl.get_latest_state();
            control_debug_telem.motor_bl = self.motor_bl.get_latest_state();
            control_debug_telem.motor_br = self.motor_br.get_latest_state();
            control_debug_telem.motor_fr = self.motor_fr.get_latest_state();

            control_debug_telem.imu_accel[0] = self.last_accel_x_ms;
            control_debug_telem.imu_accel[1] = self.last_accel_y_ms;

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

            embassy_futures::join::join4(
                self.motor_fl.leave_reset(),
                self.motor_bl.leave_reset(),
                self.motor_br.leave_reset(),
                self.motor_fr.leave_reset(),
            ).await;

            let robot_model = self.get_robot_model();
            let mut robot_controller = BodyVelocityController::new_from_global_params(CONTROL_LOOP_RATE_S, robot_model);

            let mut loop_rate_ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_RATE_MS));

            let mut cmd_vel = Vector3::new(0.0, 0.0, 0.0);
            let mut ticks_since_control_packet = 0;

            while self.shared_robot_state.get_imu_inop() {
                defmt::info!("Waiting for IMU to be ready.");
                loop_rate_ticker.next().await;
            }

            self.motor_fl.set_telemetry_enabled(true);
            self.motor_bl.set_telemetry_enabled(true);
            self.motor_br.set_telemetry_enabled(true);
            self.motor_fr.set_telemetry_enabled(true);

            self.motor_fl.set_motion_enabled(true);
            self.motor_bl.set_motion_enabled(true);
            self.motor_br.set_motion_enabled(true);
            self.motor_fr.set_motion_enabled(true);

            self.motor_fl.set_motion_type(CONTROL_MOTION_TYPE);
            self.motor_bl.set_motion_type(CONTROL_MOTION_TYPE);
            self.motor_br.set_motion_type(CONTROL_MOTION_TYPE);
            self.motor_fr.set_motion_type(CONTROL_MOTION_TYPE);

            // Need to send one motion command to get telemetry started.
            self.motor_fl.send_motion_command();
            self.motor_bl.send_motion_command();
            self.motor_br.send_motion_command();
            self.motor_fr.send_motion_command();

            Timer::after_millis(10).await;

            let mut motor_fl_last_timestamp_ms = self.motor_fl.read_current_timestamp_ms();
            let mut motor_bl_last_timestamp_ms = self.motor_bl.read_current_timestamp_ms();
            let mut motor_br_last_timestamp_ms = self.motor_br.read_current_timestamp_ms();
            let mut motor_fr_last_timestamp_ms = self.motor_fr.read_current_timestamp_ms();

            loop {
                self.motor_fl.process_packets();
                self.motor_bl.process_packets();
                self.motor_br.process_packets();
                self.motor_fr.process_packets();

                // Check if the motors are still responding, and log which ones are not.
                let motor_fl_current_timestamp_ms = self.motor_fl.read_current_timestamp_ms();
                let motor_bl_current_timestamp_ms = self.motor_bl.read_current_timestamp_ms();
                let motor_br_current_timestamp_ms = self.motor_br.read_current_timestamp_ms();
                let motor_fr_current_timestamp_ms = self.motor_fr.read_current_timestamp_ms();
                // Log which ones have the same timestamp as before.
                if motor_fl_current_timestamp_ms == motor_fl_last_timestamp_ms {
                    defmt::warn!("FL motor responded with the same timestamp.");
                }

                if motor_bl_current_timestamp_ms == motor_bl_last_timestamp_ms {
                    defmt::warn!("BL motor responded with the same timestamp.");
                }

                if motor_br_current_timestamp_ms == motor_br_last_timestamp_ms {
                    defmt::warn!("BR motor responded with the same timestamp.");
                }

                if motor_fr_current_timestamp_ms == motor_fr_last_timestamp_ms {
                    defmt::warn!("FR motor responded with the same timestamp.");
                }

                motor_fl_last_timestamp_ms = motor_fl_current_timestamp_ms;
                motor_bl_last_timestamp_ms = motor_bl_current_timestamp_ms;
                motor_br_last_timestamp_ms = motor_br_current_timestamp_ms;
                motor_fr_last_timestamp_ms = motor_fr_current_timestamp_ms;

                // self.motor_fl.log_reset("FL");
                // self.motor_bl.log_reset("BL");
                // self.motor_br.log_reset("BR");
                // self.motor_fr.log_reset("FR");

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
                    //defmt::warn!("ticks since packet lockout");
                }

                // now we have setpoint r(t) in self.cmd_vel
                while let Some(battery_v) = self.battery_subscriber.try_next_message_pure() {
                    self.last_battery_v = battery_v;
                }

                while let Some(gyro_rads) = self.gyro_subscriber.try_next_message_pure() {
                    self.last_gyro_x_rads = gyro_rads[0];
                    self.last_gyro_y_rads = gyro_rads[1];
                    self.last_gyro_z_rads = gyro_rads[2];
                }

                while let Some(accel_ms) = self.accel_subscriber.try_next_message_pure() {
                    self.last_accel_x_ms = accel_ms[0];
                    self.last_accel_y_ms = accel_ms[1];
                    self.last_accel_z_ms = accel_ms[2];
                }

                let controls_enabled = false;

                // let kill_vel = self.shared_robot_state.get_battery_low() || self.shared_robot_state.get_battery_crit() || self.shared_robot_state.shutdown_requested();
                let kill_vel = false;
                let wheel_vels = if !kill_vel {
                    self.do_control_update(&mut robot_controller, cmd_vel, self.last_gyro_z_rads, controls_enabled)
                } else {
                    // Battery is too low, set velocity to zero
                    defmt::warn!("CT - low battery / shutting down command lockout");
                    Vector4::new(0.0, 0.0, 0.0, 0.0)
                };

                self.motor_fl.set_setpoint(wheel_vels[0]);
                self.motor_bl.set_setpoint(wheel_vels[1]);
                self.motor_br.set_setpoint(wheel_vels[2]);
                self.motor_fr.set_setpoint(wheel_vels[3]);

                defmt::info!("MEASURED_CUR: FL: {}, BL: {}, BR: {}, FR: {}",
                    (self.motor_fl.read_current() * 1000.0) as i32,
                    (self.motor_bl.read_current() * 1000.0) as i32,
                    (self.motor_br.read_current() * 1000.0) as i32,
                    (self.motor_fr.read_current() * 1000.0) as i32);

                //defmt::info!("TARGET_VEL: {}, MEASURED_VEL: {}, MEASURED_CUR: {}, TARGET_TOR: {}, MEASURED_TOR: {}, COMP_TOR: {}, DC_TOR: {}, DC_VEL: {}",
                //    (self.motor_br.read_vel_setpoint() * 1000.0) as i32,
                //    (self.motor_br.read_rads() * 1000.0) as i32,
                //    (self.motor_br.read_current() * 1000.0) as i32,
                //    (self.motor_br.read_torque_setpoint() * 1000.0) as i32,
                //    (self.motor_br.read_torque_estimate() * 1000.0) as i32,
                //    (self.motor_br.read_torque_computed_nm() * 1000.0) as i32,
                //    (self.motor_br.read_torque_computed_duty() * 1000.0) as i32,
                //    (self.motor_br.read_vel_computed_duty() * 1000.0) as i32);

                //defmt::info!("stspin temp: {} {} {} {}", self.motor_fl.read_mcu_temperature(), self.motor_bl.read_mcu_temperature(), self.motor_br.read_mcu_temperature(), self.motor_fr.read_mcu_temperature());
                self.send_motor_commands_and_telemetry(
                    &mut robot_controller, self.last_battery_v);

                loop_rate_ticker.next().await;
            }
        }

        async fn motor_calibrate_task_entry(&mut self) {
            defmt::info!("Motor Calibrate task init.");

            // wait for the switch state to be read
            while !self.shared_robot_state.hw_init_state_valid() {
                Timer::after_millis(10).await;
            }

            self.flash_motor_firmware(
                self.shared_robot_state.hw_in_debug_mode()).await;

            embassy_futures::join::join4(
                self.motor_fl.leave_reset(),
                self.motor_bl.leave_reset(),
                self.motor_br.leave_reset(),
                self.motor_fr.leave_reset(),
            ).await;

            let mut loop_rate_ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_RATE_MS));

            while self.shared_robot_state.get_imu_inop() {
                defmt::info!("Waiting for IMU to be ready.");
                loop_rate_ticker.next().await;
            }

            self.motor_fl.set_telemetry_enabled(true);
            self.motor_bl.set_telemetry_enabled(true);
            self.motor_br.set_telemetry_enabled(true);
            self.motor_fr.set_telemetry_enabled(true);

            // Need to send one motion command to get telemetry started.
            self.motor_fl.send_motion_command();
            self.motor_bl.send_motion_command();
            self.motor_br.send_motion_command();
            self.motor_fr.send_motion_command();

            Timer::after_millis(10).await;

            let mut gyro_movement_detected;
            let mut gyro_stable_count = 0;
            defmt::info!("Waiting for gyro data to be settled.");
            loop {
                while let Some(gyro_rads) = self.gyro_subscriber.try_next_message_pure() {
                    self.last_gyro_x_rads = gyro_rads[0];
                    self.last_gyro_y_rads = gyro_rads[1];
                    self.last_gyro_z_rads = gyro_rads[2];
                }

                gyro_movement_detected = self.last_gyro_x_rads.abs() > MAX_STATIONARY_GYRO_RADS ||
                    self.last_gyro_y_rads.abs() > MAX_STATIONARY_GYRO_RADS ||
                    self.last_gyro_z_rads.abs() > MAX_STATIONARY_GYRO_RADS;

                if gyro_movement_detected {
                    defmt::info!("Gyro movement detected! Stop that! Rads: {}, {}, {}",
                        self.last_gyro_x_rads,
                        self.last_gyro_y_rads,
                        self.last_gyro_z_rads);
                    gyro_stable_count = 0;
                }
                else {
                    gyro_stable_count += 1;
                }

                if gyro_stable_count >= MIN_GYRO_STABLE_COUNT {
                    defmt::info!("Gyro data settled.");
                    break;
                }

                loop_rate_ticker.next().await;
            }

            defmt::info!("Starting motor calibration.");
            let mut motor_fl_current_offset: f32 = 0.0;
            let mut motor_bl_current_offset: f32 = 0.0;
            let mut motor_br_current_offset: f32 = 0.0;
            let mut motor_fr_current_offset: f32 = 0.0;
            let mut motor_average_count = 0;
            loop {
                self.motor_fl.process_packets();
                self.motor_bl.process_packets();
                self.motor_br.process_packets();
                self.motor_fr.process_packets();

                defmt::info!("Current timestamp ms: {}, {}, {}, {}",
                    self.motor_fl.read_current_timestamp_ms(),
                    self.motor_bl.read_current_timestamp_ms(),
                    self.motor_br.read_current_timestamp_ms(),
                    self.motor_fr.read_current_timestamp_ms());

                if self.motor_fl.read_rads().abs() > WHEEL_VELOCITY_STATIONARY_RADS_MAX ||
                    self.motor_bl.read_rads().abs() > WHEEL_VELOCITY_STATIONARY_RADS_MAX ||
                    self.motor_br.read_rads().abs() > WHEEL_VELOCITY_STATIONARY_RADS_MAX ||
                    self.motor_fr.read_rads().abs() > WHEEL_VELOCITY_STATIONARY_RADS_MAX {
                    defmt::info!("One or more motors are moving, waiting for them to stop. Vel: FL: {}, BL: {}, BR: {}, FR: {}",
                        self.motor_fl.read_rads(),
                        self.motor_bl.read_rads(),
                        self.motor_br.read_rads(),
                        self.motor_fr.read_rads());
                }
                else {
                    motor_fl_current_offset += self.motor_fl.read_current();
                    motor_bl_current_offset += self.motor_bl.read_current();
                    motor_br_current_offset += self.motor_br.read_current();
                    motor_fr_current_offset += self.motor_fr.read_current();
                    motor_average_count += 1;

                    if motor_average_count >= CURRENT_CALIBRATION_SAMPLES {
                        defmt::info!("Calibration complete. FL: {}, BL: {}, BR: {}, FR: {}",
                            motor_fl_current_offset / motor_average_count as f32,
                            motor_bl_current_offset / motor_average_count as f32,
                            motor_br_current_offset / motor_average_count as f32,
                            motor_fr_current_offset / motor_average_count as f32);

                        break;
                    }
                }

                loop_rate_ticker.next().await;
            }

            defmt::info!("Setting motor current offsets.");
            loop {
                // TODO
                loop_rate_ticker.next().await;
            }
        }

        async fn flash_motor_firmware(&mut self, debug: bool) {
            defmt::info!("flashing firmware");
            if debug {
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

                if had_motor_error {
                    defmt::error!("one or more motors failed to flash.")
                } else {
                    defmt::debug!("all motors flashed");
                }
            } else {
                let res = embassy_futures::join::join4(
                    self.motor_fl.load_default_firmware_image(),
                    self.motor_bl.load_default_firmware_image(),
                    self.motor_br.load_default_firmware_image(),
                    self.motor_fr.load_default_firmware_image(),
                )
                .await;

                let error_mask = res.0.is_err() as u8
                        | ((res.1.is_err() as u8) & 0x01) << 1
                        | ((res.2.is_err() as u8) & 0x01) << 2
                        | ((res.3.is_err() as u8) & 0x01) << 3;

                self.shared_robot_state.set_wheels_inop(error_mask);

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

#[embassy_executor::task]
async fn motor_calibrate_task_entry(mut control_task: ControlTask<MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>) {
    loop {
        control_task.motor_calibrate_task_entry().await;
        defmt::error!("Motor calibrate task returned");
    }
}

pub async fn start_control_task(
    control_task_spawner: Spawner,
    uart_queue_spawner: SendSpawner,
    robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    battery_subscriber: BatteryVoltSubscriber,
    gyro_subscriber: GyroDataSubscriber,
    accel_subscriber: AccelDataSubscriber,
    motor_fl_uart: MotorFLUart, motor_fl_rx_pin: MotorFLUartRxPin, motor_fl_tx_pin: MotorFLUartTxPin, motor_fl_rx_dma: MotorFLDmaRx, motor_fl_tx_dma: MotorFLDmaTx, motor_fl_boot0_pin: MotorFLBootPin, motor_fl_nrst_pin: MotorFLResetPin,
    motor_bl_uart: MotorBLUart, motor_bl_rx_pin: MotorBLUartRxPin, motor_bl_tx_pin: MotorBLUartTxPin, motor_bl_rx_dma: MotorBLDmaRx, motor_bl_tx_dma: MotorBLDmaTx, motor_bl_boot0_pin: MotorBLBootPin, motor_bl_nrst_pin: MotorBLResetPin,
    motor_br_uart: MotorBRUart, motor_br_rx_pin: MotorBRUartRxPin, motor_br_tx_pin: MotorBRUartTxPin, motor_br_rx_dma: MotorBRDmaRx, motor_br_tx_dma: MotorBRDmaTx, motor_br_boot0_pin: MotorBRBootPin, motor_br_nrst_pin: MotorBRResetPin,
    motor_fr_uart: MotorFRUart, motor_fr_rx_pin: MotorFRUartRxPin, motor_fr_tx_pin: MotorFRUartTxPin, motor_fr_rx_dma: MotorFRDmaRx, motor_fr_tx_dma: MotorFRDmaTx, motor_fr_boot0_pin: MotorFRBootPin, motor_fr_nrst_pin: MotorFRResetPin,
    do_motor_calibrate: bool,
) {
    let initial_motor_controller_uart_config = stm32_interface::get_bootloader_uart_config();

    //////////////////////////
    //  create motor uarts  //
    //////////////////////////

    let fl_uart = Uart::new(motor_fl_uart, motor_fl_rx_pin, motor_fl_tx_pin, SystemIrqs, motor_fl_tx_dma, motor_fl_rx_dma, initial_motor_controller_uart_config).unwrap();
    let bl_uart = Uart::new(motor_bl_uart, motor_bl_rx_pin, motor_bl_tx_pin, SystemIrqs, motor_bl_tx_dma, motor_bl_rx_dma, initial_motor_controller_uart_config).unwrap();
    let br_uart = Uart::new(motor_br_uart, motor_br_rx_pin, motor_br_tx_pin, SystemIrqs, motor_br_tx_dma, motor_br_rx_dma, initial_motor_controller_uart_config).unwrap();
    let fr_uart = Uart::new(motor_fr_uart, motor_fr_rx_pin, motor_fr_tx_pin, SystemIrqs, motor_fr_tx_dma, motor_fr_rx_dma, initial_motor_controller_uart_config).unwrap();

    //////////////////////////////////////////////
    //  register motor queues and DMA hardware  //
    //////////////////////////////////////////////

    FRONT_LEFT_IDLE_BUFFERED_UART.init();
    BACK_LEFT_IDLE_BUFFERED_UART.init();
    BACK_RIGHT_IDLE_BUFFERED_UART.init();
    FRONT_RIGHT_IDLE_BUFFERED_UART.init();

    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_LEFT, fl_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_LEFT, bl_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, BACK_RIGHT, br_uart);
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, FRONT_RIGHT, fr_uart);

    ////////////////////////////////
    //  create motor controllers  //
    ////////////////////////////////

    let motor_fl = WheelMotor::new_from_pins(&FRONT_LEFT_IDLE_BUFFERED_UART, FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(), FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(), motor_fl_boot0_pin, motor_fl_nrst_pin, WHEEL_FW_IMG);
    let motor_bl = WheelMotor::new_from_pins(&BACK_LEFT_IDLE_BUFFERED_UART, BACK_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(), BACK_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(), motor_bl_boot0_pin, motor_bl_nrst_pin, WHEEL_FW_IMG);
    let motor_br = WheelMotor::new_from_pins(&BACK_RIGHT_IDLE_BUFFERED_UART,  BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(), BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(), motor_br_boot0_pin, motor_br_nrst_pin, WHEEL_FW_IMG);
    let motor_fr = WheelMotor::new_from_pins(&FRONT_RIGHT_IDLE_BUFFERED_UART, FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(), FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(), motor_fr_boot0_pin, motor_fr_nrst_pin, WHEEL_FW_IMG);

    let control_task = ControlTask::new(
        robot_state, command_subscriber, telemetry_publisher, battery_subscriber,
        gyro_subscriber, accel_subscriber, motor_fl, motor_bl, motor_br, motor_fr);

    if !do_motor_calibrate {
        defmt::info!("Control task starting!");
        control_task_spawner.spawn(control_task_entry(control_task)).unwrap();
    }
    else {
        defmt::info!("Motor calibration starting!");
        control_task_spawner.spawn(motor_calibrate_task_entry(control_task)).unwrap();
    }
}