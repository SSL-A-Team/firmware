use ateam_common_packets::{bindings::{BasicControl, BasicTelemetry, KickerTelemetry, MotionCommandType, PowerTelemetry}, radio::TelemetryPacket};
use ateam_lib_stm32::{drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Ticker, Timer};
use nalgebra::{Vector3, Vector4};

use crate::{include_external_cpp_bin, motion::{self, params::robot_physical_params::{
        WHEEL_ANGLES_DEG, WHEEL_DISTANCE_TO_ROBOT_CENTER_M, WHEEL_RADIUS_M
    }, robot_controller::BodyVelocityController, robot_model::{RobotConstants, RobotModel}}, parameter_interface::ParameterInterface, pins::*, robot_state::{RobotState, SharedRobotState}, stspin_motor::WheelMotor, SystemIrqs, ROBOT_VERSION_MAJOR, ROBOT_VERSION_MINOR};

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

const MAX_TX_PACKET_SIZE: usize = 60;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 60;
const RX_BUF_DEPTH: usize = 20;

type ControlWheelMotor = WheelMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>;
static_idle_buffered_uart!(FRONT_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(FRONT_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);

const TICKS_WITHOUT_PACKET_STOP: usize = 20;

// This is Gyro for all axes.
const MAX_STATIONARY_GYRO_RADS: f32 = 0.1; // rad/s
const MIN_GYRO_STABLE_COUNT: usize = 1000; // number of ticks to wait for gyro to stabilize
// This is Acceleration for x and y.
const MAX_STATIONARY_ACCEL_MS: f32 = 0.1; // m/s^2

const WHEEL_VELOCITY_STATIONARY_RADS_MAX: f32 = 0.1; // rad/s
const CURRENT_CALIBRATION_SAMPLES: usize = 1000; // number of samples to take for current calibration

const CONTROL_LOOP_RATE_MS: u64 = 10; // 100Hz
const CONTROL_LOOP_RATE_S: f32 = CONTROL_LOOP_RATE_MS as f32 / 1000.0;
const CONTROL_MOTION_TYPE: MotionCommandType::Type = MotionCommandType::OPEN_LOOP;

// Internal macro with do_motor_calibrate as a parameter
#[macro_export]
macro_rules! __create_control_task_internal {
    (
        $main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $control_command_subscriber:ident, $control_telemetry_publisher:ident,
        $power_telemetry_subscriber:ident, $kicker_telemetry_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident,
        $do_motor_calibrate:expr
    ) => {
        ateam_control_board::tasks::control_task::start_control_task(
            $main_spawner, $uart_queue_spawner,
            $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $power_telemetry_subscriber, $kicker_telemetry_subscriber,
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
        $power_telemetry_subscriber:ident, $kicker_telemetry_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident
    ) => {
        $crate::__create_control_task_internal!(
            $main_spawner, $uart_queue_spawner, $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $power_telemetry_subscriber, $kicker_telemetry_subscriber,
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
        $power_telemetry_subscriber:ident, $kicker_telemetry_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident
    ) => {
        $crate::__create_control_task_internal!(
            $main_spawner, $uart_queue_spawner, $robot_state,
            $control_command_subscriber, $control_telemetry_publisher,
            $power_telemetry_subscriber, $kicker_telemetry_subscriber,
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
    telemetry_publisher: TelemetryPublisher,
    gyro_subscriber: GyroDataSubscriber,
    accel_subscriber: AccelDataSubscriber,
    power_telemetry_subscriber: PowerTelemetrySubscriber,
    kicker_telemetry_subscriber: KickerTelemetrySubscriber,

    last_gyro_x_rads: f32,
    last_gyro_y_rads: f32,
    last_gyro_z_rads: f32,
    last_accel_x_ms: f32,
    last_accel_y_ms: f32,
    last_accel_z_ms: f32,
    last_command: BasicControl,
    last_power_telemetry: PowerTelemetry,
    last_kicker_telemetry: KickerTelemetry,

    motor_fl: ControlWheelMotor,
    motor_bl: ControlWheelMotor,
    motor_br: ControlWheelMotor,
    motor_fr: ControlWheelMotor,
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
                gyro_subscriber: GyroDataSubscriber,
                accel_subscriber: AccelDataSubscriber,
                power_telemetry_subscriber: PowerTelemetrySubscriber,
                kicker_telemetry_subscriber: KickerTelemetrySubscriber,
                motor_fl: ControlWheelMotor,
                motor_bl: ControlWheelMotor,
                motor_br: ControlWheelMotor,
                motor_fr: ControlWheelMotor,
        ) -> Self {
            ControlTask {
                shared_robot_state: robot_state,
                command_subscriber: command_subscriber,
                telemetry_publisher: telemetry_publisher,
                gyro_subscriber: gyro_subscriber,
                accel_subscriber: accel_subscriber,
                power_telemetry_subscriber: power_telemetry_subscriber,
                kicker_telemetry_subscriber: kicker_telemetry_subscriber,
                last_gyro_x_rads: 0.0,
                last_gyro_y_rads: 0.0,
                last_gyro_z_rads: 0.0,
                last_accel_x_ms: 0.0,
                last_accel_y_ms: 0.0,
                last_accel_z_ms: 0.0,
                last_command: Default::default(),
                last_power_telemetry: Default::default(),
                last_kicker_telemetry: Default::default(),
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
                                            seq_number: u16,
                                            robot_controller: &mut BodyVelocityController,
                                            cur_state: RobotState)
        {


            self.motor_fl.send_motion_command();
            self.motor_bl.send_motion_command();
            self.motor_br.send_motion_command();
            self.motor_fr.send_motion_command();

            let front_left_motor_error = self.motor_fl.read_is_error() as u32;
            let back_left_motor_error = self.motor_bl.read_is_error() as u32;
            let back_right_motor_error = self.motor_br.read_is_error() as u32;
            let front_right_motor_error = self.motor_fr.read_is_error() as u32;
            let dribbler_motor_error = self.last_kicker_telemetry.dribbler_motor.master_error() as u32;

            let front_left_hall_error = self.motor_fl.check_hall_error() as u32;
            let back_left_hall_error = self.motor_bl.check_hall_error() as u32;
            let back_right_hall_error = self.motor_br.check_hall_error() as u32;
            let front_right_hall_error = self.motor_fr.check_hall_error() as u32;
            let dribbler_motor_hall_error = self.last_kicker_telemetry.dribbler_motor.hall_disconnected_error() as u32;

            let basic_telem = TelemetryPacket::Basic(BasicTelemetry {
                sequence_number: seq_number,
                robot_revision_major: ROBOT_VERSION_MAJOR,
                robot_revision_minor: ROBOT_VERSION_MINOR,
                _bitfield_align_1: Default::default(),
                _bitfield_1: BasicTelemetry::new_bitfield_1(
                    !self.last_power_telemetry.power_ok() as u32,  // power error
                    cur_state.power_inop as u32,  // power board error
                    !self.last_power_telemetry.battery_info.battery_ok() as u32,  // battery error
                    self.last_power_telemetry.battery_info.battery_low() as u32,  // battery low
                    self.last_power_telemetry.battery_info.battery_critical() as u32,  // battery crit
                    self.last_power_telemetry.shutdown_requested() as u32,  // shutdown pending
                    cur_state.robot_tipped as u32,  // tipped error
                    self.last_kicker_telemetry.error_detected() as u32,  // breakbeam error
                    self.last_kicker_telemetry.ball_detected() as u32,  // ball detected
                    cur_state.imu_inop as u32,  // accel 0 error
                    false as u32,  // accel 1 error, uninstalled
                    cur_state.imu_inop as u32,  // gyro 0 error
                    false as u32,  // gyro 1 error, uninstalled
                    front_left_motor_error,
                    front_left_hall_error,
                    back_left_motor_error,
                    back_left_hall_error,
                    back_right_motor_error,
                    back_right_hall_error,
                    front_right_motor_error,
                    front_right_hall_error,
                    dribbler_motor_error,
                    dribbler_motor_hall_error,
                    self.last_kicker_telemetry.error_detected() as u32,
                    false as u32,  // chipper available
                    (!cur_state.kicker_inop && self.last_kicker_telemetry.error_detected() == 0) as u32,
                    self.last_command.body_vel_controls_enabled(),
                    self.last_command.wheel_vel_control_enabled(),
                    self.last_command.wheel_torque_control_enabled(),
                    Default::default()),
                battery_percent: self.last_power_telemetry.battery_info.battery_pct as u16,
                kicker_charge_percent: self.last_kicker_telemetry.charge_pct,
            });

            if cur_state.radio_bridge_ok {
                self.telemetry_publisher.publish_immediate(basic_telem);
            }

            let mut control_debug_telem = robot_controller.get_control_debug_telem();

            control_debug_telem.front_left_motor = self.motor_fl.get_latest_state();
            control_debug_telem.back_left_motor = self.motor_bl.get_latest_state();
            control_debug_telem.back_right_motor = self.motor_br.get_latest_state();
            control_debug_telem.front_right_motor = self.motor_fr.get_latest_state();

            control_debug_telem.imu_accel[0] = self.last_accel_x_ms;
            control_debug_telem.imu_accel[1] = self.last_accel_y_ms;

            control_debug_telem.kicker_status = self.last_kicker_telemetry;
            control_debug_telem.power_status = self.last_power_telemetry;

            let control_debug_telem = TelemetryPacket::Extended(control_debug_telem);
            if cur_state.radio_bridge_ok {
                self.telemetry_publisher.publish_immediate(control_debug_telem);
            }
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

            let mut seq_number = 0;
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

            Timer::after_millis(10).await;

            // Need to send one motion command to get telemetry started.
            self.motor_fl.send_motion_command();
            self.motor_bl.send_motion_command();
            self.motor_br.send_motion_command();
            self.motor_fr.send_motion_command();

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

                let cur_state = self.shared_robot_state.get_state();

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

                            if latest_control.reboot_robot() != 0 {
                                loop {
                                    cortex_m::peripheral::SCB::sys_reset();
                                }
                            }

                            if latest_control.request_shutdown() != 0 {
                                self.shared_robot_state.flag_shutdown_requested();
                            }

                            let wheel_motion_type = match (self.last_command.wheel_vel_control_enabled() != 0, self.last_command.wheel_torque_control_enabled() != 0) {
                                (true, true) => MotionCommandType::VELOCITY_W_TORQUE,
                                (true, false) => MotionCommandType::VELOCITY,
                                (false, true) => MotionCommandType::TORQUE,
                                (false, false) => MotionCommandType::OPEN_LOOP,
                            };

                            self.motor_fl.set_motion_type(wheel_motion_type);
                            self.motor_bl.set_motion_type(wheel_motion_type);
                            self.motor_br.set_motion_type(wheel_motion_type);
                            self.motor_fr.set_motion_type(wheel_motion_type);

                            self.last_command = latest_control;
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

                while let Some(kicker_telemetry) = self.kicker_telemetry_subscriber.try_next_message_pure() {
                    self.last_kicker_telemetry = kicker_telemetry;
                }

                while let Some(power_telemetry) = self.power_telemetry_subscriber.try_next_message_pure() {
                    self.last_power_telemetry = power_telemetry;
                }

                if self.stop_wheels() {
                    cmd_vel = Vector3::new(0.0, 0.0, 0.0);
                } else if self.last_command.game_state_in_stop() != 0 {
                    // TODO impl 1.5m/s clamping or something
                }

                let wheel_vels = if self.stop_wheels() {
                    defmt::warn!("control task - motor commands locked out");
                    Vector4::new(0.0, 0.0, 0.0, 0.0)
                } else {
                    let controls_enabled = self.last_command.body_vel_controls_enabled() != 0;
                    self.do_control_update(&mut robot_controller, cmd_vel, self.last_gyro_z_rads, controls_enabled)
                };

                self.motor_fl.set_setpoint(wheel_vels[0]);
                self.motor_bl.set_setpoint(wheel_vels[1]);
                self.motor_br.set_setpoint(wheel_vels[2]);
                self.motor_fr.set_setpoint(wheel_vels[3]);

                // defmt::info!("wheel vels: {} {} {} {}", self.motor_fl.read_encoder_delta(), self.motor_bl.read_encoder_delta(), self.motor_br.read_encoder_delta(), self.motor_fr.read_encoder_delta());
                defmt::info!("wheel curr: {} {} {} {}", self.motor_fl.read_current(), self.motor_bl.read_current(), self.motor_br.read_current(), self.motor_fr.read_current());
                defmt::info!("wheel curr offset: {} {} {} {}", self.motor_fl.read_vbus_voltage(), self.motor_bl.read_vbus_voltage(), self.motor_br.read_vbus_voltage(), self.motor_fr.read_vbus_voltage());

                ///////////////////////////////////
                //  send commands and telemetry  //
                ///////////////////////////////////

                self.send_motor_commands_and_telemetry(seq_number,
                    &mut robot_controller, cur_state);

                // increment seq number
                seq_number += 1;
                seq_number &= 0xFFF;

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

            self.motor_fl.set_calibrate_current(true);
            self.motor_bl.set_calibrate_current(true);
            self.motor_br.set_calibrate_current(true);
            self.motor_fr.set_calibrate_current(true);

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
                        motor_fl_current_offset /= motor_average_count as f32;
                        motor_bl_current_offset /= motor_average_count as f32;
                        motor_br_current_offset /= motor_average_count as f32;
                        motor_fr_current_offset /= motor_average_count as f32;

                        defmt::info!("Calibration complete. FL: {}, BL: {}, BR: {}, FR: {}",
                            motor_fl_current_offset,
                            motor_bl_current_offset,
                            motor_br_current_offset,
                            motor_fr_current_offset);

                        break;
                    }
                }

                loop_rate_ticker.next().await;
            }

            defmt::info!("Setting motor current offsets.");
            let mut motor_constant_error = false;


            if self.motor_fl.save_motor_current_constants(motor_fl_current_offset).await.is_err() {
                defmt::error!("Failed to save FL motor current constants");
                motor_constant_error = true;
            } else {
                defmt::info!("FL motor current constants saved.");
            }

            if self.motor_bl.save_motor_current_constants(motor_bl_current_offset).await.is_err() {
                defmt::error!("Failed to save BL motor current constants");
                motor_constant_error = true;
            } else {
                defmt::info!("BL motor current constants saved.");
            }

            if self.motor_br.save_motor_current_constants(motor_br_current_offset).await.is_err() {
                defmt::error!("Failed to save BR motor current constants");
                motor_constant_error = true;
            } else {
                defmt::info!("BR motor current constants saved.");
            }

            if self.motor_fr.save_motor_current_constants(motor_fr_current_offset).await.is_err() {
                defmt::error!("Failed to save FR motor current constants");
                motor_constant_error = true;
            } else {
                defmt::info!("FR motor current constants saved.");
            }

            if motor_constant_error {
                defmt::error!("One or more motors failed to save current constants. Run again!");
            } else {
                defmt::info!("All motors current constants saved.");
            }

            loop {
                loop_rate_ticker.next().await;
            }
        }

        async fn flash_motor_firmware(&mut self, debug: bool) {
            defmt::info!("flashing firmware");
            if debug {
                let mut had_motor_error = false;
                if self.motor_fl.load_default_firmware_image(debug).await.is_err() {
                    defmt::error!("failed to flash FL");
                    had_motor_error = true;
                } else {
                    defmt::info!("FL flashed");
                }

                if self.motor_bl.load_default_firmware_image(debug).await.is_err() {
                    defmt::error!("failed to flash BL");
                    had_motor_error = true;
                } else {
                    defmt::info!("BL flashed");
                }

                if self.motor_br.load_default_firmware_image(debug).await.is_err() {
                    defmt::error!("failed to flash BR");
                    had_motor_error = true;
                } else {
                    defmt::info!("BR flashed");
                }

                if self.motor_fr.load_default_firmware_image(debug).await.is_err() {
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
                    self.motor_fl.load_default_firmware_image(debug),
                    self.motor_bl.load_default_firmware_image(debug),
                    self.motor_br.load_default_firmware_image(debug),
                    self.motor_fr.load_default_firmware_image(debug),
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

        fn stop_wheels(&self) -> bool {
            // defmt::debug!("hco: {}, sd req: {}, estop: {}", self.last_power_telemetry.high_current_operations_allowed() == 0, self.shared_robot_state.shutdown_requested(), self.last_command.emergency_stop() != 0);

            self.last_power_telemetry.high_current_operations_allowed() == 0
            || self.shared_robot_state.shutdown_requested()
            || self.last_command.emergency_stop() != 0
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
    power_telemetry_subscriber: PowerTelemetrySubscriber,
    kicker_telemetry_subscriber: KickerTelemetrySubscriber,
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
        robot_state, command_subscriber, telemetry_publisher,
        gyro_subscriber, accel_subscriber,
        power_telemetry_subscriber, kicker_telemetry_subscriber,
        motor_fl, motor_bl, motor_br, motor_fr);

    if !do_motor_calibrate {
        defmt::info!("Control task starting!");
        control_task_spawner.spawn(control_task_entry(control_task)).unwrap();
    }
    else {
        defmt::info!("Motor calibration starting!");
        control_task_spawner.spawn(motor_calibrate_task_entry(control_task)).unwrap();
    }
}