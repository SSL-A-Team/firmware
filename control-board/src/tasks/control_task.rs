use ateam_common_packets::{
    bindings::{BasicControl, BasicTelemetry, CcmMotionControlType, KickerTelemetry, MotionCommandType, PowerTelemetry},
    radio::TelemetryPacket,
};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart,
};
use ateam_controls::{Vector3f, Vector4f};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{usart::Uart, Peri};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::{
    include_external_cpp_bin,
    motion::robot_controller::BodyController,
    parameter_interface::ParameterInterface,
    pins::*,
    robot_state::{RobotState, SharedRobotState},
    motor::CurrentControlledMotor,
    SystemIrqs, DEBUG_MOTOR_UART_QUEUES, ROBOT_VERSION_MAJOR, ROBOT_VERSION_MINOR,
};

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel-torque.bin"}

const MAX_TX_PACKET_SIZE: usize = 80;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 80;
const RX_BUF_DEPTH: usize = 20;

type ControlWheelMotor =
    CurrentControlledMotor<'static, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH, DEBUG_MOTOR_UART_QUEUES>;
static_idle_buffered_uart!(FRONT_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_MOTOR_UART_QUEUES, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_LEFT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_MOTOR_UART_QUEUES, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(BACK_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_MOTOR_UART_QUEUES, #[link_section = ".axisram.buffers"]);
static_idle_buffered_uart!(FRONT_RIGHT, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, DEBUG_MOTOR_UART_QUEUES, #[link_section = ".axisram.buffers"]);

const TICKS_WITHOUT_PACKET_STOP: usize = 200;
const TICKS_BASIC_TELEM_INTERVAL: usize = 20;  // send basic telem every 10 ticks (100 Hz if loop is 1 kHz)
const TICKS_EXTENDED_TELEM_INTERVAL: usize = 20;  // send extended telem every 20 ticks (50 Hz if loop is 1 kHz)
// const TICKS_TRACE_PRINT: usize = 1000;  // print trace every 1000 ticks (1 second if loop is 1 kHz)
const TICKS_TRACE_PRINT: usize = 100;  // print trace every 100 ticks (0.1 second if loop is 1 kHz)

#[macro_export]
macro_rules! create_control_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $robot_state:ident,
        $control_command_subscriber:ident, $control_telemetry_publisher:ident,
        $power_telemetry_subscriber:ident, $kicker_telemetry_subscriber:ident,
        $control_gyro_data_subscriber:ident, $control_accel_data_subscriber:ident,
        $p:ident
    ) => {
        ateam_control_board::tasks::control_task::start_control_task(
            $main_spawner,
            $uart_queue_spawner,
            $robot_state,
            $control_command_subscriber,
            $control_telemetry_publisher,
            $control_gyro_data_subscriber,
            $control_accel_data_subscriber,
            $power_telemetry_subscriber,
            $kicker_telemetry_subscriber,
            $p.UART7,
            $p.PF6,
            $p.PF7,
            $p.DMA1_CH1,
            $p.DMA1_CH0,
            $p.PF5,
            $p.PF4,
            $p.USART10,
            $p.PE2,
            $p.PE3,
            $p.DMA1_CH3,
            $p.DMA1_CH2,
            $p.PE5,
            $p.PE4,
            $p.USART6,
            $p.PC7,
            $p.PC6,
            $p.DMA1_CH5,
            $p.DMA1_CH4,
            $p.PG7,
            $p.PG8,
            $p.USART3,
            $p.PD9,
            $p.PD8,
            $p.DMA1_CH7,
            $p.DMA1_CH6,
            $p.PB12,
            $p.PB13,
        )
        .await;
    };
}

pub struct ControlTask<
    const MAX_RX_PACKET_SIZE: usize,
    const MAX_TX_PACKET_SIZE: usize,
    const RX_BUF_DEPTH: usize,
    const TX_BUF_DEPTH: usize,
> {
    shared_robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    gyro_subscriber: GyroDataSubscriber,
    accel_subscriber: AccelDataSubscriber,
    power_telemetry_subscriber: PowerTelemetrySubscriber,
    kicker_telemetry_subscriber: KickerTelemetrySubscriber,

    last_gyro_rads: f32,
    last_accel_x_ms: f32,
    last_accel_y_ms: f32,
    last_command: BasicControl,
    last_power_telemetry: PowerTelemetry,
    last_kicker_telemetry: KickerTelemetry,

    ticks_since_extended_telem: usize,
    ticks_since_basic_telem: usize,

    motor_fl: ControlWheelMotor,
    motor_bl: ControlWheelMotor,
    motor_br: ControlWheelMotor,
    motor_fr: ControlWheelMotor,
}

impl<
        const MAX_RX_PACKET_SIZE: usize,
        const MAX_TX_PACKET_SIZE: usize,
        const RX_BUF_DEPTH: usize,
        const TX_BUF_DEPTH: usize,
    > ControlTask<MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH, TX_BUF_DEPTH>
{
    pub fn new(
        robot_state: &'static SharedRobotState,
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
            power_telemetry_subscriber,
            kicker_telemetry_subscriber,
            last_gyro_rads: 0.0,
            last_accel_x_ms: 0.0,
            last_accel_y_ms: 0.0,
            last_command: Default::default(),
            last_power_telemetry: Default::default(),
            last_kicker_telemetry: Default::default(),
            ticks_since_basic_telem: 0,
            ticks_since_extended_telem: 0,
            motor_fl: motor_fl,
            motor_bl: motor_bl,
            motor_br: motor_br,
            motor_fr: motor_fr,
        }
    }

    fn send_motor_commands_and_telemetry(
        &mut self,
        seq_number: u16,
        timestamp_us: u64,
        robot_controller: &mut BodyController,
        cur_state: RobotState,
    ) {
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
        let dribbler_motor_hall_error = self
            .last_kicker_telemetry
            .dribbler_motor
            .hall_disconnected_error() as u32;

        let basic_telem = TelemetryPacket::Basic(BasicTelemetry {
            control_data_sequence_number: seq_number as u8,
            transmission_sequence_number: 0,
            robot_revision_major: ROBOT_VERSION_MAJOR,
            robot_revision_minor: ROBOT_VERSION_MINOR,
            _bitfield_align_1: Default::default(),
            _bitfield_1: BasicTelemetry::new_bitfield_1(
                !self.last_power_telemetry.power_ok() as u32, // power error
                cur_state.power_inop as u32,                  // power board error
                !self.last_power_telemetry.battery_info.battery_ok() as u32, // battery error
                self.last_power_telemetry.battery_info.battery_low() as u32, // battery low
                self.last_power_telemetry.battery_info.battery_critical() as u32, // battery crit
                self.last_power_telemetry.shutdown_requested() as u32, // shutdown pending
                cur_state.robot_tipped as u32,                // tipped error
                self.last_kicker_telemetry.error_detected() as u32, // breakbeam error
                self.last_kicker_telemetry.ball_detected() as u32, // ball detected
                cur_state.imu_inop as u32,                    // accel 0 error
                false as u32,                                 // accel 1 error, uninstalled
                cur_state.imu_inop as u32,                    // gyro 0 error
                false as u32,                                 // gyro 1 error, uninstalled
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
                false as u32, // chipper available
                (!cur_state.kicker_inop && self.last_kicker_telemetry.error_detected() == 0) as u32,
                self.last_command.body_pose_control_enabled(),
                self.last_command.body_twist_control_enabled(),
                self.last_command.body_accel_control_enabled(),
                self.last_command.wheel_vel_control_enabled(),
                self.last_command.wheel_torque_control_enabled(),
                Default::default(),
            ),
            battery_percent: self.last_power_telemetry.battery_info.battery_pct as u16,
            kicker_charge_percent: self.last_kicker_telemetry.charge_pct,
        });

        self.ticks_since_basic_telem += 1;
        if cur_state.radio_bridge_ok && self.ticks_since_basic_telem >= TICKS_BASIC_TELEM_INTERVAL {
            self.telemetry_publisher.publish_immediate(basic_telem);
            self.ticks_since_basic_telem = 0;
        }

        let mut control_debug_telem = robot_controller.get_control_debug_telem();

        control_debug_telem.timestamp_us_lo = (timestamp_us & 0xFFFFFFFF) as u32;
        control_debug_telem.timestamp_us_hi = ((timestamp_us >> 32) & 0xFFFFFFFF) as u32;

        control_debug_telem.front_left_motor = self.motor_fl.get_latest_state();
        control_debug_telem.back_left_motor = self.motor_bl.get_latest_state();
        control_debug_telem.back_right_motor = self.motor_br.get_latest_state();
        control_debug_telem.front_right_motor = self.motor_fr.get_latest_state();

        control_debug_telem.imu_accel[0] = self.last_accel_x_ms;
        control_debug_telem.imu_accel[1] = self.last_accel_y_ms;

        // control_debug_telem.kicker_status = self.last_kicker_telemetry;
        // control_debug_telem.power_status = self.last_power_telemetry;

        // Send extended telemetry if vision update was received, or if the extended telemetry interval has elapsed
        let vision_update = control_debug_telem.vision_update() != 0;
        let control_debug_telem = TelemetryPacket::Extended(control_debug_telem);
        self.ticks_since_extended_telem += 1;
        if cur_state.radio_bridge_ok && (self.ticks_since_extended_telem >= TICKS_EXTENDED_TELEM_INTERVAL || vision_update) {
            self.telemetry_publisher
                .publish_immediate(control_debug_telem);
            self.ticks_since_extended_telem = 0;
        }
    }

    async fn control_task_entry(&mut self) {
        defmt::info!("control task init.");

        // wait for the switch state to be read
        while !self.shared_robot_state.hw_init_state_valid() {
            Timer::after_millis(10).await;
        }

        self.flash_motor_firmware(self.shared_robot_state.hw_in_debug_mode())
            .await;

        embassy_futures::join::join4(
            self.motor_fl.leave_reset(),
            self.motor_bl.leave_reset(),
            self.motor_br.leave_reset(),
            self.motor_fr.leave_reset(),
        )
        .await;

        self.motor_fl.set_telemetry_enabled(true);
        self.motor_bl.set_telemetry_enabled(true);
        self.motor_br.set_telemetry_enabled(true);
        self.motor_fr.set_telemetry_enabled(true);

        Timer::after_millis(10).await;

        let mut ctrl_seq_number = 0;
        let loop_period = Duration::from_millis(1);  // 1 kHz
        let mut loop_rate_ticker = Ticker::every(loop_period);

        let mut robot_controller = BodyController::new(loop_period);

        let mut cmd = Vector3f::default();
        let mut last_vision_pose_meas = Vector3f::default();
        let mut vision_update = false;
        let mut ticks_since_control_packet = 0;

        // //////////////////////// Frequency Measurement Vars //////////////////////////
        // let mut loop_ticks_since_freqeuncy_measurement = 0;
        // let mut frequency_measurement_time_elapsed_sum_ms: f32 = 0.;
        // let frequency_measurement_window_length = 60;
        // let mut last_frequency_measurement_time = Instant::now();
        // //////////////////////////////////////////////////////////////////////////////

        let mut last_loop_start_time = Instant::now();
        let mut last_loop_term_time = Instant::now();
        let mut ticks_since_trace_print = 0;
        let mut t_us_at_loop_start = 0u64;

        loop {
            t_us_at_loop_start += (Instant::now() - last_loop_start_time).as_micros() as u64;
            last_loop_start_time = Instant::now();
            let mut start = last_loop_start_time;
            let loop_invocation_dead_time = last_loop_start_time - last_loop_term_time;
            if loop_invocation_dead_time > Duration::from_micros(1100) {
                defmt::warn!("control loop scheuling lagged. Expected ~1ms between loop invocations, but got {:?}us", loop_invocation_dead_time.as_micros());
            }

            self.motor_fl.process_packets();
            self.motor_bl.process_packets();
            self.motor_br.process_packets();
            self.motor_fr.process_packets();

            let motor_packet_process_time = Instant::now() - start;
            start = Instant::now();

            let cur_state = self.shared_robot_state.get_state();

            // self.motor_fl.log_reset("FL");
            // self.motor_bl.log_reset("BL");
            // self.motor_br.log_reset("BR");
            // self.motor_fr.log_reset("FR");

            ticks_since_control_packet += 1;
            while let Some(latest_packet) = self.command_subscriber.try_next_message_pure() {
                match latest_packet {
                    ateam_common_packets::radio::DataPacket::BasicControl(latest_control) => {
                        // //////////////////////// Loop Rate Measurement ///////////////////////////////
                        // let frequency_measurement_loop_time_elapsed = ((Instant::now() - last_frequency_measurement_time).as_micros() as f32) / 1000.0;
                        // frequency_measurement_time_elapsed_sum_ms += frequency_measurement_loop_time_elapsed;
                        // if loop_ticks_since_freqeuncy_measurement == frequency_measurement_window_length {
                        //     let frequency: f32 = loop_ticks_since_freqeuncy_measurement as f32 / (frequency_measurement_time_elapsed_sum_ms / 1000.0);
                        //     defmt::debug!("Command RX Frequency - {} hz", frequency);
                        //     frequency_measurement_time_elapsed_sum_ms = 0.;
                        //     loop_ticks_since_freqeuncy_measurement = 0;
                        // }
                        // last_frequency_measurement_time = Instant::now();
                        // loop_ticks_since_freqeuncy_measurement += 1;
                        // //////////////////////////////////////////////////////////////////////////////

                        if latest_control.reboot_robot() != 0 {
                            loop {
                                cortex_m::peripheral::SCB::sys_reset();
                            }
                        }

                        cmd = Vector3f::new(
                            latest_control.x_linear_cmd,
                            latest_control.y_linear_cmd,
                            latest_control.z_angular_cmd,
                        );
                        last_vision_pose_meas = Vector3f::new(
                            latest_control.pose_x_linear_vision,
                            latest_control.pose_y_linear_vision,
                            latest_control.pose_z_angular_vision,
                        );
                        vision_update = latest_control.vision_update() != 0;

                        if latest_control.request_shutdown() != 0 {
                            self.shared_robot_state.flag_shutdown_requested();
                        }

                        let wheel_motion_type = match (
                            self.last_command.wheel_vel_control_enabled() != 0,
                            self.last_command.wheel_torque_control_enabled() != 0,
                        ) {
                            (true, true) => CcmMotionControlType::CCM_MCT_VELOCITY_CURRENT,
                            (true, false) => CcmMotionControlType::CCM_MCT_VELOCITY,
                            (false, true) => CcmMotionControlType::CCM_MCT_CURRENT,
                            (false, false) => CcmMotionControlType::CCM_MCT_MOTOR_OFF,
                        };

                        self.motor_fl.set_motion_type(wheel_motion_type);
                        self.motor_bl.set_motion_type(wheel_motion_type);
                        self.motor_br.set_motion_type(wheel_motion_type);
                        self.motor_fr.set_motion_type(wheel_motion_type);

                        let motion_enabled = wheel_motion_type != CcmMotionControlType::CCM_MCT_MOTOR_OFF;
                        self.motor_fl.set_motion_enabled(motion_enabled);
                        self.motor_bl.set_motion_enabled(motion_enabled);
                        self.motor_br.set_motion_enabled(motion_enabled);
                        self.motor_fr.set_motion_enabled(motion_enabled);

                        self.last_command = latest_control;
                        ticks_since_control_packet = 0;
                    }
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
                    }
                }
            }

            let command_packet_process_time = Instant::now() - start;
            start = Instant::now();

            let wheel_vel_meas = Vector4f::new(
                self.motor_fl.read_rads(),
                self.motor_bl.read_rads(),
                self.motor_br.read_rads(),
                self.motor_fr.read_rads(),
            );

            while let Some(gyro_rads) = self.gyro_subscriber.try_next_message_pure() {
                self.last_gyro_rads = gyro_rads[2];
            }

            while let Some(accel_ms) = self.accel_subscriber.try_next_message_pure() {
                self.last_accel_x_ms = accel_ms[0];
                self.last_accel_y_ms = accel_ms[1];
            }

            while let Some(kicker_telemetry) =
                self.kicker_telemetry_subscriber.try_next_message_pure()
            {
                self.last_kicker_telemetry = kicker_telemetry;
            }

            while let Some(power_telemetry) =
                self.power_telemetry_subscriber.try_next_message_pure()
            {
                self.last_power_telemetry = power_telemetry;
            }

            if self.last_command.game_state_in_stop() != 0 {
                // TODO impl 1.5m/s clamping or something
            }

            robot_controller.control_update(
                cmd,
                self.last_command.body_pose_control_enabled() != 0,
                self.last_command.body_twist_control_enabled() != 0,
                self.last_command.body_accel_control_enabled() != 0,
                last_vision_pose_meas,
                vision_update,
                wheel_vel_meas,
                self.last_gyro_rads,
                ticks_since_trace_print >= TICKS_TRACE_PRINT,
            );
            vision_update = false;  // reset vision update flag after use

            let mut wheel_torque_cmd = Vector4f::default();
            let mut wheel_vel_cmd = Vector4f::default();
            if self.stop_wheels() || ticks_since_control_packet >= TICKS_WITHOUT_PACKET_STOP {
                // if ticks_since_trace_print >= TICKS_TRACE_PRINT {
                //     defmt::warn!("control task - motor commands locked out");
                // }
                defmt::warn!("control task - motor commands locked out");
            } else {
                wheel_torque_cmd = robot_controller.get_wheel_torques();
                wheel_vel_cmd = robot_controller.get_wheel_velocities();
            }
            
            ////////////////// TODO: Move this/delete this //////////////////////

            let TORQUE_CONSTANT = 0.0335; // Nm/A
            let mut wheel_ma = wheel_torque_cmd / TORQUE_CONSTANT * 1000.0;

            let MAX_CURRENT_MA = 750.0; // mA
            // TODO: remove this safety clamp after testing
            if wheel_ma.x.abs() > MAX_CURRENT_MA || wheel_ma.y.abs() > MAX_CURRENT_MA || wheel_ma.z.abs() > MAX_CURRENT_MA || wheel_ma.w.abs() > MAX_CURRENT_MA {
                defmt::warn!(
                    "high wheel current command detected: {} {} {} {}",
                    wheel_ma.x as i16,
                    wheel_ma.y as i16,
                    wheel_ma.z as i16,
                    wheel_ma.w as i16
                );
                wheel_ma = Vector4f::default();
            }

            // if ticks_since_trace_print >= TICKS_TRACE_PRINT {
            //     defmt::info!(
            //         "wheel vel cmd: {} {} {} {}",
            //         wheel_vel_cmd.x,
            //         wheel_vel_cmd.y,
            //         wheel_vel_cmd.z,
            //         wheel_vel_cmd.w,
            //     );
            // }
            // if ticks_since_trace_print >= TICKS_TRACE_PRINT {
            //     defmt::info!(
            //         "wheel_ma cmd: {} {} {} {}",
            //         wheel_ma.x as i16,
            //         wheel_ma.y as i16,
            //         wheel_ma.z as i16,
            //         wheel_ma.w as i16,
            //     );
            // }

            /////////////////////////////////////////////////////////???????????

            self.motor_fl.set_setpoint(wheel_vel_cmd.x);
            self.motor_bl.set_setpoint(wheel_vel_cmd.y);
            self.motor_br.set_setpoint(wheel_vel_cmd.z);
            self.motor_fr.set_setpoint(wheel_vel_cmd.w);

            self.motor_fl.set_current_setpoint(wheel_ma.x as i16);
            self.motor_bl.set_current_setpoint(wheel_ma.y as i16);
            self.motor_br.set_current_setpoint(wheel_ma.z as i16);
            self.motor_fr.set_current_setpoint(wheel_ma.w as i16);
            // self.motor_fl.set_current_setpoint(0);
            // self.motor_bl.set_current_setpoint(0);
            // self.motor_br.set_current_setpoint(0);
            // self.motor_fr.set_current_setpoint(0);

            let control_update_time = Instant::now() - start;
            start = Instant::now();

            ///////////////////////////////////
            //  send commands and telemetry  //
            ///////////////////////////////////

            let timestamp_us = t_us_at_loop_start + (Instant::now() - last_loop_start_time).as_micros() as u64;
            self.send_motor_commands_and_telemetry(
                ctrl_seq_number,
                timestamp_us,
                &mut robot_controller,
                cur_state,
            );

            // increment seq number
            ctrl_seq_number = (ctrl_seq_number + 1) & 0x00FF;

            let channel_update_time = Instant::now() - start;
            start = Instant::now();

            let loop_execution_time_us = Instant::now().duration_since(last_loop_start_time).as_micros();
            if ticks_since_trace_print > TICKS_TRACE_PRINT || loop_execution_time_us > 300 {
                defmt::trace!(
                    "control loop trace: motor_pkt_proc: {} us, cmd_pkt_proc: {} us, control_update: {} us, publish: {} us",
                    motor_packet_process_time.as_micros(),
                    command_packet_process_time.as_micros(),
                    control_update_time.as_micros(),
                    channel_update_time.as_micros(),
                );
                defmt::trace!("TOTAL CONTROL LOOP EXECUTION TIME: {} us", loop_execution_time_us);
            }

            if loop_execution_time_us > 300 {
                defmt::warn!("control loop is taking >300us: {} us (it may be interrupted by higher priority tasks). This is >30% of an execution frame.", loop_execution_time_us);
            }

            if ticks_since_trace_print > TICKS_TRACE_PRINT {
                ticks_since_trace_print = 0;
            }
            ticks_since_trace_print += 1;

            last_loop_term_time = Instant::now();
            loop_rate_ticker.next().await;
        }
    }

    async fn flash_motor_firmware(&mut self, debug: bool) {
        defmt::info!("flashing firmware");
        let force_flash = debug;
        if debug {
            let mut had_motor_error = false;
            if self
                .motor_fl
                .init_default_firmware_image(force_flash)
                .await
                .is_err()
            {
                defmt::error!("failed to flash FL");
                had_motor_error = true;
            } else {
                defmt::info!("FL flashed");
            }

            if self
                .motor_bl
                .init_default_firmware_image(force_flash)
                .await
                .is_err()
            {
                defmt::error!("failed to flash BL");
                had_motor_error = true;
            } else {
                defmt::info!("BL flashed");
            }

            if self
                .motor_br
                .init_default_firmware_image(force_flash)
                .await
                .is_err()
            {
                defmt::error!("failed to flash BR");
                had_motor_error = true;
            } else {
                defmt::info!("BR flashed");
            }

            if self
                .motor_fr
                .init_default_firmware_image(force_flash)
                .await
                .is_err()
            {
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
                self.motor_fl.init_default_firmware_image(force_flash),
                self.motor_bl.init_default_firmware_image(force_flash),
                self.motor_br.init_default_firmware_image(force_flash),
                self.motor_fr.init_default_firmware_image(force_flash),
            )
            .await;

            let error_mask = res.0.is_err() as u8
                | ((res.1.is_err() as u8) & 0x01) << 1
                | ((res.2.is_err() as u8) & 0x01) << 2
                | ((res.3.is_err() as u8) & 0x01) << 3;

            self.shared_robot_state.set_wheels_inop(error_mask);

            if error_mask != 0 {
                defmt::error!(
                    "failed to flash drive motor (FL, BL, BR, FR, DRIB): {}",
                    res
                );
            } else {
                defmt::debug!("motor firmware flashed");
            }
        }
    }

    // fn get_robot_model(&mut self) -> motion::robot_model::RobotModel {
    //     let robot_model_constants: RobotConstants = RobotConstants {
    //         wheel_angles_rad: Vector4::new(
    //             WHEEL_ANGLES_DEG[0].to_radians(),
    //             WHEEL_ANGLES_DEG[1].to_radians(),
    //             WHEEL_ANGLES_DEG[2].to_radians(),
    //             WHEEL_ANGLES_DEG[3].to_radians(),
    //         ),
    //         wheel_radius_m: Vector4::new(
    //             WHEEL_RADIUS_M,
    //             WHEEL_RADIUS_M,
    //             WHEEL_RADIUS_M,
    //             WHEEL_RADIUS_M,
    //         ),
    //         wheel_dist_to_cent_m: Vector4::new(
    //             WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
    //             WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
    //             WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
    //             WHEEL_DISTANCE_TO_ROBOT_CENTER_M,
    //         ),
    //     };

    //     let robot_model: RobotModel = RobotModel::new(robot_model_constants);

    //     return robot_model;
    // }

    fn stop_wheels(&self) -> bool {
        // defmt::debug!("hco: {}, sd req: {}, estop: {}", self.last_power_telemetry.high_current_operations_allowed() == 0, self.shared_robot_state.shutdown_requested(), self.last_command.emergency_stop() != 0);

        // self.last_power_telemetry.high_current_operations_allowed() == 0
        self.shared_robot_state.shutdown_requested() || self.last_command.emergency_stop() != 0
    }
}

#[embassy_executor::task]
async fn control_task_entry(
    mut control_task: ControlTask<
        MAX_RX_PACKET_SIZE,
        MAX_TX_PACKET_SIZE,
        RX_BUF_DEPTH,
        TX_BUF_DEPTH,
    >,
) {
    loop {
        control_task.control_task_entry().await;
        defmt::error!("control task returned");
    }
}

pub async fn start_control_task(
    control_task_spawner: Spawner,
    uart_queue_spawner: SendSpawner,
    robot_state: &'static SharedRobotState,
    command_subscriber: CommandsSubscriber,
    telemetry_publisher: TelemetryPublisher,
    gyro_subscriber: GyroDataSubscriber,
    accel_subscriber: AccelDataSubscriber,
    power_telemetry_subscriber: PowerTelemetrySubscriber,
    kicker_telemetry_subscriber: KickerTelemetrySubscriber,
    motor_fl_uart: Peri<'static, MotorFLUart>,
    motor_fl_rx_pin: Peri<'static, MotorFLUartRxPin>,
    motor_fl_tx_pin: Peri<'static, MotorFLUartTxPin>,
    motor_fl_rx_dma: Peri<'static, MotorFLDmaRx>,
    motor_fl_tx_dma: Peri<'static, MotorFLDmaTx>,
    motor_fl_boot0_pin: Peri<'static, MotorFLBootPin>,
    motor_fl_nrst_pin: Peri<'static, MotorFLResetPin>,
    motor_bl_uart: Peri<'static, MotorBLUart>,
    motor_bl_rx_pin: Peri<'static, MotorBLUartRxPin>,
    motor_bl_tx_pin: Peri<'static, MotorBLUartTxPin>,
    motor_bl_rx_dma: Peri<'static, MotorBLDmaRx>,
    motor_bl_tx_dma: Peri<'static, MotorBLDmaTx>,
    motor_bl_boot0_pin: Peri<'static, MotorBLBootPin>,
    motor_bl_nrst_pin: Peri<'static, MotorBLResetPin>,
    motor_br_uart: Peri<'static, MotorBRUart>,
    motor_br_rx_pin: Peri<'static, MotorBRUartRxPin>,
    motor_br_tx_pin: Peri<'static, MotorBRUartTxPin>,
    motor_br_rx_dma: Peri<'static, MotorBRDmaRx>,
    motor_br_tx_dma: Peri<'static, MotorBRDmaTx>,
    motor_br_boot0_pin: Peri<'static, MotorBRBootPin>,
    motor_br_nrst_pin: Peri<'static, MotorBRResetPin>,
    motor_fr_uart: Peri<'static, MotorFRUart>,
    motor_fr_rx_pin: Peri<'static, MotorFRUartRxPin>,
    motor_fr_tx_pin: Peri<'static, MotorFRUartTxPin>,
    motor_fr_rx_dma: Peri<'static, MotorFRDmaRx>,
    motor_fr_tx_dma: Peri<'static, MotorFRDmaTx>,
    motor_fr_boot0_pin: Peri<'static, MotorFRBootPin>,
    motor_fr_nrst_pin: Peri<'static, MotorFRResetPin>,
) {
    let initial_motor_controller_uart_conifg = stm32_interface::get_bootloader_uart_config();

    //////////////////////////
    //  create motor uarts  //
    //////////////////////////

    let fl_uart = Uart::new(
        motor_fl_uart,
        motor_fl_rx_pin,
        motor_fl_tx_pin,
        SystemIrqs,
        motor_fl_tx_dma,
        motor_fl_rx_dma,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();
    let bl_uart = Uart::new(
        motor_bl_uart,
        motor_bl_rx_pin,
        motor_bl_tx_pin,
        SystemIrqs,
        motor_bl_tx_dma,
        motor_bl_rx_dma,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();
    let br_uart = Uart::new(
        motor_br_uart,
        motor_br_rx_pin,
        motor_br_tx_pin,
        SystemIrqs,
        motor_br_tx_dma,
        motor_br_rx_dma,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();
    let fr_uart = Uart::new(
        motor_fr_uart,
        motor_fr_rx_pin,
        motor_fr_tx_pin,
        SystemIrqs,
        motor_fr_tx_dma,
        motor_fr_rx_dma,
        initial_motor_controller_uart_conifg,
    )
    .unwrap();

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

    let motor_fl = ControlWheelMotor::new_from_pins(
        &FRONT_LEFT_IDLE_BUFFERED_UART,
        FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
        FRONT_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
        motor_fl_boot0_pin.into(),
        motor_fl_nrst_pin.into(),
        WHEEL_FW_IMG,
    );
    let motor_bl = ControlWheelMotor::new_from_pins(
        &BACK_LEFT_IDLE_BUFFERED_UART,
        BACK_LEFT_IDLE_BUFFERED_UART.get_uart_read_queue(),
        BACK_LEFT_IDLE_BUFFERED_UART.get_uart_write_queue(),
        motor_bl_boot0_pin.into(),
        motor_bl_nrst_pin.into(),
        WHEEL_FW_IMG,
    );
    let motor_br = ControlWheelMotor::new_from_pins(
        &BACK_RIGHT_IDLE_BUFFERED_UART,
        BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
        BACK_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
        motor_br_boot0_pin.into(),
        motor_br_nrst_pin.into(),
        WHEEL_FW_IMG,
    );
    let motor_fr = ControlWheelMotor::new_from_pins(
        &FRONT_RIGHT_IDLE_BUFFERED_UART,
        FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_read_queue(),
        FRONT_RIGHT_IDLE_BUFFERED_UART.get_uart_write_queue(),
        motor_fr_boot0_pin.into(),
        motor_fr_nrst_pin.into(),
        WHEEL_FW_IMG,
    );

    let control_task = ControlTask::new(
        robot_state,
        command_subscriber,
        telemetry_publisher,
        gyro_subscriber,
        accel_subscriber,
        power_telemetry_subscriber,
        kicker_telemetry_subscriber,
        motor_fl,
        motor_bl,
        motor_br,
        motor_fr,
    );

    control_task_spawner
        .spawn(control_task_entry(control_task))
        .unwrap();
}
