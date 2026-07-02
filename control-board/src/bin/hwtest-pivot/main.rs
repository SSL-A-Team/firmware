#![no_std]
#![no_main]

//! Hardware test: pivot maneuver
//!
//! Mocks radio commands and repeatedly executes a pivot from 0° → 180°, then
//! 180° → 0°, with hold pauses between each leg.  No vision updates are needed
//! — pivot is heading-driven and works purely from gyro + encoder feedback.
//!
//! Phase sequence (loops forever):
//!   1. pivot 0° → 180°  (3 s)   — robot pivots CCW around the ball
//!   2. hold  at 180°    (1 s)   — robot stationary, held at target
//!   3. pivot 180° → 0°  (3 s)   — robot pivots CW back
//!   4. hold  at 0°      (1 s)   — robot stationary, held at start
//!
//! Button controls (take effect on the next command tick):
//!   Down  — increase orbit radius (+5 mm)
//!   Up    — decrease orbit radius (-5 mm)
//!   Right — increase inset angle (+0.1 rad)
//!   Left  — decrease inset angle (-0.1 rad)
//!   Enter — increase max angular velocity (+0.5 rad/s)
//!   Back  — decrease max angular velocity (-0.5 rad/s)
//!
//! The robot ID knob sets the dribbler current setpoint (ID × 0.01); ID 0 stops
//! all motion. Max angular acceleration is fixed at 2.0 rad/s². The tuned values
//! [orbit_radius, inset_angle, max_angular_vel, max_angular_acc] can be read back
//! over the radio via a parameter read of KF_PROCESS_STD (parameter 0).
//!
//! The ball is placed at the field origin (0, 0).

use ateam_common_packets::{
    bindings::{
        BasicControl, BodyControlCommand, BodyControlMode, DribblerCommand, HeadingPivotCommand,
        KickRequest, ParameterCommand, ParameterCommandCode, ParameterCommand_ParameterData,
        ParameterDataFormat, ParameterName,
    },
    radio::{DataPacket, TelemetryPacket},
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::{Input, Pull},
    interrupt,
    pac::Interrupt,
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;

use ateam_control_board::{
    create_audio_task, create_control_task, create_dotstar_task, create_imu_task, create_io_task,
    create_kicker_task, create_radio_task, get_system_config,
    pins::{
        AccelDataPubSub, CommandsPubSub, GyroDataPubSub, KickerTelemetryPubSub, LedCommandPubSub,
        PowerTelemetryPubSub, TelemetryPubSub,
    },
    robot_state::SharedRobotState,
};
use ateam_controls::pivot_trajectory::PivotParams;

// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
use panic_probe as _;
use static_cell::ConstStaticCell;

// ============================================================================
// Static resources
// ============================================================================

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
// Commands received over the radio flow into their own channel so they don't
// clash with the local pivot-sequencer commands on RADIO_C2_CHANNEL
// (CommandsPubSub only permits a single publisher).
static RADIO_C2_RX_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
// TelemetryPubSub only permits two publishers. The control task and the pivot
// loop's parameter-response publisher take those two slots on the radio channel,
// so IMU telemetry is routed to its own (unread) channel for this test.
static IMU_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();
static POWER_DATA_CHANNEL: PowerTelemetryPubSub = PubSubChannel::new();
static KICKER_DATA_CHANNEL: KickerTelemetryPubSub = PubSubChannel::new();
static LED_COMMAND_PUBSUB: LedCommandPubSub = PubSubChannel::new();

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

// ============================================================================
// Interrupt handlers for task executors
// ============================================================================

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn CORDIC() {
    RADIO_UART_QUEUE_EXECUTOR.on_interrupt();
}

// ============================================================================
// Pivot sequencer constants
// ============================================================================

/// Pivot step per leg, in degrees. The robot pivots this much clockwise each
/// leg, wrapping around and continuing in a circle forever. Configurable here.
const PIVOT_STEP_DEG: f32 = 90.0;

/// Main loop interval in milliseconds (100 Hz command rate).
const LOOP_INTERVAL_MS: u64 = 10;

/// Ticks spent executing each pivot leg (3 s at 100 Hz).
const EXEC_TICKS: u32 = 300;

/// Ticks spent holding position after each pivot completes (1 s at 100 Hz).
const HOLD_TICKS: u32 = 100;

/// Orbit radius adjustment per button press (meters).
const ORBIT_RADIUS_STEP: f32 = 0.005;
const ORBIT_RADIUS_MIN: f32 = 0.0;
const ORBIT_RADIUS_MAX: f32 = 0.5;

/// Inset angle adjustment per button press (radians).
const INSET_ANGLE_STEP: f32 = 0.1;
const INSET_ANGLE_MIN: f32 = -core::f32::consts::PI;
const INSET_ANGLE_MAX: f32 = core::f32::consts::PI;

/// Fixed max angular acceleration for the pivot (rad/s²).
const FIXED_MAX_ANGULAR_ACC: f32 = 2.0;

/// Max angular velocity adjustment per button press (rad/s).
/// Enter increases, Back decreases.
const ACC_STEP: f32 = 0.1;
const ACC_MIN: f32 = 0.5;
const ACC_MAX: f32 = 8.0 * core::f32::consts::PI;

/// Initial max angular velocity (rad/s), tunable at runtime via Enter/Back.
const DEFAULT_MAX_ANGULAR_ACC: f32 = 4.0 * core::f32::consts::PI; // rad/s

/// Existing robot parameter reused to read back the tuned pivot values over the
/// radio. A `PCC_READ` of this name is answered with a VEC4 payload carrying
/// `[orbit_radius, inset_angle, max_angular_vel, max_angular_acc]`.
const PIVOT_READBACK_PARAM: ParameterName::Type = ParameterName::KF_PROCESS_STD;

// ============================================================================
// Phase state machine
// ============================================================================

/// Pivot step per leg in radians (clockwise → negative).
fn pivot_step_rad() -> f32 {
    -PIVOT_STEP_DEG * core::f32::consts::PI / 180.0
}

/// Wrap an angle to (-π, π].
fn wrap_pi(a: f32) -> f32 {
    use core::f32::consts::PI;
    let pi2 = 2.0 * PI;
    let mut a = a % pi2;
    if a > PI {
        a -= pi2;
    } else if a <= -PI {
        a += pi2;
    }
    a
}

fn phase_duration_ticks(phase_idx: u32) -> u32 {
    if phase_idx % 2 == 0 {
        EXEC_TICKS
    } else {
        HOLD_TICKS
    }
}

// ============================================================================
// Entry point
// ============================================================================

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("hwtest-pivot: initialising");

    let robot_state = ROBOT_STATE.take();

    // ── executor pools ──────────────────────────────────────────────────────

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CORDIC,
        embassy_stm32::interrupt::Priority::P6,
    );
    let radio_uart_queue_spawner = RADIO_UART_QUEUE_EXECUTOR.start(Interrupt::CORDIC);

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P7,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    // ── buttons (active-low, polled at loop rate) ────────────────────────────

    let btn_up = Input::new(p.PE14, Pull::Up);
    let btn_down = Input::new(p.PE15, Pull::Up);
    let btn_left = Input::new(p.PE12, Pull::Up);
    let btn_right = Input::new(p.PE13, Pull::Up);
    let btn_enter = Input::new(p.PE11, Pull::Up); // increase max angular velocity
    let btn_back = Input::new(p.PE10, Pull::Up); // decrease max angular velocity

    // ── inter-task channels ──────────────────────────────────────────────────

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    let command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    // IMU telemetry goes to its own channel to leave a radio-telemetry publisher
    // slot free for the pivot loop's parameter responses.
    let imu_telemetry_publisher = IMU_TELEMETRY_CHANNEL.publisher().unwrap();
    // Telemetry publisher used by the pivot loop to answer parameter reads.
    let pivot_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

    // Radio receives commands into its own channel; the pivot loop consumes them
    // to answer parameter reads without disturbing the control task.
    let radio_command_publisher = RADIO_C2_RX_CHANNEL.publisher().unwrap();
    let mut radio_command_subscriber = RADIO_C2_RX_CHANNEL.subscriber().unwrap();
    let radio_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let imu_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    let control_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();
    let control_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();

    let control_task_power_telemetry_subscriber = POWER_DATA_CHANNEL.subscriber().unwrap();

    let kicker_board_telemetry_publisher = KICKER_DATA_CHANNEL.publisher().unwrap();
    let control_task_kicker_telemetry_subscriber = KICKER_DATA_CHANNEL.subscriber().unwrap();

    // ── spawn tasks ──────────────────────────────────────────────────────────

    create_io_task!(main_spawner, robot_state, p);
    create_dotstar_task!(main_spawner, led_command_subscriber, p);
    create_audio_task!(main_spawner, robot_state, p);

    create_imu_task!(
        main_spawner,
        robot_state,
        imu_gyro_data_publisher,
        imu_accel_data_publisher,
        imu_led_cmd_publisher,
        imu_telemetry_publisher,
        p
    );

    create_control_task!(
        main_spawner,
        uart_queue_spawner,
        robot_state,
        control_command_subscriber,
        control_telemetry_publisher,
        control_task_power_telemetry_subscriber,
        control_task_kicker_telemetry_subscriber,
        control_gyro_data_subscriber,
        control_accel_data_subscriber,
        p
    );

    create_kicker_task!(
        main_spawner,
        uart_queue_spawner,
        robot_state,
        kicker_command_subscriber,
        kicker_board_telemetry_publisher,
        p
    );

    create_radio_task!(
        main_spawner,
        radio_uart_queue_spawner,
        radio_uart_queue_spawner,
        robot_state,
        radio_command_publisher,
        radio_telemetry_subscriber,
        radio_led_cmd_publisher,
        wifi_credentials,
        p
    );

    // ── tunable parameters (adjusted via buttons) ────────────────────────────

    let mut orbit_radius: f32 = PivotParams::default().orbit_radius;
    let mut inset_angle: f32 = PivotParams::default().inset_angle;
    // Max angular velocity is tuned at runtime via Enter/Back; accel is fixed.
    // let mut max_angular_acc: f32 = PivotParams::default().max_accel_angular;
    let mut max_angular_acc: f32 = 7.0;

    defmt::info!(
        "hwtest-pivot: orbit_radius = {} m, inset_angle = {} rad, max_angular_acc = {} rad/s/s",
        orbit_radius,
        inset_angle,
        max_angular_acc,
    );

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── pivot sequencer loop ─────────────────────────────────────────────────

    let mut phase_idx: u32 = 0;
    let mut phase_tick: u32 = 0;

    // Running pivot target, advanced one clockwise step at the start of each
    // exec leg. Seeded one step ahead so the first leg pivots immediately.
    let mut target_theta: f32 = wrap_pi(pivot_step_rad());

    // Throttle counter for periodic parameter logging (1 Hz at 100 Hz loop).
    let mut print_tick: u32 = 0;

    // Previous button states for falling-edge detection (true = not pressed).
    let mut prev_up = true;
    let mut prev_down = true;
    let mut prev_left = true;
    let mut prev_right = true;
    let mut prev_enter = true;
    let mut prev_back = true;

    defmt::info!(
        "hwtest-pivot: starting — clockwise {} deg steps",
        PIVOT_STEP_DEG,
    );

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;
        phase_tick += 1;

        // ── answer radio parameter reads with the tuned pivot values ─────────

        while let Some(pkt) = radio_command_subscriber.try_next_message_pure() {
            if let DataPacket::ParameterCommand(param_cmd) = pkt {
                if param_cmd.command_code == ParameterCommandCode::PCC_READ
                    && param_cmd.parameter_name == PIVOT_READBACK_PARAM
                {
                    let resp = ParameterCommand {
                        command_code: ParameterCommandCode::PCC_ACK,
                        data_format: ParameterDataFormat::VEC4_F32,
                        parameter_name: PIVOT_READBACK_PARAM,
                        data: ParameterCommand_ParameterData {
                            vec3_f32: [
                                orbit_radius,
                                inset_angle,
                                max_angular_acc,
                            ],
                        },
                    };
                    defmt::info!(
                        "hwtest-pivot: param read → orbit_radius {} m, inset_angle {} rad, max_angular_acc {} rad/s",
                        orbit_radius,
                        inset_angle,
                        max_angular_acc,
                    );
                    pivot_telemetry_publisher
                        .publish_immediate(TelemetryPacket::ParameterCommandResponse(resp));
                }
            }
        }

        // ── button edge detection (falling edge = press) ─────────────────────

        let cur_up = btn_up.is_high();
        let cur_down = btn_down.is_high();
        let cur_left = btn_left.is_high();
        let cur_right = btn_right.is_high();
        let cur_enter = btn_enter.is_high();
        let cur_back = btn_back.is_high();

        if prev_down && !cur_down {
            orbit_radius = (orbit_radius + ORBIT_RADIUS_STEP).min(ORBIT_RADIUS_MAX);
            defmt::info!("hwtest-pivot: orbit_radius → {} m", orbit_radius);
        }
        if prev_up && !cur_up {
            orbit_radius = (orbit_radius - ORBIT_RADIUS_STEP).max(ORBIT_RADIUS_MIN);
            defmt::info!("hwtest-pivot: orbit_radius → {} m", orbit_radius);
        }
        if prev_right && !cur_right {
            inset_angle = (inset_angle + INSET_ANGLE_STEP).min(INSET_ANGLE_MAX);
            defmt::info!("hwtest-pivot: inset_angle → {} rad", inset_angle);
        }
        if prev_left && !cur_left {
            inset_angle = (inset_angle - INSET_ANGLE_STEP).max(INSET_ANGLE_MIN);
            defmt::info!("hwtest-pivot: inset_angle → {} rad", inset_angle);
        }
        if prev_enter && !cur_enter {
            max_angular_acc = (max_angular_acc + ACC_STEP).min(ACC_MAX);
            defmt::info!("hwtest-pivot: max_angular_acc → {} rad/s", max_angular_acc);
        }
        if prev_back && !cur_back {
            max_angular_acc = (max_angular_acc - ACC_STEP).max(ACC_MIN);
            defmt::info!("hwtest-pivot: max_angular_acc → {} rad/s", max_angular_acc);
        }

        prev_up = cur_up;
        prev_down = cur_down;
        prev_left = cur_left;
        prev_right = cur_right;
        prev_enter = cur_enter;
        prev_back = cur_back;

        // ── phase advance ────────────────────────────────────────────────────

        // Robot ID 0 → stop all motion (wheels + dribbler off).
        let robot_id = robot_state.get_hw_robot_id();
        let motion_stopped = robot_id == 0;

        // ── robot ID knob sets the 1e-2 digit of the dribbler setpoint ───────
        // (robot ID 1 → 0.01, 2 → 0.02, …).
        let dribbler_setpoint = if motion_stopped {
            0.0
        } else {
            robot_id as f32 * 0.01
        };

        if phase_tick >= phase_duration_ticks(phase_idx) {
            phase_idx = (phase_idx + 1) % 2;
            phase_tick = 0;
            // Entering a new exec leg → advance the target one step clockwise.
            if phase_idx == 0 {
                target_theta = wrap_pi(target_theta + pivot_step_rad());
            }
            defmt::info!(
                "hwtest-pivot: phase {} → target {} deg",
                phase_idx,
                target_theta * 180.0 / core::f32::consts::PI,
            );
        }

        // ── periodic log of the currently-used parameters (1 Hz) ─────────────

        print_tick += 1;
        if print_tick >= 100 {
            print_tick = 0;
            defmt::info!(
                "\n\nhwtest-pivot params:\norbit_radius={} m\ninset_angle={} rad\nmax_angular_acc={} rad/s²\ndribbler_setpoint={}",
                orbit_radius,
                inset_angle,
                max_angular_acc,
                dribbler_setpoint,
            );
        }

        // ── publish command ──────────────────────────────────────────────────

        command_publisher.publish_immediate(DataPacket::BasicControl(BasicControl {
            _bitfield_1: BasicControl::new_bitfield_1(
                0, // request_shutdown
                0, // reboot_robot
                0, // game_state_in_stop
                0, // game_state_in_halt
                0, // emergency_stop
                1, // wheel_vel_control_enabled
                1, // wheel_torque_control_enabled
                0, // vision_update
                0, // reset_controller
                0, // reserved1
            ),
            _bitfield_align_1: Default::default(),

            vision_position_update: [0.0, 0.0, 0.0],

            body_control_mode: if motion_stopped {
                BodyControlMode::BCM_OFF
            } else {
                BodyControlMode::BCM_HEADING_PIVOT
            },
            kick_request: KickRequest::KR_DISABLE,
            play_song: 0,
            dribbler_mode: if motion_stopped {
                DribblerCommand::DC_DISABLE
            } else {
                DribblerCommand::DC_CURRENT
            },

            kick_vel: 0.0,
            dribbler_setpoint: dribbler_setpoint,

            cmd: BodyControlCommand {
                heading_pivot: HeadingPivotCommand {
                    global_theta: target_theta,
                    max_angular_vel: 20.0,
                    max_angular_acc: max_angular_acc,
                    orbit_radius,
                    inset_angle: inset_angle,
                    ..Default::default()
                },
            },
        }));
    }
}
