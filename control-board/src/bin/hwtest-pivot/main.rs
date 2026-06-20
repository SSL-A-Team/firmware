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
//!   Down  — increase orbit radius (+1 cm)
//!   Up    — decrease orbit radius (-1 cm)
//!   Right — increase dribbler speed (+10 rpm)
//!   Left  — decrease dribbler speed (-10 rpm)
//!
//! The ball is placed at the field origin (0, 0).

use ateam_common_packets::{
    bindings::{BasicControl, BodyControlCommand, BodyControlMode, KickRequest, PivotCommand},
    radio::DataPacket,
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
    create_kicker_task, get_system_config,
    pins::{
        AccelDataPubSub, CommandsPubSub, GyroDataPubSub, KickerTelemetryPubSub, LedCommandPubSub,
        PowerTelemetryPubSub, TelemetryPubSub,
    },
    robot_state::SharedRobotState,
};
use ateam_controls::pivot_trajectory::PivotParams;
use libm::roundf;

use embassy_time::Timer;
use panic_probe as _;
use static_cell::ConstStaticCell;

// ============================================================================
// Static resources
// ============================================================================

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
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

/// Pivot interval range mapped from the robot ID dial (0–15).
/// Dial = 0  → MIN_PIVOT_DEG, Dial = 15 → MAX_PIVOT_DEG, linearly interpolated.
const MIN_PIVOT_DEG: f32 = 5.0;
const MAX_PIVOT_DEG: f32 = 180.0;
const ROTARY_MAX: f32 = 15.0;

fn pivot_angle_from_robot_id(robot_id: u8) -> f32 {
    let t = (robot_id as f32).clamp(0.0, ROTARY_MAX) / ROTARY_MAX;
    let deg = MIN_PIVOT_DEG + t * (MAX_PIVOT_DEG - MIN_PIVOT_DEG);
    deg * core::f32::consts::PI / 180.0
}

/// Main loop interval in milliseconds (100 Hz command rate).
const LOOP_INTERVAL_MS: u64 = 10;

/// Ticks spent executing each pivot leg (3 s at 100 Hz).
const EXEC_TICKS: u32 = 300;

/// Ticks spent holding position after each pivot completes (1 s at 100 Hz).
const HOLD_TICKS: u32 = 100;

/// Ball centre in field coordinates (meters).
const BALL_X: f32 = 0.0;
const BALL_Y: f32 = 0.0;

/// Orbit radius adjustment per button press (meters).
const ORBIT_RADIUS_STEP: f32 = 0.005;
const ORBIT_RADIUS_MIN: f32 = 0.001;
const ORBIT_RADIUS_MAX: f32 = 0.5;

/// Heading lag adjustment per button press (radians).
const HEADING_LAG_STEP: f32 = 0.05;
const HEADING_LAG_MIN: f32 = 0.0;
const HEADING_LAG_MAX: f32 = core::f32::consts::PI;

/// Angular acceleration adjustment per button press (rad/s²).
const ACCEL_STEP: f32 = 0.5;
const ACCEL_MIN: f32 = 0.5;
const ACCEL_MAX: f32 = 20.0 * core::f32::consts::PI;

/// Default max angular velocity — set high so the trajectory stays in the
/// triangular (acceleration-limited) regime regardless of accel setting.
const DEFAULT_MAX_ANGULAR_VEL: f32 = 4.0 * core::f32::consts::PI; // rad/s

/// Sequence mode — change this constant to switch between half and full circle.
const SEQUENCE_MODE: SequenceMode = SequenceMode::FullCircle;

// ============================================================================
// Sequence mode
// ============================================================================

#[derive(Clone, Copy, PartialEq)]
enum SequenceMode {
    /// Alternates +interval → 0 → +interval repeatedly.
    HalfCircle,
    /// Sweeps CCW in `interval` steps until a full circle, then repeats.
    FullCircle,
}

impl SequenceMode {
    fn label(self) -> &'static str {
        match self {
            SequenceMode::HalfCircle => "half-circle",
            SequenceMode::FullCircle => "full-circle",
        }
    }
}

// ============================================================================
// Phase state machine
// ============================================================================

/// Number of pivot legs to complete one full circle at the given step angle.
fn num_full_circle_legs(pivot_angle: f32) -> u32 {
    use core::f32::consts::PI;
    (roundf(2.0 * PI / pivot_angle) as u32).max(1)
}

/// Number of phase steps (pivot + hold pairs × 2) for the current mode and dial angle.
fn phase_num_steps(mode: SequenceMode, pivot_angle: f32) -> u32 {
    let legs = match mode {
        SequenceMode::HalfCircle => 2,
        SequenceMode::FullCircle => num_full_circle_legs(pivot_angle),
    };
    legs * 2
}

fn phase_duration_ticks(phase_idx: u32) -> u32 {
    if phase_idx % 2 == 0 {
        EXEC_TICKS
    } else {
        HOLD_TICKS
    }
}

/// Target heading for the given phase, in radians.
fn phase_target(phase_idx: u32, mode: SequenceMode, pivot_angle: f32) -> f32 {
    use core::f32::consts::PI;
    let num_legs = match mode {
        SequenceMode::HalfCircle => 2,
        SequenceMode::FullCircle => num_full_circle_legs(pivot_angle),
    };
    let target_idx = (phase_idx / 2) % num_legs;
    let raw = pivot_angle * (target_idx as f32 + 1.0);
    // wrap to (-π, π]
    let pi2 = 2.0 * PI;
    let mut a = raw % pi2;
    if a > PI {
        a -= pi2;
    } else if a <= -PI {
        a += pi2;
    }
    a
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
    let btn_enter = Input::new(p.PE11, Pull::Up); // lower acceleration
    let btn_back = Input::new(p.PE10, Pull::Up); // unused (accel set in code)

    // ── inter-task channels ──────────────────────────────────────────────────

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    let command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let imu_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();

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

    let _ = radio_uart_queue_spawner; // radio task not needed; suppress unused warning

    // ── tunable parameters (adjusted via buttons) ────────────────────────────

    let mut orbit_radius: f32 = PivotParams::default().orbit_radius;
    let mut heading_lag: f32 = PivotParams::default().heading_lag;
    let mut max_angular_acc: f32 = PivotParams::default().max_accel_angular;

    defmt::info!(
        "hwtest-pivot: orbit_radius = {} m, heading_lag = {} rad, max_angular_acc = {} rad/s²",
        orbit_radius,
        heading_lag,
        max_angular_acc,
    );

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── pivot sequencer loop ─────────────────────────────────────────────────

    let mut phase_idx: u32 = 0;
    let mut phase_tick: u32 = 0;

    // Previous button states for falling-edge detection (true = not pressed).
    let mut prev_up = true;
    let mut prev_down = true;
    let mut prev_left = true;
    let mut prev_right = true;
    let mut prev_enter = true;
    let mut prev_back = true;

    defmt::info!("hwtest-pivot: starting — mode: {}", SEQUENCE_MODE.label());

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;
        phase_tick += 1;

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
            heading_lag = (heading_lag + HEADING_LAG_STEP).min(HEADING_LAG_MAX);
            defmt::info!("hwtest-pivot: heading_lag → {} rad", heading_lag);
        }
        if prev_left && !cur_left {
            heading_lag = (heading_lag - HEADING_LAG_STEP).max(HEADING_LAG_MIN);
            defmt::info!("hwtest-pivot: heading_lag → {} rad", heading_lag);
        }
        if prev_enter && !cur_enter {
            max_angular_acc = (max_angular_acc + ACCEL_STEP).min(ACCEL_MAX);
            defmt::info!("hwtest-pivot: max_angular_acc → {} rad/s²", max_angular_acc);
        }
        if prev_back && !cur_back {
            max_angular_acc = (max_angular_acc - ACCEL_STEP).max(ACCEL_MIN);
            defmt::info!("hwtest-pivot: max_angular_acc → {} rad/s²", max_angular_acc);
        }

        prev_up = cur_up;
        prev_down = cur_down;
        prev_left = cur_left;
        prev_right = cur_right;
        prev_enter = cur_enter;
        prev_back = cur_back;

        // ── phase advance ────────────────────────────────────────────────────

        let pivot_angle = pivot_angle_from_robot_id(robot_state.get_hw_robot_id());

        if phase_tick >= phase_duration_ticks(phase_idx) {
            phase_idx = (phase_idx + 1) % phase_num_steps(SEQUENCE_MODE, pivot_angle);
            phase_tick = 0;
            defmt::info!(
                "hwtest-pivot: phase {} → pivot_angle {} deg",
                phase_idx,
                pivot_angle * 180.0 / core::f32::consts::PI,
            );
        }

        let target_theta = phase_target(phase_idx, SEQUENCE_MODE, pivot_angle);

        // ── publish command ──────────────────────────────────────────────────

        command_publisher.publish_immediate(DataPacket::BasicControl(BasicControl {
            _bitfield_1: BasicControl::new_bitfield_1(
                0, // request_shutdown
                0, // reboot_robot
                0, // game_state_in_stop
                0, // emergency_stop
                1, // wheel_vel_control_enabled
                1, // wheel_torque_control_enabled
                0, // vision_update
                0, // reset_controller
                0, // reserved1
            ),
            _bitfield_align_1: Default::default(),

            vision_position_update: [0.0, 0.0, 0.0],

            body_control_mode: BodyControlMode::BCM_PIVOT,
            kick_request: KickRequest::KR_DISABLE,
            play_song: 0,
            reserved2: [0; 1],

            kick_vel: 0.0,
            dribbler_speed: 200.0,

            cmd: BodyControlCommand {
                pivot: PivotCommand {
                    global_x_center: BALL_X,
                    global_y_center: BALL_Y,
                    global_theta: target_theta,
                    max_angular_vel: DEFAULT_MAX_ANGULAR_VEL,
                    max_angular_acc: max_angular_acc,
                    orbit_radius,
                    heading_lag: heading_lag,
                },
            },
        }));
    }
}
