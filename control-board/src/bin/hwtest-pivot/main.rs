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
use embassy_stm32::{gpio::{Input, Pull}, interrupt, pac::Interrupt};
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

/// Dribbler speed adjustment per button press (rpm).
const DRIBBLER_SPEED_STEP: f32 = 10.0;
const DRIBBLER_SPEED_MIN: f32 = 0.0;
const DRIBBLER_SPEED_MAX: f32 = 500.0;

// ============================================================================
// Phase state machine
// ============================================================================

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    PivotToPI,
    HoldAtPI,
    PivotToZero,
    HoldAtZero,
}

impl Phase {
    /// Commanded heading target sent in each `PivotCommand`.
    fn target_theta(self) -> f32 {
        match self {
            Phase::PivotToPI | Phase::HoldAtPI => core::f32::consts::PI,
            Phase::PivotToZero | Phase::HoldAtZero => 0.0,
        }
    }

    fn duration_ticks(self) -> u32 {
        match self {
            Phase::PivotToPI | Phase::PivotToZero => EXEC_TICKS,
            Phase::HoldAtPI | Phase::HoldAtZero => HOLD_TICKS,
        }
    }

    fn next(self) -> Self {
        match self {
            Phase::PivotToPI => Phase::HoldAtPI,
            Phase::HoldAtPI => Phase::PivotToZero,
            Phase::PivotToZero => Phase::HoldAtZero,
            Phase::HoldAtZero => Phase::PivotToPI,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Phase::PivotToPI => "pivot  0° → 180°",
            Phase::HoldAtPI => "hold  at 180°",
            Phase::PivotToZero => "pivot 180° →   0°",
            Phase::HoldAtZero => "hold  at   0°",
        }
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

    let btn_up    = Input::new(p.PE14, Pull::Up);
    let btn_down  = Input::new(p.PE15, Pull::Up);
    let btn_left  = Input::new(p.PE12, Pull::Up);
    let btn_right = Input::new(p.PE13, Pull::Up);

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
    let mut dribbler_speed: f32 = 300.0;

    defmt::info!(
        "hwtest-pivot: orbit_radius = {} m, dribbler_speed = {} rpm",
        orbit_radius, dribbler_speed,
    );

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── pivot sequencer loop ─────────────────────────────────────────────────

    let mut phase = Phase::PivotToPI;
    let mut phase_tick: u32 = 0;

    // Previous button states for falling-edge detection (true = not pressed).
    let mut prev_up    = true;
    let mut prev_down  = true;
    let mut prev_left  = true;
    let mut prev_right = true;

    defmt::info!("hwtest-pivot: starting — {}", phase.label());

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;
        phase_tick += 1;

        // ── button edge detection (falling edge = press) ─────────────────────

        let cur_up    = btn_up.is_high();
        let cur_down  = btn_down.is_high();
        let cur_left  = btn_left.is_high();
        let cur_right = btn_right.is_high();

        if prev_down && !cur_down {
            orbit_radius = (orbit_radius + ORBIT_RADIUS_STEP).min(ORBIT_RADIUS_MAX);
            defmt::info!("hwtest-pivot: orbit_radius → {} m", orbit_radius);
        }
        if prev_up && !cur_up {
            orbit_radius = (orbit_radius - ORBIT_RADIUS_STEP).max(ORBIT_RADIUS_MIN);
            defmt::info!("hwtest-pivot: orbit_radius → {} m", orbit_radius);
        }
        if prev_right && !cur_right {
            dribbler_speed = (dribbler_speed + DRIBBLER_SPEED_STEP).min(DRIBBLER_SPEED_MAX);
            defmt::info!("hwtest-pivot: dribbler_speed → {} rpm", dribbler_speed);
        }
        if prev_left && !cur_left {
            dribbler_speed = (dribbler_speed - DRIBBLER_SPEED_STEP).max(DRIBBLER_SPEED_MIN);
            defmt::info!("hwtest-pivot: dribbler_speed → {} rpm", dribbler_speed);
        }

        prev_up    = cur_up;
        prev_down  = cur_down;
        prev_left  = cur_left;
        prev_right = cur_right;

        // ── phase advance ────────────────────────────────────────────────────

        if phase_tick >= phase.duration_ticks() {
            phase = phase.next();
            phase_tick = 0;
            defmt::info!("hwtest-pivot: phase → {}", phase.label());
        }

        let target_theta = phase.target_theta();

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
            dribbler_speed,

            cmd: BodyControlCommand {
                pivot: PivotCommand {
                    global_x_center: BALL_X,
                    global_y_center: BALL_Y,
                    global_theta: target_theta,
                    max_angular_vel: 0.0,  // zero → PivotParams::default()
                    max_angular_acc: 0.0,
                    orbit_radius,
                    heading_lag: 1.0,
                },
            },
        }));
    }
}

