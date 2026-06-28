#![no_std]
#![no_main]

//! Hardware test: line-following (HeadingLine) maneuver
//!
//! Mocks radio commands and repeatedly drives the robot back and forth along a
//! line through the origin pointing in +x, holding a fixed heading of 0 rad.
//! Line following requires vision (like position mode), so a fixed vision pose
//! at the origin is mocked each tick to keep the controller active for bring-up;
//! on-field use needs real vision.
//!
//! Phase sequence (loops forever):
//!   1. follow line at +line_vel   (3 s)
//!   2. hold (line_vel = 0)        (1 s)
//!   3. follow line at -line_vel   (3 s)
//!   4. hold (line_vel = 0)        (1 s)
//!
//! Button controls (take effect on the next command tick):
//!   Down  — increase line velocity      (+0.1 m/s)
//!   Up    — decrease line velocity       (-0.1 m/s)
//!   Enter — increase colinear accel      (+0.5 m/s²)
//!   Back  — decrease colinear accel      (-0.5 m/s²)

use ateam_common_packets::{
    bindings::{
        BasicControl, BodyControlCommand, BodyControlMode, DribblerCommand, HeadingLineCommand,
        KickRequest,
    },
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
use ateam_controls::linear_trajectory::LinearParams;

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
// Line sequencer constants
// ============================================================================

/// Main loop interval in milliseconds (100 Hz command rate).
const LOOP_INTERVAL_MS: u64 = 10;

/// Ticks spent following the line each direction (3 s at 100 Hz).
const EXEC_TICKS: u32 = 300;

/// Ticks spent holding (line velocity 0) between legs (1 s at 100 Hz).
const HOLD_TICKS: u32 = 100;

/// Line velocity adjustment per button press (m/s).
const LINE_VEL_STEP: f32 = 0.1;
const LINE_VEL_MIN: f32 = 0.1;
const LINE_VEL_MAX: f32 = 3.0;

/// Colinear acceleration adjustment per button press (m/s²).
const ACCEL_STEP: f32 = 0.5;
const ACCEL_MIN: f32 = 0.5;
const ACCEL_MAX: f32 = 20.0;

/// Fixed target heading held throughout the test (rad).
const TARGET_THETA: f32 = 0.0;

/// Four phases: +follow, hold, -follow, hold.
const NUM_PHASES: u32 = 4;

fn phase_duration_ticks(phase_idx: u32) -> u32 {
    if phase_idx % 2 == 0 {
        EXEC_TICKS
    } else {
        HOLD_TICKS
    }
}

/// Line velocity command for the given phase. Even exec phases drive +/- the
/// magnitude; odd hold phases command zero.
fn phase_line_velocity(phase_idx: u32, line_vel_mag: f32) -> f32 {
    match phase_idx {
        0 => line_vel_mag,
        2 => -line_vel_mag,
        _ => 0.0,
    }
}

// ============================================================================
// Entry point
// ============================================================================

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("hwtest-line: initialising");

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
    let btn_enter = Input::new(p.PE11, Pull::Up);
    let btn_back = Input::new(p.PE10, Pull::Up);

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

    let mut line_vel_mag: f32 = 1.0;
    let mut max_accel_colinear: f32 = LinearParams::default().max_accel_colinear;

    defmt::info!(
        "hwtest-line: line_vel = {} m/s, max_accel_colinear = {} m/s²",
        line_vel_mag,
        max_accel_colinear,
    );

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── line sequencer loop ──────────────────────────────────────────────────

    let mut phase_idx: u32 = 0;
    let mut phase_tick: u32 = 0;

    // Previous button states for falling-edge detection (true = not pressed).
    let mut prev_up = true;
    let mut prev_down = true;
    let mut prev_enter = true;
    let mut prev_back = true;

    defmt::info!("hwtest-line: starting");

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;
        phase_tick += 1;

        // ── button edge detection (falling edge = press) ─────────────────────

        let cur_up = btn_up.is_high();
        let cur_down = btn_down.is_high();
        let cur_enter = btn_enter.is_high();
        let cur_back = btn_back.is_high();

        if prev_down && !cur_down {
            line_vel_mag = (line_vel_mag + LINE_VEL_STEP).min(LINE_VEL_MAX);
            defmt::info!("hwtest-line: line_vel → {} m/s", line_vel_mag);
        }
        if prev_up && !cur_up {
            line_vel_mag = (line_vel_mag - LINE_VEL_STEP).max(LINE_VEL_MIN);
            defmt::info!("hwtest-line: line_vel → {} m/s", line_vel_mag);
        }
        if prev_enter && !cur_enter {
            max_accel_colinear = (max_accel_colinear + ACCEL_STEP).min(ACCEL_MAX);
            defmt::info!(
                "hwtest-line: max_accel_colinear → {} m/s²",
                max_accel_colinear
            );
        }
        if prev_back && !cur_back {
            max_accel_colinear = (max_accel_colinear - ACCEL_STEP).max(ACCEL_MIN);
            defmt::info!(
                "hwtest-line: max_accel_colinear → {} m/s²",
                max_accel_colinear
            );
        }

        prev_up = cur_up;
        prev_down = cur_down;
        prev_enter = cur_enter;
        prev_back = cur_back;

        // ── phase advance ────────────────────────────────────────────────────

        if phase_tick >= phase_duration_ticks(phase_idx) {
            phase_idx = (phase_idx + 1) % NUM_PHASES;
            phase_tick = 0;
            defmt::info!("hwtest-line: phase {}", phase_idx);
        }

        let line_velocity = phase_line_velocity(phase_idx, line_vel_mag);

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
                1, // vision_update (mocked at origin — line following requires vision)
                0, // reset_controller
                0, // reserved1
            ),
            _bitfield_align_1: Default::default(),

            vision_position_update: [0.0, 0.0, 0.0],

            body_control_mode: BodyControlMode::BCM_HEADING_LINE,
            kick_request: KickRequest::KR_DISABLE,
            play_song: 0,
            dribbler_mode: DribblerCommand::DC_CURRENT,

            kick_vel: 0.0,
            dribbler_setpoint: 0.0,

            cmd: BodyControlCommand {
                heading_line: HeadingLineCommand {
                    start_x: 0.0,
                    start_y: 0.0,
                    dir_x: 1.0,
                    dir_y: 0.0,
                    line_velocity,
                    global_theta: TARGET_THETA,
                    max_accel_colinear,
                    ..Default::default()
                },
            },
        }));
    }
}
