#![no_std]
#![no_main]

//! Hardware test: ball-capture drive (GlobalPosition) maneuver
//!
//! Mocks radio commands and repeatedly drives the robot forward in +x while
//! running the dribbler, then holds position with the dribbler dropped to a low
//! idle setpoint (0.01) to "capture" the ball, pauses, and drives forward again.
//! Position mode requires vision, so vision is mocked by echoing the KF state
//! estimate (from BasicTelemetry) back as the vision pose each tick (see
//! hwtest-line). On-field use needs real vision.
//!
//! Phase sequence (loops forever, advancing the target one drive length each cycle):
//!   1. drive   forward DRIVE_DISTANCE_M — dribbler runs at the ID-dial setpoint
//!   2. capture hold at target           — dribbler dropped to 0.01, CAPTURE_HOLD_MS
//!   3. wait    hold at target           — dribbler dropped to 0.01, POST_CAPTURE_WAIT_MS
//!   → advance target by DRIVE_DISTANCE_M and repeat from (1)
//!
//! The drive→capture transition happens once the robot has actually reached the
//! target (estimated x within POS_TOLERANCE_M) and settled (estimated speed
//! below VEL_TOLERANCE_MPS), rather than after a fixed time.
//!
//! Button controls (take effect on the next command tick, mirroring hwtest-pivot):
//!   Down  — increase max linear velocity (+0.1 m/s)
//!   Up    — decrease max linear velocity (-0.1 m/s)
//!
//! The robot ID knob sets the dribbler current setpoint during the drive phase
//! (ID × 0.01); ID 0 stops all motion. During the capture and wait phases the
//! dribbler setpoint drops to 0.01 regardless of the ID knob (unless ID 0).

use ateam_common_packets::{
    bindings::{
        BasicControl, BodyControlCommand, BodyControlMode, DribblerCommand, GlobalPositionCommand,
        KickRequest,
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
    create_kicker_task, get_system_config,
    pins::{
        AccelDataPubSub, CommandsPubSub, GyroDataPubSub, KickerTelemetryPubSub, LedCommandPubSub,
        PowerTelemetryPubSub, TelemetryPubSub,
    },
    robot_state::SharedRobotState,
};
use ateam_controls::bangbang_trajectory::TrajectoryParams;

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
// Capture sequencer constants
// ============================================================================

/// Main loop interval in milliseconds (100 Hz command rate).
const LOOP_INTERVAL_MS: u64 = 10;

/// Forward drive distance in +x (meters).
const DRIVE_DISTANCE_M: f32 = 0.2;

/// Position tolerance for considering the target reached (meters).
const POS_TOLERANCE_M: f32 = 0.01;

/// Speed tolerance for considering the robot settled at the target (m/s).
const VEL_TOLERANCE_MPS: f32 = 0.05;

/// Dribbler setpoint held after the drive completes ("captured" idle speed).
const CAPTURE_DRIBBLER_SETPOINT: f32 = 0.01;

/// Duration to hold the captured position at the idle dribbler setpoint (ms).
const CAPTURE_HOLD_MS: u32 = 2000;

/// Additional pause after the capture hold before driving forward again (ms).
const POST_CAPTURE_WAIT_MS: u32 = 5000;

/// Capture/wait phase durations expressed in loop ticks.
const CAPTURE_HOLD_TICKS: u32 = CAPTURE_HOLD_MS / LOOP_INTERVAL_MS as u32;
const POST_CAPTURE_WAIT_TICKS: u32 = POST_CAPTURE_WAIT_MS / LOOP_INTERVAL_MS as u32;

/// Max linear velocity adjustment per button press (m/s).
/// Down increases, Up decreases (mirrors hwtest-pivot's orbit-radius controls).
const MAX_LINEAR_VEL_STEP: f32 = 0.1;
const MAX_LINEAR_VEL_MIN: f32 = 0.1;
const MAX_LINEAR_VEL_MAX: f32 = 3.0;

// ============================================================================
// Phase state machine
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Driving forward to the target while running the dribbler.
    Drive,
    /// Reached the target; hold position with the dribbler at idle speed.
    Capture,
    /// Extra pause after the capture hold before driving forward again.
    Wait,
}

// ============================================================================
// Entry point
// ============================================================================

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("hwtest-capture: initialising");

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

    let btn_up = Input::new(p.PE14, Pull::Up); // decrease max linear velocity
    let btn_down = Input::new(p.PE15, Pull::Up); // increase max linear velocity

    // ── inter-task channels ──────────────────────────────────────────────────

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    let command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let imu_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    // Subscribe to telemetry so vision can be mocked from the KF state estimate.
    let mut telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

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

    // Basic/extended telemetry only publishes when the radio bridge is up. There
    // is no radio task here, so force it on so the KF state estimate is published
    // and can be echoed back as the mocked vision measurement.
    robot_state.set_radio_bridge_ok(true);

    // ── tunable parameters (adjusted via buttons) ────────────────────────────

    let mut max_linear_vel: f32 = 0.1;

    defmt::info!(
        "hwtest-capture: drive_distance = {} m, max_linear_vel = {} m/s",
        DRIVE_DISTANCE_M,
        max_linear_vel,
    );

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── capture sequencer loop ───────────────────────────────────────────────

    let mut phase = Phase::Drive;

    // Tick counter for the timed capture/wait phases (reset on each entry).
    let mut phase_tick: u32 = 0;

    // Running forward target in +x. Advanced one drive length each cycle so the
    // robot keeps driving forward, capturing, and repeating.
    let mut target_x: f32 = DRIVE_DISTANCE_M;

    // Mocked vision pose, fed back from the KF state estimate. Starts at the
    // origin until the first BasicTelemetry arrives.
    let mut mock_vision_pose: [f32; 3] = [0.0, 0.0, 0.0];

    // Latest estimated body speed² (m²/s²), from the KF velocity estimate. Used
    // to decide when the forward drive has settled at the target (avoids sqrt in
    // no_std by comparing against VEL_TOLERANCE_MPS²).
    let mut est_speed_sq: f32 = f32::INFINITY;

    // Previous button states for falling-edge detection (true = not pressed).
    let mut prev_up = true;
    let mut prev_down = true;

    // Throttle counter for periodic parameter logging (1 Hz at 100 Hz loop).
    let mut print_tick: u32 = 0;

    defmt::info!("hwtest-capture: starting — driving forward {} m", DRIVE_DISTANCE_M);

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;

        // ── mock vision: echo the latest KF state estimate from telemetry ────
        // Drain queued telemetry, keeping the most recent BasicTelemetry pose
        // estimate (mm/mrad → m/rad) to send back as the mocked vision update,
        // and the linear speed estimate (mm/s → m/s) to detect drive completion.
        while let Some(pkt) = telemetry_subscriber.try_next_message_pure() {
            if let TelemetryPacket::Basic(b) = pkt {
                mock_vision_pose = [
                    b.kf_body_pos_estimate[0] as f32 / 1000.0,
                    b.kf_body_pos_estimate[1] as f32 / 1000.0,
                    b.kf_body_pos_estimate[2] as f32 / 1000.0,
                ];
                let vx = b.kf_body_vel_estimate[0] as f32 / 1000.0;
                let vy = b.kf_body_vel_estimate[1] as f32 / 1000.0;
                est_speed_sq = vx * vx + vy * vy;
            }
        }

        // ── button edge detection (falling edge = press) ─────────────────────

        let cur_up = btn_up.is_high();
        let cur_down = btn_down.is_high();

        if prev_down && !cur_down {
            max_linear_vel = (max_linear_vel + MAX_LINEAR_VEL_STEP).min(MAX_LINEAR_VEL_MAX);
            defmt::info!("hwtest-capture: max_linear_vel → {} m/s", max_linear_vel);
        }
        if prev_up && !cur_up {
            max_linear_vel = (max_linear_vel - MAX_LINEAR_VEL_STEP).max(MAX_LINEAR_VEL_MIN);
            defmt::info!("hwtest-capture: max_linear_vel → {} m/s", max_linear_vel);
        }

        prev_up = cur_up;
        prev_down = cur_down;

        // ── phase advance ────────────────────────────────────────────────────
        // Drive → Capture → Wait → (advance target) → Drive, forever.
        //   Drive:   leave once the robot has actually reached the target
        //            (estimated x within tolerance) and settled (low speed).
        //   Capture: hold at the target with the idle dribbler for CAPTURE_HOLD.
        //   Wait:    additional pause, then advance the target and drive again.
        match phase {
            Phase::Drive => {
                let x = mock_vision_pose[0];
                let reached =
                    x >= target_x - POS_TOLERANCE_M && x <= target_x + POS_TOLERANCE_M;
                let settled = est_speed_sq <= VEL_TOLERANCE_MPS * VEL_TOLERANCE_MPS;
                if reached && settled {
                    phase = Phase::Capture;
                    phase_tick = 0;
                    defmt::info!(
                        "hwtest-capture: drive complete (x {} m, speed² {} m²/s²) → capture hold, dribbler → {}",
                        x,
                        est_speed_sq,
                        CAPTURE_DRIBBLER_SETPOINT,
                    );
                }
            }
            Phase::Capture => {
                phase_tick += 1;
                if phase_tick >= CAPTURE_HOLD_TICKS {
                    phase = Phase::Wait;
                    phase_tick = 0;
                    defmt::info!("hwtest-capture: capture hold done → waiting {} ms", POST_CAPTURE_WAIT_MS);
                }
            }
            Phase::Wait => {
                phase_tick += 1;
                if phase_tick >= POST_CAPTURE_WAIT_TICKS {
                    phase = Phase::Drive;
                    phase_tick = 0;
                    target_x += DRIVE_DISTANCE_M;
                    defmt::info!("hwtest-capture: wait done → driving forward to x {} m", target_x);
                }
            }
        }

        // ── robot ID knob sets the dribbler setpoint during the drive ────────
        // (robot ID 1 → 0.01, 2 → 0.02, …). ID 0 stops all motion. During the
        // capture and wait phases the dribbler drops to the idle capture setpoint.
        let robot_id = robot_state.get_hw_robot_id();
        let motion_stopped = robot_id == 0;

        let dribbler_setpoint = if motion_stopped {
            0.0
        } else {
            match phase {
                Phase::Drive => robot_id as f32 * 0.01,
                Phase::Capture | Phase::Wait => CAPTURE_DRIBBLER_SETPOINT,
            }
        };

        // ── periodic log of the currently-used parameters (1 Hz) ─────────────

        print_tick += 1;
        if print_tick >= 100 {
            print_tick = 0;
            defmt::info!(
                "\n\nhwtest-capture params:\nphase={}\nmax_linear_vel={} m/s\ndribbler_setpoint={}",
                match phase {
                    Phase::Drive => "drive",
                    Phase::Capture => "capture",
                    Phase::Wait => "wait",
                },
                max_linear_vel,
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
                1, // vision_update (mocked from KF estimate — position mode requires vision)
                0, // reset_controller
                0, // reserved1
            ),
            _bitfield_align_1: Default::default(),

            vision_position_update: mock_vision_pose,

            body_control_mode: if motion_stopped {
                BodyControlMode::BCM_OFF
            } else {
                BodyControlMode::BCM_GLOBAL_POSITION
            },
            kick_request: KickRequest::KR_DISABLE,
            play_song: 0,
            dribbler_mode: if motion_stopped {
                DribblerCommand::DC_DISABLE
            } else {
                DribblerCommand::DC_CURRENT
            },

            kick_vel: 0.0,
            dribbler_setpoint,

            cmd: BodyControlCommand {
                global_pos: GlobalPositionCommand {
                    global_x: target_x,
                    global_y: 0.0,
                    global_theta: 0.0,
                    max_linear_vel,
                    ..Default::default()
                },
            },
        }));
    }
}
