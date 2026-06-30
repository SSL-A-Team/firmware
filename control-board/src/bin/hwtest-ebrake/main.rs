#![no_std]
#![no_main]

//! Hardware test: emergency-brake (ActiveBrakeController)
//!
//! Mocks radio commands to exercise the active braking path. The robot drives
//! straight forward under local velocity control, then an emergency stop is
//! asserted so the `ActiveBrakeController` takes over and drives every wheel
//! back to zero velocity (KF-independent, P controller on raw encoder readings).
//!
//! Local velocity control needs no vision, so nothing is mocked here.
//!
//! Phase sequence (loops forever):
//!   1. DRIVE  — local velocity forward at FORWARD_VEL m/s   (2 s)
//!   2. EBRAKE — assert emergency_stop, ActiveBrakeController halts the wheels (3 s)
//!
//! The EBRAKE phase keeps publishing commands every tick (so the control task
//! never falls into the packet-timeout hard stop) and keeps a non-OFF body
//! control mode with wheel motion enabled. Those are exactly the conditions
//! under which `control_task` routes the wheel commands through the
//! `ActiveBrakeController` instead of the normal body controller.

use ateam_common_packets::{
    bindings::{
        BasicControl, BodyControlCommand, BodyControlMode, DribblerCommand, KickRequest,
        LocalVelocityCommand,
    },
    radio::DataPacket,
};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, pac::Interrupt};
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
// Ebrake sequencer constants
// ============================================================================

/// Main loop interval in milliseconds (100 Hz command rate).
const LOOP_INTERVAL_MS: u64 = 10;

/// Forward local velocity commanded during the DRIVE phase (m/s).
const FORWARD_VEL: f32 = 1.0;

/// Ticks spent driving forward before braking (3 s at 100 Hz).
const DRIVE_TICKS: u32 = 200;

/// Ticks spent under emergency brake (3 s at 100 Hz).
const EBRAKE_TICKS: u32 = 300;

/// Two phases: drive forward, then emergency brake.
const NUM_PHASES: u32 = 2;

const PHASE_DRIVE: u32 = 0;
const PHASE_EBRAKE: u32 = 1;

fn phase_duration_ticks(phase_idx: u32) -> u32 {
    match phase_idx {
        PHASE_DRIVE => DRIVE_TICKS,
        _ => EBRAKE_TICKS,
    }
}

// ============================================================================
// Entry point
// ============================================================================

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("hwtest-ebrake: initialising");

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

    let _ = radio_uart_queue_spawner; // radio task not needed

    // ── wait for the control task to finish motor firmware flashing ──────────

    Timer::after_millis(5000).await;

    // ── ebrake sequencer loop ────────────────────────────────────────────────

    let mut phase_idx: u32 = PHASE_DRIVE;
    let mut phase_tick: u32 = 0;

    defmt::info!(
        "hwtest-ebrake: starting (drive forward at {} m/s, then emergency brake)",
        FORWARD_VEL
    );

    loop {
        Timer::after_millis(LOOP_INTERVAL_MS).await;
        phase_tick += 1;

        // ── phase advance ────────────────────────────────────────────────────

        if phase_tick >= phase_duration_ticks(phase_idx) {
            phase_idx = (phase_idx + 1) % NUM_PHASES;
            phase_tick = 0;
            match phase_idx {
                PHASE_DRIVE => defmt::info!("hwtest-ebrake: phase DRIVE (forward velocity)"),
                _ => defmt::info!("hwtest-ebrake: phase EBRAKE (ActiveBrakeController)"),
            }
        }

        let ebrake = phase_idx == PHASE_EBRAKE;

        // During DRIVE, command a forward local velocity. During EBRAKE the
        // commanded velocity is irrelevant — the control task overrides it with
        // the ActiveBrakeController output — so command zero for clarity.
        let local_xd = if ebrake { 0.0 } else { FORWARD_VEL };

        // ── publish command ──────────────────────────────────────────────────

        command_publisher.publish_immediate(DataPacket::BasicControl(BasicControl {
            _bitfield_1: BasicControl::new_bitfield_1(
                0,             // request_shutdown
                0,             // reboot_robot
                0,             // game_state_in_stop
                0,             // game_state_in_halt
                ebrake as u32, // emergency_stop — engages ActiveBrakeController
                1,             // wheel_vel_control_enabled
                1,             // wheel_torque_control_enabled
                0,             // vision_update (local velocity needs no vision)
                0,             // reset_controller
                0,             // reserved1
            ),
            _bitfield_align_1: Default::default(),

            vision_position_update: [0.0, 0.0, 0.0],

            // Keep a non-OFF mode with motion enabled so the control task routes
            // through the active brake rather than the packet-timeout hard stop.
            body_control_mode: BodyControlMode::BCM_LOCAL_VELOCITY,
            kick_request: KickRequest::KR_DISABLE,
            play_song: 0,
            dribbler_mode: DribblerCommand::DC_CURRENT,

            kick_vel: 0.0,
            dribbler_setpoint: 0.0,

            cmd: BodyControlCommand {
                local_vel: LocalVelocityCommand {
                    local_xd,
                    local_yd: 0.0,
                    local_omega: 0.0,
                    max_linear_acc: 0.0,  // use default limits
                    max_angular_acc: 0.0, // use default limits
                },
            },
        }));
    }
}
