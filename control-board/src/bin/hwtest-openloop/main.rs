#![no_std]
#![no_main]

use core::f32::consts::PI;

use ateam_common_packets::{bindings::{BasicControl, KickRequest}, radio::DataPacket};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, pac::Interrupt};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;

use ateam_control_board::{
    create_audio_task, create_control_task, create_dotstar_task, create_imu_task, create_io_task,
    create_kicker_task, create_power_task, create_radio_task, get_system_config,
    pins::{
        AccelDataPubSub, CommandsPubSub, GyroDataPubSub, KickerTelemetryPubSub, LedCommandPubSub,
        PowerTelemetryPubSub, TelemetryPubSub,
    },
    robot_state::SharedRobotState,
};

// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::{Instant, Ticker, Timer};
use libm::{cosf, sinf};
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

// #[panic_handler]
// fn panic(info: &core::panic::PanicInfo) -> ! {
//     defmt::error!("{}", defmt::Display2Format(info));
//     // Delay to give it a change to print
//     cortex_m::asm::delay(10_000_000);
//     cortex_m::peripheral::SCB::sys_reset();
// }

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_DUMMY_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();
static POWER_DATA_CHANNEL: PowerTelemetryPubSub = PubSubChannel::new();
static KICKER_DATA_CHANNEL: KickerTelemetryPubSub = PubSubChannel::new();
static LED_COMMAND_PUBSUB: LedCommandPubSub = PubSubChannel::new();

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

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

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    // init system
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    let robot_state = ROBOT_STATE.take();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CORDIC,
        embassy_stm32::interrupt::Priority::P6,
    );
    let radio_uart_queue_spawner = RADIO_UART_QUEUE_EXECUTOR.start(Interrupt::CORDIC);

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P7,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    // commands channel
    let radio_dummy_command_publisher = RADIO_DUMMY_C2_CHANNEL.publisher().unwrap();
    let mut radio_dummy_command_subscriber = RADIO_DUMMY_C2_CHANNEL.subscriber().unwrap();
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();
    let radio_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    // imu channel
    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let imu_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    let control_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();
    let control_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();

    // power channel
    let power_board_telemetry_publisher = POWER_DATA_CHANNEL.publisher().unwrap();
    let control_task_power_telemetry_subscriber = POWER_DATA_CHANNEL.subscriber().unwrap();

    // kicker channel
    let kicker_board_telemetry_publisher = KICKER_DATA_CHANNEL.publisher().unwrap();
    let control_task_kicker_telemetry_subscriber = KICKER_DATA_CHANNEL.subscriber().unwrap();

    // power board
    let power_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner, robot_state, p);

    create_dotstar_task!(main_spawner, led_command_subscriber, p);

    create_audio_task!(main_spawner, robot_state, p);

    create_radio_task!(
        main_spawner,
        radio_uart_queue_spawner,
        radio_uart_queue_spawner,
        robot_state,
        radio_dummy_command_publisher,
        radio_telemetry_subscriber,
        radio_led_cmd_publisher,
        wifi_credentials,
        p
    );

    create_power_task!(
        main_spawner,
        uart_queue_spawner,
        robot_state,
        power_board_telemetry_publisher,
        power_led_cmd_publisher,
        p
    );

    create_imu_task!(
        main_spawner,
        robot_state,
        imu_gyro_data_publisher,
        imu_accel_data_publisher,
        imu_led_cmd_publisher,
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

    let loop_period = embassy_time::Duration::from_millis(1);
    let mut loop_rate_ticker = Ticker::every(loop_period);

    let w = 2.0 * PI / 3.0; // 3 second period
    // let a_angular = 5.0;  // rad/s amplitude
    let a_angular = 5.0;  // rad/s^2 amplitude
    let a_linear = 0.5;  // m/s amplitude

    let request_shutdown = 0;
    let reboot_robot = 0;
    let game_state_in_stop = 0;
    let emergency_stop = 0;
    let body_pose_control_enabled = 0;
    let body_twist_control_enabled = 0;
    let body_accel_control_enabled = 1;
    let wheel_vel_control_enabled = 0;
    let wheel_torque_control_enabled = 1;
    let vision_update = 0;
    let dribbler_multiplier = 0;
    let reserved = 0;
    let play_song = 0;

    let mut control = BasicControl {
        _bitfield_1: BasicControl::new_bitfield_1(request_shutdown, reboot_robot, game_state_in_stop, emergency_stop, body_pose_control_enabled, body_twist_control_enabled, body_accel_control_enabled, wheel_vel_control_enabled, wheel_torque_control_enabled, vision_update, dribbler_multiplier, reserved, play_song),
        _bitfield_align_1: Default::default(),
        pose_x_linear_vision: 0.0,
        pose_y_linear_vision: 0.0,
        pose_z_angular_vision: 0.0,
        x_linear_cmd: 0.0,
        y_linear_cmd: 0.0,
        z_angular_cmd: 0.0,
        kick_vel: 0.0,
        dribbler_speed: 0.0,
        kick_request: KickRequest::KR_DISABLE,
    };

    Timer::after_secs(3).await;

    let mut last_loop_start = Instant::now();
    let mut t = 0.0;
    loop {
        t += Instant::now().checked_duration_since(last_loop_start).unwrap().as_micros() as f32 / 1_000_000.0;
        last_loop_start = Instant::now();


        // if t < 3.0 {
        //     control.z_angular_cmd = 2.0;
        // } else {
        //     control.z_angular_cmd = 0.0;
        // }

        control.z_angular_cmd = a_angular * sinf(w * t);
        // control.x_linear_cmd = a_linear * cosf(w * t);
        // control.y_linear_cmd = a_linear * sinf(w * t);

        // Extract any vision updates
        while let Some(latest_packet) = radio_dummy_command_subscriber.try_next_message_pure() {
            match latest_packet {
                ateam_common_packets::radio::DataPacket::BasicControl(latest_control) => {
                    if latest_control.vision_update() != 0 {
                        control.set_vision_update(1);
                        control.pose_x_linear_vision = latest_control.pose_x_linear_vision;
                        control.pose_y_linear_vision = latest_control.pose_y_linear_vision;
                        control.pose_z_angular_vision = latest_control.pose_z_angular_vision;
                    }
                }
                ateam_common_packets::radio::DataPacket::ParameterCommand(_) => {},
            }
        }
        radio_command_publisher.publish_immediate(DataPacket::BasicControl(control));

        control.set_vision_update(0);

        loop_rate_ticker.next().await;
    }
}
