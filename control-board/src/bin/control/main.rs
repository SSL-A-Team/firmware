#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _; 

use ateam_control_board::{
    create_control_task, create_dotstar_task, create_imu_task, create_io_task, create_kicker_task, create_power_task, create_radio_task, get_system_config, pins::{AccelDataPubSub, BatteryVoltPubSub, CommandsPubSub, GyroDataPubSub, LedCommandPubSub, TelemetryPubSub}, robot_state::SharedRobotState};

// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
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

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();
static BATTERY_VOLT_CHANNEL: BatteryVoltPubSub = PubSubChannel::new();
static LED_COMMAND_PUBSUB: LedCommandPubSub = PubSubChannel::new();

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

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

    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CORDIC, embassy_stm32::interrupt::Priority::P6);
    let radio_uart_queue_spawner = RADIO_UART_QUEUE_EXECUTOR.start(Interrupt::CORDIC);

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P7);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    // commands channel
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();
    let radio_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    // Battery Channel
    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();
    let battery_volt_subscriber = BATTERY_VOLT_CHANNEL.subscriber().unwrap();

    // imu channel
    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let imu_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    let control_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();
    let control_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner,
        robot_state,
        battery_volt_publisher,
        p);

    create_dotstar_task!(main_spawner,
        led_command_subscriber,
        p);

    // TODO Come back to. Extra compute.
    // create_audio_task!(main_spawner,
    //    robot_state,
    //    p);

    create_radio_task!(main_spawner, radio_uart_queue_spawner, uart_queue_spawner,
        robot_state,
        radio_command_publisher, radio_telemetry_subscriber, radio_led_cmd_publisher,
        wifi_credentials,
        p);

    create_power_task!(main_spawner, uart_queue_spawner,
        robot_state,
        p);

    create_imu_task!(main_spawner,
        robot_state,
        imu_gyro_data_publisher, imu_accel_data_publisher, imu_led_cmd_publisher,
        p);

    create_control_task!(main_spawner, uart_queue_spawner, 
        robot_state, 
        control_command_subscriber, control_telemetry_publisher,
        battery_volt_subscriber,
        control_gyro_data_subscriber, control_accel_data_subscriber,
        p);

    create_kicker_task!(
        main_spawner, uart_queue_spawner,
        robot_state,
        kicker_command_subscriber,
        p);

    loop {
        Timer::after_millis(10).await;
    }
}