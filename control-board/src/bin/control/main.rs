#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt,
    pac::Interrupt
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _; 

use ateam_control_board::{get_system_config, pins::{AccelDataPubSub, CommandsPubSub, GyroDataPubSub, TelemetryPubSub}, tasks::{imu_task::start_imu_task, radio_task::start_radio_task}};


// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    // init system
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P6);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let _control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    // telemetry channel
    let _control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

    // TODO imu channel
    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    let wifi_network = wifi_credentials[0];
    start_radio_task(
        uart_queue_spawner, main_spawner,
        radio_command_publisher, radio_telemetry_subscriber,
        wifi_network,
        p.USART10, p.PE2, p.PE3, p.PG13, p.PG14,
        p.DMA2_CH1, p.DMA2_CH0,
        p.PC13, p.PE4);

    start_imu_task(&main_spawner,
        imu_gyro_data_publisher, imu_accel_data_publisher,
        p.SPI1, p.PA5, p.PA7, p.PA6,
        p.DMA2_CH7, p.DMA2_CH6,
        p.PA4, p.PC4, p.PC5,
        p.PB1, p.PB2, p.EXTI1, p.EXTI2,
        p.PF11);

    loop {
        Timer::after_millis(10).await;
    }
}