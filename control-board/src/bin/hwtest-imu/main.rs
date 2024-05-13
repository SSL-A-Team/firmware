#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_futures::select::{self, Either3};
use embassy_stm32::interrupt;
use embassy_sync::pubsub::{PubSubChannel, WaitResult};

use defmt_rtt as _; 

use ateam_control_board::{get_system_config, pins::{AccelDataPubSub, GyroDataPubSub}, tasks::imu_task::start_imu_task};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;

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


    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let mut imu_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let mut imu_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    start_imu_task(&main_spawner,
        imu_gyro_data_publisher, imu_accel_data_publisher,
        p.SPI1, p.PA5, p.PA7, p.PA6,
        p.DMA2_CH7, p.DMA2_CH6,
        p.PA4, p.PC4, p.PC5,
        p.PB1, p.PB2, p.EXTI1, p.EXTI2,
        p.PF11);

    loop {
        match select::select3(imu_gyro_data_subscriber.next_message(), imu_accel_data_subscriber.next_message(), Timer::after_millis(1000)).await {
            Either3::First(gyro_data) => {
                match gyro_data {
                    WaitResult::Lagged(amnt) => {
                        defmt::warn!("publishing gyro data lagged by {}", amnt);
                    }
                    WaitResult::Message(msg) => {
                        defmt::info!("got gyro data (x: {}, y: {}, z: {})", msg[1], msg[1], msg[2]);

                    }
                }
            }
            Either3::Second(accel_data) => {
                match accel_data {
                    WaitResult::Lagged(amnt) => {
                        defmt::warn!("publishing accel data lagged by {}", amnt);
                    }
                    WaitResult::Message(msg) => {
                        defmt::info!("got accel data (x: {}, y: {}, z: {})", msg[1], msg[1], msg[2]);

                    }
                }
            }
            Either3::Third(_) => {
                defmt::warn!("receiving timed out.");
            }
        }
    }
}