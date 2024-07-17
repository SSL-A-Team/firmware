use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::Peripheral;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::spi::{SckPin, MisoPin, MosiPin};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Subscriber, Error};

use nalgebra::Vector3;

use static_cell::StaticCell;
use static_cell::make_static;

use crate::drivers::imu::bmi085::{Bmi085, GyroRange, GyroBandwidth, GyroIntMap, GyroIntPinActiveState, GyroIntPinMode, AccelRange, AccelConfOdr, AccelConfBwp};
use crate::drivers::imu::{GyroFrame, AccelFrame};

use crate::pins::{ImuSpi, ImuTxDma, ImuRxDma, ImuAccelCsPin, ImuGyroCsPin, ImuAccelIntPin, ImuGyroIntPin};

type AccelDataChannel = PubSubChannel<ThreadModeRawMutex, AccelFrame, 1, 1, 1>;
type GyroDataChannel = PubSubChannel<ThreadModeRawMutex, GyroFrame, 1, 1, 1>;

pub type AccelSub<'a> = Subscriber<'a, ThreadModeRawMutex, AccelFrame, 1, 1, 1>;
pub type GyroSub<'a> = Subscriber<'a, ThreadModeRawMutex, GyroFrame, 1, 1, 1>;

// #[link_section = ".axisram.buffers"]
static IMU_BUFFER_CELL: StaticCell<[u8; 8]> = StaticCell::new();
// let imu_buf = make_static!([0u8; 8], #[link_section = ".axisram.buffers"]);

static ACCEL_DATA_CHANNEL: AccelDataChannel = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataChannel = PubSubChannel::new();

#[embassy_executor::task]
async fn imu_task_entry(mut imu: Bmi085<'static, 'static, ImuSpi, ImuTxDma, ImuRxDma, ImuAccelCsPin, ImuGyroCsPin>, mut _accel_int: ExtiInput<'static, ImuAccelIntPin>, mut gyro_int: ExtiInput<'static, ImuGyroIntPin>) {
    let accel_pub = ACCEL_DATA_CHANNEL.publisher().expect("accel data channel had no publisher left");
    let gyro_pub = GYRO_DATA_CHANNEL.publisher().expect("gyro data channel had no publisher left");

    imu.init().await;
    let self_test_res = imu.self_test().await;
    if self_test_res.is_err() {
        defmt::error!("IMU self test failed");
    }

    imu.accel_set_range(AccelRange::Range2g).await;  // range +- 19.6 m/s^2
    imu.accel_set_bandwidth(AccelConfBwp::OverSampling2Fold, AccelConfOdr::OutputDataRate400p0).await;  // bandwidth 80Hz

    imu.gyro_set_int_config(GyroIntPinActiveState::ActiveLow,
        GyroIntPinMode::OpenDrain,
        GyroIntPinActiveState::ActiveLow,
        GyroIntPinMode::OpenDrain).await;
    // enable BMI085 Gyro INT3 which is electrically wired to IMU breakout INT2 named gyro_int_pin
    imu.gyro_set_int_map(GyroIntMap::Int3).await;

    imu.gyro_set_range(GyroRange::PlusMinus2000DegPerSec).await;
    imu.gyro_set_bandwidth(GyroBandwidth::FilterBw64Hz).await;

    // enable interrupts
    imu.gyro_enable_interrupts().await;

    // BMI085 internally shares a SPI bus so we can't actually asynchronously read both the IMU and Accel
    // unless we used a mutex but that's still a lot of overhead. We'll just sync on the Gyro INT since
    // that gyro data is far more critical for control. When the gyro has an update, we'll also grab the 
    // accel update/

    loop {
        // block on gyro interrupt, active low
        gyro_int.wait_for_falling_edge().await;

        // read gyro data
        let imu_data = imu.gyro_get_data_rads().await;
        gyro_pub.publish_immediate(Vector3::new(imu_data[0], imu_data[1], imu_data[2]));

        // read accel data
        // TODO: don't use raw data, impl conversion
        let accel_data = imu.accel_get_raw_data().await;
        accel_pub.publish_immediate(Vector3::new(accel_data[0] as f32, accel_data[1] as f32, accel_data[2] as f32));
    }
}

pub fn start_imu_task(spawner: &Spawner, 
        peri: impl Peripheral<P = ImuSpi> + 'static,
        sck: impl Peripheral<P = impl SckPin<ImuSpi>> + 'static,
        mosi: impl Peripheral<P = impl MosiPin<ImuSpi>> + 'static,
        miso: impl Peripheral<P = impl MisoPin<ImuSpi>> + 'static,
        txdma: impl Peripheral<P = ImuTxDma> + 'static,
        rxdma: impl Peripheral<P = ImuRxDma> + 'static,
        accel_cs: impl Peripheral<P = ImuAccelCsPin> + 'static,
        gyro_cs: impl Peripheral<P = ImuGyroCsPin> + 'static,
        accel_int_pin: impl Peripheral<P = ImuAccelIntPin> + 'static,
        gyro_int_pin: impl Peripheral<P = ImuGyroIntPin> + 'static,
        accel_int: impl Peripheral<P = <ImuAccelIntPin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
        gyro_int: impl Peripheral<P = <ImuGyroIntPin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
    ) -> Result<(), SpawnError> {

    defmt::warn!("here3!");

    let imu_buf = IMU_BUFFER_CELL.init([0; 8]);
    // let imu_buf = make_static!([0u8; 8], #[link_section = ".axisram.buffers"]);

    defmt::warn!("here4!");

    let imu = Bmi085::new_from_pins(peri, sck, mosi, miso, txdma, rxdma, accel_cs, gyro_cs, imu_buf);

    let accel_int_input = Input::new(accel_int_pin, Pull::Down);
    let accel_int = ExtiInput::new(accel_int_input, accel_int);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let gyro_int_input = Input::new(gyro_int_pin, Pull::Up);
    let gyro_int = ExtiInput::new(gyro_int_input, gyro_int);

    defmt::warn!("here!");

    spawner.spawn(imu_task_entry(imu, accel_int, gyro_int))
}

pub fn get_accel_sub() -> Result<AccelSub<'static>, Error> {
    return ACCEL_DATA_CHANNEL.subscriber();
}

pub fn get_gyro_sub() -> Result<GyroSub<'static>, Error> {
    return GYRO_DATA_CHANNEL.subscriber();
}
