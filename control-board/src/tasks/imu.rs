use embassy_stm32::Peripheral;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::spi::{SckPin, MisoPin, MosiPin};
use embassy_sync::{pubsub::Publisher, blocking_mutex::raw::RawMutex};

use nalgebra::Vector3;

use static_cell::StaticCell;

use crate::drivers::imu::bmi085::{Bmi085, GyroRange, GyroBandwidth, GyroIntMap, GyroIntPinActiveState, GyroIntPinMode};
use crate::drivers::imu::{GyroFrame, AccelFrame};

use crate::pins::{ImuSpi, ImuTxDma, ImuRxDma, ImuAccelCsPin, ImuGyroCsPin, ImuAccelIntPin, ImuGyroIntPin};

#[link_section = ".axisram.buffers"]
static IMU_BUFFER_CELL: StaticCell<[u8; 8]> = StaticCell::new();
// static mut IMU_BUF: [u8; 8] = [0; 8];

// #[embassy_executor::task]
pub async fn imu_task<'a,
        GyroMutex: RawMutex,
        const GYRO_CAP: usize,
        const GYRO_NSUB: usize,
        const GYRO_NPUB: usize,
        AccelMutex: RawMutex,
        const ACCEL_CAP: usize,
        const ACCEL_NSUB: usize,
        const ACCEL_NPUB: usize>(
    gyro_pub: Publisher<'a, GyroMutex, GyroFrame, GYRO_CAP, GYRO_NSUB, GYRO_NPUB>,
    accel_pub: Publisher<'a, AccelMutex, AccelFrame, ACCEL_CAP, ACCEL_NSUB, ACCEL_NPUB>,
    imu_spi: impl Peripheral<P = ImuSpi> + 'a,
    sck: impl Peripheral<P = impl SckPin<ImuSpi>> + 'a,
    mosi: impl Peripheral<P = impl MosiPin<ImuSpi>> + 'a,
    miso: impl Peripheral<P = impl MisoPin<ImuSpi>> + 'a,
    txdma: impl Peripheral<P = ImuTxDma> + 'a,
    rxdma: impl Peripheral<P = ImuRxDma> + 'a,
    accel_cs: impl Peripheral<P = ImuAccelCsPin> + 'a,
    gyro_cs: impl Peripheral<P = ImuGyroCsPin> + 'a,
    accel_int_pin: impl Peripheral<P = ImuAccelIntPin> + 'a,
    gyro_int_pin: impl Peripheral<P = ImuGyroIntPin> + 'a,
    accel_int: impl Peripheral<P = <ImuAccelIntPin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'a,
    gyro_int: impl Peripheral<P = <ImuGyroIntPin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'a,
    ) {
    
    let imu_buf = IMU_BUFFER_CELL.init([0; 8]);
    let mut imu = Bmi085::new_from_pins(imu_spi, sck, mosi, miso, txdma, rxdma, accel_cs, gyro_cs, imu_buf);

    let accel_int_input = Input::new(accel_int_pin, Pull::Down);
    let _accel_int = ExtiInput::new(accel_int_input, accel_int);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let gyro_int_input = Input::new(gyro_int_pin, Pull::Up);
    let mut gyro_int = ExtiInput::new(gyro_int_input, gyro_int);
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
        let imu_data = imu.gyro_get_data().await;
        gyro_pub.publish_immediate(Vector3::new(imu_data[0], imu_data[1], imu_data[2]));

        // read accel data
        // TODO: don't use raw data, impl conversion
        let accel_data = imu.accel_get_raw_data().await;
        accel_pub.publish_immediate(Vector3::new(accel_data[0] as f32, accel_data[1] as f32, accel_data[2] as f32));
    }
}