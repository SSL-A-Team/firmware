use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::Peripheral;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::spi::{SckPin, MisoPin, MosiPin};
use embassy_sync::pubsub::{PubSubChannel, Error};

use embassy_time::Timer;
use nalgebra::Vector3;

use static_cell::ConstStaticCell;

use ateam_lib_stm32::drivers::imu::bmi085::{Bmi085, GyroRange, GyroBandwidth, GyroIntMap, GyroIntPinActiveState, GyroIntPinMode, AccelRange, AccelConfOdr, AccelConfBwp};

use crate::pins::{
    AccelDataPublisher, ExtImuNDetPin, ExtImuSpiNss1Pin, ExtImuSpiNss2Pin, ExtImuSpiNss3Pin, GyroDataPublisher, ImuSpi, ImuSpiInt1Exti, ImuSpiInt1Pin, ImuSpiInt2Exti, ImuSpiInt2Pin, ImuSpiMisoPin, ImuSpiMosiPin, ImuSpiNss0Pin, ImuSpiRxDma, ImuSpiSckPin, ImuSpiTxDma};

#[link_section = ".axisram.buffers"]
static IMU_BUFFER_CELL: ConstStaticCell<[u8; 8]> = ConstStaticCell::new([0; 8]);

#[embassy_executor::task]
async fn imu_task_entry(
        accel_pub: AccelDataPublisher,
        gyro_pub: GyroDataPublisher,
        mut imu: Bmi085<'static, 'static, ImuSpi>,
        mut _accel_int: ExtiInput<'static>,
        mut gyro_int: ExtiInput<'static>) {

    defmt::info!("imu start startup.");

    'imu_configuration_loop:
    loop {
        imu.init().await;
        let self_test_res = imu.self_test().await;
        if self_test_res.is_err() {
            defmt::error!("IMU self test failed");
            Timer::after_millis(1000).await;
            continue 'imu_configuration_loop;
        }
    
        // set measurement ranges
        imu.accel_set_range(AccelRange::Range2g).await;  // range +- 19.6 m/s^2
        imu.accel_set_bandwidth(AccelConfBwp::OverSampling2Fold, AccelConfOdr::OutputDataRate400p0).await;  // bandwidth 80Hz
    
        // set interrupt config
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
    
        'imu_data_loop:
        loop {
            // block on gyro interrupt, active low
            match select(gyro_int.wait_for_falling_edge(), Timer::after_millis(1000)).await {
                Either::First(_) => {
                    // read gyro data
                    let imu_data = imu.gyro_get_data_rads().await;
                    gyro_pub.publish_immediate(Vector3::new(imu_data[0], imu_data[1], imu_data[2]));
            
                    // read accel data
                    // TODO: don't use raw data, impl conversion
                    let accel_data = imu.accel_get_raw_data().await;
                    accel_pub.publish_immediate(Vector3::new(accel_data[0] as f32, accel_data[1] as f32, accel_data[2] as f32));
                }
                Either::Second(_) => {
                    defmt::warn!("imu interrupt based data acq timed out.");
                    // attempt connect validation and reconfig
                    break 'imu_data_loop
                }
            };
        }
    }


}

pub fn start_imu_task(
        imu_task_spawner: &Spawner,
        gyro_data_publisher: GyroDataPublisher,
        accel_data_publisher: AccelDataPublisher,
        peri: impl Peripheral<P = ImuSpi> + 'static,
        sck: impl Peripheral<P = impl SckPin<ImuSpi>> + 'static,
        mosi: impl Peripheral<P = impl MosiPin<ImuSpi>> + 'static,
        miso: impl Peripheral<P = impl MisoPin<ImuSpi>> + 'static,
        txdma: impl Peripheral<P = ImuSpiTxDma> + 'static,
        rxdma: impl Peripheral<P = ImuSpiRxDma> + 'static,
        accel_cs: ExtImuSpiNss1Pin,
        gyro_cs: ExtImuSpiNss2Pin,
        accel_int_pin: impl Peripheral<P = ImuSpiInt1Pin> + 'static,
        gyro_int_pin: impl Peripheral<P = ImuSpiInt2Pin> + 'static,
        accel_int: impl Peripheral<P = <ImuSpiInt1Pin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
        gyro_int: impl Peripheral<P = <ImuSpiInt2Pin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
        ext_imu_det_pin: ExtImuNDetPin) {
    let imu_buf = IMU_BUFFER_CELL.take();

    let imu = Bmi085::new_from_pins(peri, sck, mosi, miso, txdma, rxdma, accel_cs, gyro_cs, imu_buf);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::Down);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::Up);

    imu_task_spawner.spawn(imu_task_entry(accel_data_publisher, gyro_data_publisher, imu, accel_int, gyro_int)).unwrap();
}

