use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::Peripheral;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::spi::{SckPin, MisoPin, MosiPin};

use embassy_time::Timer;
use nalgebra::Vector3;

use static_cell::ConstStaticCell;

use ateam_lib_stm32::drivers::imu::bmi323::{self, *};

use crate::pins::*;

#[macro_export]
macro_rules! create_imu_task {
    ($main_spawner:ident, $imu_gyro_data_publisher:ident, $imu_accel_data_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::imu_task::start_imu_task(&$main_spawner,
            $imu_gyro_data_publisher, $imu_accel_data_publisher,
            $p.SPI1, $p.PA5, $p.PA7, $p.PA6,
            $p.DMA2_CH7, $p.DMA2_CH6,
            $p.PA4, $p.PC4, $p.PC5,
            $p.PB1, $p.PB2, $p.EXTI1, $p.EXTI2,
            $p.PF11);
    };
}

#[link_section = ".axisram.buffers"]
static IMU_BUFFER_CELL: ConstStaticCell<[u8; bmi323::SPI_MIN_BUF_LEN]> = ConstStaticCell::new([0; bmi323::SPI_MIN_BUF_LEN]);

#[embassy_executor::task]
async fn imu_task_entry(
        accel_pub: AccelDataPublisher,
        gyro_pub: GyroDataPublisher,
        mut imu: Bmi323<'static, 'static>,
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
    
        // configure the gyro, map int to int pin 2
        let gyro_config_res = imu.set_gyro_config(GyroMode::ContinuousHighPerformance,
            GyroRange::PlusMinus2000DegPerSec,
            Bandwidth3DbCutoffFreq::AccOdrOver2,
            OutputDataRate::Odr6400p0,
            DataAveragingWindow::Average64Samples).await;
        imu.set_gyro_interrupt_mode(InterruptMode::MappedToInt2).await;

        if gyro_config_res.is_err() {
            defmt::error!("gyro configration failed.");
        }
        
        // configure the gyro, map int to int pin 1
        let acc_config_res = imu.set_accel_config(AccelMode::ContinuousHighPerformance,
            AccelRange::Range2g,
            Bandwidth3DbCutoffFreq::AccOdrOver2,
            OutputDataRate::Odr6400p0,
            DataAveragingWindow::Average64Samples).await;
        imu.set_accel_interrupt_mode(InterruptMode::MappedToInt1).await;

        if acc_config_res.is_err() {
            defmt::error!("accel configration failed.");
        }

        // configure the phys properties of the int pins
        imu.set_int1_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull).await;
        imu.set_int2_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull).await;

        // enable gyro int
        imu.set_int2_enabled(true).await;

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
        bmi323_nss: ImuSpiNss0Pin,
        _ext_nss1_pin: ExtImuSpiNss1Pin,
        _ext_nss2_pin: ExtImuSpiNss2Pin,
        accel_int_pin: impl Peripheral<P = ImuSpiInt1Pin> + 'static,
        gyro_int_pin: impl Peripheral<P = ImuSpiInt2Pin> + 'static,
        accel_int: impl Peripheral<P = <ImuSpiInt1Pin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
        gyro_int: impl Peripheral<P = <ImuSpiInt2Pin as embassy_stm32::gpio::Pin>::ExtiChannel> + 'static,
        _ext_imu_det_pin: ExtImuNDetPin) {
    let imu_buf = IMU_BUFFER_CELL.take();

    let imu = Bmi323::new_from_pins(peri, sck, mosi, miso, txdma, rxdma, bmi323_nss, imu_buf);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::None);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::None);

    imu_task_spawner.spawn(imu_task_entry(accel_data_publisher, gyro_data_publisher, imu, accel_int, gyro_int)).unwrap();
}

