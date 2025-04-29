use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::Peripheral;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::spi::{SckPin, MisoPin, MosiPin};

use embassy_time::{Instant, Timer};
use nalgebra::Vector3;

use ateam_lib_stm32::drivers::imu::bmi323::{self, *};

use crate::pins::*;
use crate::robot_state::SharedRobotState;

const TIPPED_MIN_DURATION_MS: u64 = 1000;

#[macro_export]
macro_rules! create_imu_task {
    ($main_spawner:ident, $robot_state:ident, $imu_gyro_data_publisher:ident, $imu_accel_data_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::imu_task::start_imu_task(&$main_spawner,
            $robot_state,
            $imu_gyro_data_publisher, $imu_accel_data_publisher,
            $p.SPI1, $p.PA5, $p.PA7, $p.PA6,
            $p.DMA2_CH7, $p.DMA2_CH6,
            $p.PA4, $p.PA3, $p.PC4,
            $p.PB0, $p.PB1, $p.EXTI0, $p.EXTI1,
            $p.PB2);
    };
}

#[link_section = ".axisram.buffers"]
static mut IMU_BUFFER_CELL: [u8; bmi323::SPI_MIN_BUF_LEN] = [0; bmi323::SPI_MIN_BUF_LEN];


#[embassy_executor::task]
async fn imu_task_entry(
        robot_state: &'static SharedRobotState,
        gyro_pub: GyroDataPublisher,
        accel_pub: AccelDataPublisher,
        mut imu: Bmi323<'static, 'static>,
        mut _accel_int: ExtiInput<'static>,
        mut gyro_int: ExtiInput<'static>) {

    defmt::info!("imu start startup.");
    let mut first_tipped_check_time = Instant::now(); 
    let mut first_tipped_seen = false;

    'imu_configuration_loop:
    loop {
        // At the beginning, assume IMU is not working yet.
        robot_state.set_imu_inop(true);
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
            OutputDataRate::Odr100p0,
            DataAveragingWindow::Average2Samples).await;
        imu.set_gyro_interrupt_mode(InterruptMode::MappedToInt2).await;

        if gyro_config_res.is_err() {
            defmt::error!("gyro configration failed.");
        }
        
        // configure the gyro, map int to int pin 1
        let acc_config_res = imu.set_accel_config(AccelMode::ContinuousHighPerformance,
            AccelRange::Range2g,
            Bandwidth3DbCutoffFreq::AccOdrOver2,
            OutputDataRate::Odr100p0,
            DataAveragingWindow::Average2Samples).await;
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
                    // Got an interrupt, so IMU should be working.
                    robot_state.set_imu_inop(false);

                    // read gyro data
                    let imu_data = imu.gyro_get_data_rads().await;
                    gyro_pub.publish_immediate(Vector3::new(imu_data[0], imu_data[1], imu_data[2]));
            
                    // read accel data
                    // TODO: don't use raw data, impl conversion
                    let accel_data = imu.accel_get_data_mps().await;
                    accel_pub.publish_immediate(Vector3::new(accel_data[0] as f32, accel_data[1] as f32, accel_data[2] as f32));

                    // TODO: magic number, fix after raw data conversion
                    if accel_data[2] < 4.0 {
                        if !first_tipped_seen {
                            // If it's the first time a tipping occured, start tracking.
                            first_tipped_seen = true;
                            first_tipped_check_time = Instant::now();
                        } else {
                            // After the first tipped is seen, then wait if it has been tipped for long enough.
                            let cur_time = Instant::now();
                            if Instant::checked_duration_since(&cur_time, first_tipped_check_time).unwrap().as_millis() > TIPPED_MIN_DURATION_MS {
                                robot_state.set_robot_tipped(true);
                            } else {
                                // If it hasn't been long enough, clear the robot tipped.
                                robot_state.set_robot_tipped(false);
                            }
                        }
                    } else {
                        // Not tipped so clear everything. 
                        first_tipped_seen = false;
                        robot_state.set_robot_tipped(false);
                    }
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
        robot_state: &'static SharedRobotState,
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
    defmt::debug!("starting imu task...");

    // let imu_buf = IMU_BUFFER_CELL.take();
    // let imu_buf: &'static mut [u8; 14] = unsafe { & mut IMU_BUFFER_CELL };
    let imu_buf: & mut [u8; bmi323::SPI_MIN_BUF_LEN] = unsafe { &mut (*(&raw mut IMU_BUFFER_CELL)) };

    let imu = Bmi323::new_from_pins(peri, sck, mosi, miso, txdma, rxdma, bmi323_nss, imu_buf);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::None);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::None);

    imu_task_spawner.spawn(imu_task_entry(robot_state, gyro_data_publisher, accel_data_publisher, imu, accel_int, gyro_int)).unwrap();
}

