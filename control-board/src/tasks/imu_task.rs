use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals::FLASH;
use embassy_stm32::spi::{MisoPin, MosiPin, SckPin};

use embassy_stm32::Peri;
use embassy_time::{Instant, Timer};
use nalgebra::Vector3;

use ateam_common_packets::radio::TelemetryPacket;
use ateam_lib_stm32::drivers::imu::bmi323::{self, *};
use ateam_lib_stm32::filter::{Filter, IirFilter};

use crate::create_error_telemetry_from_string;
use crate::imu_calibration::{load_calibration, store_calibration, ImuCalibration};
use crate::pins::*;
use crate::robot_state::SharedRobotState;
use crate::tasks::dotstar_task::{ControlBoardLedCommand, ImuStatusLedCommand};

const TIPPED_MIN_DURATION_MS: u64 = 1000;

/// Shared output data rate for both the accelerometer and gyroscope. Defined once so the
/// two sensors always run at the same rate (the accel is sampled on the gyro data-ready
/// interrupt, so they must match) and so the accel filter sample rate below stays in sync.
const IMU_ODR: OutputDataRate = OutputDataRate::Odr1600p0;

/// The accelerometer is sampled on the gyro data-ready interrupt, so the effective sample
/// rate of the firmware accel filter is the shared IMU ODR.
const ACCEL_FILTER_SAMPLE_RATE_HZ: f32 = IMU_ODR.to_hz();
/// -3 dB cutoff of the firmware low-pass filter applied to the X and Y accelerations to
/// reject motor/wheel vibration before the data is consumed by the state estimator.
const ACCEL_FILTER_CUTOFF_HZ: f32 = 40.0;

/// Number of stationary/upright samples averaged to estimate the accelerometer
/// (X/Y) bias when running a fresh on-chip calibration. ~2.0 s at the IMU ODR
/// (1600 Hz). The gyro bias is handled by the sensor's built-in self-calibration,
/// so only the accel bias is averaged in firmware.
const ACCEL_CALIBRATION_SAMPLES: u32 = 2 * 1600;
/// Accel Z below this magnitude (m/s^2) is treated as tipped / not upright. The robot must
/// be upright and stationary for the boot-time bias calibration to accumulate.
const ACCEL_TIPPED_Z_MPS2: f32 = 4.0;

/// Developer flag to force a fresh on-chip IMU (re)calibration even when a valid
/// calibration is already stored in flash. Flip to `true`, build/flash, boot once
/// while the robot is upright and stationary to overwrite the stored calibration,
/// then set back to `false`. Left `false` for normal operation.
const FORCE_IMU_RECALIBRATION: bool = false;

#[macro_export]
macro_rules! create_imu_task {
    ($main_spawner:ident, $robot_state:ident, $imu_gyro_data_publisher:ident, $imu_accel_data_publisher:ident, $imu_led_cmd_pub:ident, $imu_telemetry_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::imu_task::start_imu_task(
            &$main_spawner,
            $robot_state,
            $imu_gyro_data_publisher,
            $imu_accel_data_publisher,
            $imu_led_cmd_pub,
            $imu_telemetry_publisher,
            $p.SPI1,
            $p.PA5,
            $p.PA7,
            $p.PA6,
            $p.DMA2_CH7,
            $p.DMA2_CH6,
            $p.PA4,
            $p.PA3,
            $p.PC4,
            $p.PB0,
            $p.PB1,
            $p.EXTI0,
            $p.EXTI1,
            $p.PB2,
            $p.FLASH,
        );
    };
}

#[macro_export]
macro_rules! create_imu_task_ie {
    ($main_spawner:ident, $robot_state:ident, $imu_gyro_data_publisher:ident, $imu_accel_data_publisher:ident, $imu_led_cmd_pub:ident, $imu_telemetry_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::imu_task::start_imu_task_ie(
            &$main_spawner,
            $robot_state,
            $imu_gyro_data_publisher,
            $imu_accel_data_publisher,
            $imu_led_cmd_pub,
            $imu_telemetry_publisher,
            $p.SPI1,
            $p.PA5,
            $p.PA7,
            $p.PA6,
            $p.DMA2_CH7,
            $p.DMA2_CH6,
            $p.PA4,
            $p.PA3,
            $p.PC4,
            $p.PB0,
            $p.PB1,
            $p.EXTI0,
            $p.EXTI1,
            $p.PB2,
            $p.FLASH,
        );
    };
}

#[link_section = ".axisram.buffers"]
static mut IMU_BUFFER_CELL: [u8; bmi323::SPI_MIN_BUF_LEN] = [0; bmi323::SPI_MIN_BUF_LEN];

#[embassy_executor::task]
async fn imu_task_entry(
    robot_state: &'static SharedRobotState,
    gyro_pub: GyroDataPublisher,
    accel_pub: AccelDataPublisher,
    led_command_pub: LedCommandPublisher,
    telemetry_pub: TelemetryPublisher,
    mut imu: Bmi323<'static, 'static>,
    mut _accel_int: ExtiInput<'static, embassy_stm32::mode::Async>,
    mut gyro_int: ExtiInput<'static, embassy_stm32::mode::Async>,
    mut flash: Flash<'static, Blocking>,
) {
    defmt::info!("imu start startup.");
    let mut first_tipped_check_time = Instant::now();
    let mut first_tipped_seen = false;

    // Firmware low-pass filters for the X and Y accelerations. The BMI323's on-chip filter
    // can't reach a low enough cutoff without sacrificing sample freshness, so the final
    // vibration rejection is done here. Z is left unfiltered so tipped detection stays
    // responsive.
    let mut accel_x_filter =
        IirFilter::from_cutoff(ACCEL_FILTER_CUTOFF_HZ, ACCEL_FILTER_SAMPLE_RATE_HZ);
    let mut accel_y_filter =
        IirFilter::from_cutoff(ACCEL_FILTER_CUTOFF_HZ, ACCEL_FILTER_SAMPLE_RATE_HZ);

    'imu_configuration_loop: loop {
        led_command_pub
            .publish(ControlBoardLedCommand::Imu(
                ImuStatusLedCommand::Configuring,
            ))
            .await;

        // At the beginning, assume IMU is not working yet.
        robot_state.set_imu_inop(true);
        imu.init().await;
        let self_test_res = imu.self_test().await;
        if self_test_res.is_err() {
            defmt::error!("IMU self test failed");
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU self test failed"),
            ));
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            Timer::after_millis(1000).await;
            continue 'imu_configuration_loop;
        }

        // The IMU is mounted rotated 180 degrees about the board Z axis. Correct for this
        // on-chip via the feature engine so both accel and gyro report in the robot frame at
        // zero runtime cost. A 180 degree rotation about Z is X -> -X, Y -> -Y, Z -> Z, i.e.
        // pure sign inversion of the X and Y axes. This must be done while the sensors are
        // inactive (before the accel/gyro config below) and re-applied on every (re)config,
        // since the mapping is cleared by the soft reset performed in imu.init().
        if imu.enable_feature_engine().await.is_err() {
            defmt::error!("IMU feature engine enable failed");
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU feature engine enable failed"),
            ));
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            Timer::after_millis(1000).await;
            continue 'imu_configuration_loop;
        }
        if imu
            .set_axis_remap(AxisMap::XyzToXyz, true, true, false)
            .await
            .is_err()
        {
            defmt::error!("IMU axis remap failed");
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU axis remap failed"),
            ));
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            Timer::after_millis(1000).await;
            continue 'imu_configuration_loop;
        }

        // configure the gyro, map int to int pin 2
        let gyro_config_res = imu
            .set_gyro_config(
                GyroMode::ContinuousHighPerformance,
                GyroRange::PlusMinus2000DegPerSec,
                Bandwidth3DbCutoffFreq::AccOdrOver4,
                IMU_ODR,
                DataAveragingWindow::NoFiltering,
            )
            .await;
        imu.set_gyro_interrupt_mode(InterruptMode::MappedToInt2)
            .await;

        if gyro_config_res.is_err() {
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            defmt::error!("gyro configration failed.");
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU gyro configuration failed"),
            ));
        }

        // configure the gyro, map int to int pin 1
        let acc_config_res = imu
            .set_accel_config(
                AccelMode::ContinuousHighPerformance,
                AccelRange::Range4g,
                Bandwidth3DbCutoffFreq::AccOdrOver4,
                IMU_ODR,
                DataAveragingWindow::NoFiltering,
            )
            .await;
        imu.set_accel_interrupt_mode(InterruptMode::MappedToInt1)
            .await;

        if acc_config_res.is_err() {
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            defmt::error!("accel configration failed.");
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU accel configuration failed"),
            ));
        }

        // configure the phys properties of the int pins
        imu.set_int1_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull)
            .await;
        imu.set_int2_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull)
            .await;

        // enable gyro int
        imu.set_int2_enabled(true).await;

        led_command_pub
            .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Ok))
            .await;

        // Clear any stale/transient filter state before starting to publish fresh samples.
        accel_x_filter.reset();
        accel_y_filter.reset();

        // Establish the on-chip IMU bias calibration before publishing any data. The
        // BMI323 corrects bias internally via its data-path offset registers (which are
        // volatile and cleared by the soft reset in imu.init()). Prefer a calibration
        // previously stored in flash; otherwise run the sensor's built-in gyro
        // self-calibration plus a firmware accel X/Y bias estimate and persist the
        // result. A fresh calibration requires the robot to be upright and stationary
        // and is retried until it succeeds. After this returns, every published sample
        // is already bias-corrected by the sensor.
        establish_imu_calibration(&mut imu, &mut gyro_int, &mut flash, &telemetry_pub).await;

        // Drop any transient accumulated on the accel filters during calibration.
        accel_x_filter.reset();
        accel_y_filter.reset();

        'imu_data_loop: loop {
            // block on gyro interrupt, active low
            match select(gyro_int.wait_for_falling_edge(), Timer::after_millis(1000)).await {
                Either::First(_) => {
                    // Got an interrupt, so IMU should be working.
                    robot_state.set_imu_inop(false);

                    // read gyro and accel data (already bias-corrected on-chip via the
                    // data-path offset registers established during calibration).
                    let imu_data = imu.gyro_get_data_rads().await;
                    // TODO: don't use raw data, impl conversion
                    let accel_data = imu.accel_get_data_mps().await;

                    // Publish the (chip-corrected) gyro.
                    gyro_pub.publish_immediate(Vector3::new(
                        imu_data[0],
                        imu_data[1],
                        imu_data[2],
                    ));

                    // Low-pass filter the (chip-corrected) accel X/Y to reject motor/wheel
                    // vibration. Z is published unfiltered for responsive tipped detection
                    // below (and is intentionally left with no on-chip offset so it keeps
                    // measuring gravity).
                    let accel_x_unbiased = accel_data[0] as f32;
                    let accel_y_unbiased = accel_data[1] as f32;
                    accel_x_filter.add_sample(accel_x_unbiased);
                    accel_y_filter.add_sample(accel_y_unbiased);
                    let accel_x_filtered = accel_x_filter.filtered_value().unwrap_or(accel_x_unbiased);
                    let accel_y_filtered = accel_y_filter.filtered_value().unwrap_or(accel_y_unbiased);

                    accel_pub.publish_immediate(Vector3::new(
                        accel_x_filtered,
                        accel_y_filtered,
                        accel_data[2] as f32,
                    ));

                    // TODO: magic number, fix after raw data conversion
                    if (accel_data[2] as f32) < ACCEL_TIPPED_Z_MPS2 {
                        if !first_tipped_seen {
                            // If it's the first time a tipping occured, start tracking.
                            first_tipped_seen = true;
                            first_tipped_check_time = Instant::now();
                        } else {
                            // After the first tipped is seen, then wait if it has been tipped for long enough.
                            let cur_time = Instant::now();
                            if Instant::duration_since(&cur_time, first_tipped_check_time)
                                .as_millis()
                                > TIPPED_MIN_DURATION_MS
                            {
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
                    telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                        create_error_telemetry_from_string("IMU interrupt timeout"),
                    ));
                    // attempt connect validation and reconfig
                    break 'imu_data_loop;
                }
            };
        }
    }
}

/// Establishes the on-chip IMU bias calibration before data publishing begins.
///
/// Applies a stored calibration from flash if one is present and valid; otherwise
/// runs the sensor's built-in gyro self-calibration plus a firmware accelerometer
/// X/Y bias estimate, applies both to the sensor's data-path offset registers, and
/// persists the result to flash. A fresh calibration is retried until it succeeds
/// (the robot must be upright and stationary).
async fn establish_imu_calibration(
    imu: &mut Bmi323<'static, 'static>,
    gyro_int: &mut ExtiInput<'static, embassy_stm32::mode::Async>,
    flash: &mut Flash<'static, Blocking>,
    telemetry_pub: &TelemetryPublisher,
) {
    if !FORCE_IMU_RECALIBRATION {
        if let Some(cal) = load_calibration(flash) {
            imu.write_gyro_dp_offset_gain(&cal.gyro).await;
            imu.write_accel_dp_offset(&cal.accel).await;
            defmt::info!(
                "IMU calibration restored from flash: gyro_off=[{}, {}, {}], accel_off_xy=[{}, {}]",
                cal.gyro.off_x,
                cal.gyro.off_y,
                cal.gyro.off_z,
                cal.accel.off_x,
                cal.accel.off_y,
            );
            return;
        }
        defmt::info!("no valid stored IMU calibration; running on-chip calibration");
    } else {
        defmt::warn!("FORCE_IMU_RECALIBRATION set; running on-chip calibration");
    }

    loop {
        // Built-in gyro self-calibration (offset). Requires the device stationary.
        let gyro_cal = match imu.perform_gyro_self_calibration(true, false).await {
            Ok(c) => c,
            Err(_) => {
                defmt::warn!("gyro self-calibration failed (hold robot stationary); retrying");
                telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                    create_error_telemetry_from_string(
                        "IMU gyro self-calibration failed; retrying",
                    ),
                ));
                Timer::after_millis(500).await;
                continue;
            }
        };

        // Firmware accel X/Y bias estimate while upright and stationary. The accel DP
        // offset registers are still zero here (cleared by the soft reset in
        // imu.init()), so the measured average is the true bias.
        let Some((bias_x_counts, bias_y_counts)) = average_accel_bias(imu, gyro_int).await
        else {
            defmt::warn!("accel bias estimate aborted (not upright/stationary); retrying");
            continue;
        };

        let accel = AccelDpOffset {
            off_x: imu.accel_bias_counts_to_dp_offset(bias_x_counts),
            off_y: imu.accel_bias_counts_to_dp_offset(bias_y_counts),
            // Leave Z uncorrected so it keeps measuring gravity for tipped detection.
            off_z: 0,
        };
        imu.write_accel_dp_offset(&accel).await;

        let cal = ImuCalibration {
            gyro: gyro_cal,
            accel,
        };
        if store_calibration(flash, &cal).is_ok() {
            defmt::info!(
                "IMU calibrated on-chip and stored to flash: gyro_off=[{}, {}, {}], accel_off_xy=[{}, {}]",
                cal.gyro.off_x,
                cal.gyro.off_y,
                cal.gyro.off_z,
                cal.accel.off_x,
                cal.accel.off_y,
            );
        } else {
            defmt::warn!(
                "IMU calibrated on-chip but flash persistence failed (applied this session only)"
            );
        }
        return;
    }
}

/// Averages [`ACCEL_CALIBRATION_SAMPLES`] upright accelerometer samples (taken on the
/// gyro data-ready interrupt) and returns the mean raw X/Y counts, i.e. the accel
/// bias. Returns `None` if a non-upright sample or a data timeout is seen, so the
/// caller can retry — ensuring the estimate is only taken while the robot is upright.
async fn average_accel_bias(
    imu: &mut Bmi323<'static, 'static>,
    gyro_int: &mut ExtiInput<'static, embassy_stm32::mode::Async>,
) -> Option<(i16, i16)> {
    let mut sum_x: i32 = 0;
    let mut sum_y: i32 = 0;
    let mut count: u32 = 0;

    while count < ACCEL_CALIBRATION_SAMPLES {
        match select(gyro_int.wait_for_falling_edge(), Timer::after_millis(1000)).await {
            Either::First(_) => {
                let raw = imu.accel_get_raw_data().await;
                if imu.convert_accel_raw_sample_mps(raw[2]) < ACCEL_TIPPED_Z_MPS2 {
                    // Not upright (tipped/moving); abort so the caller retries.
                    return None;
                }
                sum_x += raw[0] as i32;
                sum_y += raw[1] as i32;
                count += 1;
            }
            Either::Second(_) => {
                defmt::warn!("accel bias estimate timed out waiting for IMU data");
                return None;
            }
        }
    }

    let n = count as i32;
    Some(((sum_x / n) as i16, (sum_y / n) as i16))
}

pub fn start_imu_task(
    imu_task_spawner: &Spawner,
    robot_state: &'static SharedRobotState,
    gyro_data_publisher: GyroDataPublisher,
    accel_data_publisher: AccelDataPublisher,
    led_cmd_publisher: LedCommandPublisher,
    telemetry_publisher: TelemetryPublisher,
    peri: Peri<'static, ImuSpi>,
    sck: Peri<'static, impl SckPin<ImuSpi>>,
    mosi: Peri<'static, impl MosiPin<ImuSpi>>,
    miso: Peri<'static, impl MisoPin<ImuSpi>>,
    txdma: Peri<'static, ImuSpiTxDma>,
    rxdma: Peri<'static, ImuSpiRxDma>,
    bmi323_nss: Peri<'static, ImuSpiNss0Pin>,
    _ext_nss1_pin: Peri<'static, ExtImuSpiNss1Pin>,
    _ext_nss2_pin: Peri<'static, ExtImuSpiNss2Pin>,
    accel_int_pin: Peri<'static, ImuSpiInt1Pin>,
    gyro_int_pin: Peri<'static, ImuSpiInt2Pin>,
    accel_int: Peri<'static, <ImuSpiInt1Pin as embassy_stm32::gpio::ExtiPin>::ExtiChannel>,
    gyro_int: Peri<'static, <ImuSpiInt2Pin as embassy_stm32::gpio::ExtiPin>::ExtiChannel>,
    _ext_imu_det_pin: Peri<'static, ExtImuNDetPin>,
    flash: Peri<'static, FLASH>,
) {
    defmt::debug!("starting imu task...");

    // let imu_buf = IMU_BUFFER_CELL.take();
    // let imu_buf: &'static mut [u8; 14] = unsafe { & mut IMU_BUFFER_CELL };
    let imu_buf: &mut [u8; bmi323::SPI_MIN_BUF_LEN] = unsafe { &mut (*(&raw mut IMU_BUFFER_CELL)) };

    let imu = Bmi323::new_from_pins(
        peri,
        sck,
        mosi,
        miso,
        txdma,
        rxdma,
        crate::SystemIrqs,
        bmi323_nss.into(),
        imu_buf,
    );

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::None, crate::SystemIrqs);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::None, crate::SystemIrqs);

    // Blocking flash access for persistent on-chip IMU calibration storage.
    let flash = Flash::new_blocking(flash);

    imu_task_spawner.spawn(defmt::unwrap!(imu_task_entry(
        robot_state,
        gyro_data_publisher,
        accel_data_publisher,
        led_cmd_publisher,
        telemetry_publisher,
        imu,
        accel_int,
        gyro_int,
        flash,
    )));
}

pub fn start_imu_task_via_ie(
    imu_task_spawner: &SendSpawner,
    robot_state: &'static SharedRobotState,
    gyro_data_publisher: GyroDataPublisher,
    accel_data_publisher: AccelDataPublisher,
    led_cmd_publisher: LedCommandPublisher,
    telemetry_publisher: TelemetryPublisher,
    peri: Peri<'static, ImuSpi>,
    sck: Peri<'static, impl SckPin<ImuSpi>>,
    mosi: Peri<'static, impl MosiPin<ImuSpi>>,
    miso: Peri<'static, impl MisoPin<ImuSpi>>,
    txdma: Peri<'static, ImuSpiTxDma>,
    rxdma: Peri<'static, ImuSpiRxDma>,
    bmi323_nss: Peri<'static, ImuSpiNss0Pin>,
    _ext_nss1_pin: Peri<'static, ExtImuSpiNss1Pin>,
    _ext_nss2_pin: Peri<'static, ExtImuSpiNss2Pin>,
    accel_int_pin: Peri<'static, ImuSpiInt1Pin>,
    gyro_int_pin: Peri<'static, ImuSpiInt2Pin>,
    accel_int: Peri<'static, <ImuSpiInt1Pin as embassy_stm32::gpio::ExtiPin>::ExtiChannel>,
    gyro_int: Peri<'static, <ImuSpiInt2Pin as embassy_stm32::gpio::ExtiPin>::ExtiChannel>,
    _ext_imu_det_pin: Peri<'static, ExtImuNDetPin>,
    flash: Peri<'static, FLASH>,
) {
    defmt::debug!("starting imu task...");

    // let imu_buf = IMU_BUFFER_CELL.take();
    // let imu_buf: &'static mut [u8; 14] = unsafe { & mut IMU_BUFFER_CELL };
    let imu_buf: &mut [u8; bmi323::SPI_MIN_BUF_LEN] = unsafe { &mut (*(&raw mut IMU_BUFFER_CELL)) };

    let imu = Bmi323::new_from_pins(
        peri,
        sck,
        mosi,
        miso,
        txdma,
        rxdma,
        crate::SystemIrqs,
        bmi323_nss.into(),
        imu_buf,
    );

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD. Select software Pull::Up and
    // imu open drain
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::None, crate::SystemIrqs);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::None, crate::SystemIrqs);

    // Blocking flash access for persistent on-chip IMU calibration storage.
    let flash = Flash::new_blocking(flash);

    imu_task_spawner.spawn(defmt::unwrap!(imu_task_entry(
        robot_state,
        gyro_data_publisher,
        accel_data_publisher,
        led_cmd_publisher,
        telemetry_publisher,
        imu,
        accel_int,
        gyro_int,
        flash,
    )));
}
