use embassy_executor::Spawner;
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
use crate::imu_calibration::{erase_calibration, load_calibration_to_chip, run_calibration};
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

/// Accel Z below this magnitude (m/s^2) is treated as tipped / not upright.
const ACCEL_TIPPED_Z_MPS2: f32 = 4.0;

/// Settle delay after the calibration LED turns magenta before sampling starts, so the
/// robot (and the operator's hand) can come to rest.
const CALIBRATION_SETTLE_MS: u64 = 1000;

/// While the IMU is inoperational (uncalibrated), re-publish an error telemetry at this
/// interval so the software stack is informed over the radio.
const INOP_ERROR_TELEM_INTERVAL_MS: u64 = 1000;

/// Action selected at boot (by held buttons in `main`) for the IMU maintenance path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ImuBootAction {
    /// Normal boot: load the stored calibration (or go inop). No maintenance.
    Normal,
    /// Run a fresh calibration and store it, then halt (reboot to run normally).
    Calibrate,
    /// Erase the stored calibration, then halt.
    EraseCalibration,
}

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

#[link_section = ".axisram.buffers"]
static mut IMU_BUFFER_CELL: [u8; bmi323::SPI_MIN_BUF_LEN] = [0; bmi323::SPI_MIN_BUF_LEN];

/// Constructs the BMI323 driver from its SPI peripheral + pins. Shared by the normal imu
/// task ([`start_imu_task`]) and the boot maintenance path (`main`).
#[allow(clippy::too_many_arguments)]
pub fn build_imu(
    peri: Peri<'static, ImuSpi>,
    sck: Peri<'static, impl SckPin<ImuSpi>>,
    mosi: Peri<'static, impl MosiPin<ImuSpi>>,
    miso: Peri<'static, impl MisoPin<ImuSpi>>,
    txdma: Peri<'static, ImuSpiTxDma>,
    rxdma: Peri<'static, ImuSpiRxDma>,
    bmi323_nss: Peri<'static, ImuSpiNss0Pin>,
) -> Bmi323<'static, 'static> {
    let imu_buf: &mut [u8; bmi323::SPI_MIN_BUF_LEN] = unsafe { &mut (*(&raw mut IMU_BUFFER_CELL)) };
    Bmi323::new_from_pins(
        peri,
        sck,
        mosi,
        miso,
        txdma,
        rxdma,
        crate::SystemIrqs,
        bmi323_nss.into(),
        imu_buf,
    )
}

/// Runs the full BMI323 configuration sequence: soft reset, self test, feature engine,
/// 180-degree axis remap, gyro/accel config, and interrupt config. Returns `Err` on any
/// failure (details logged via defmt). Shared by the imu task's (re)configuration loop
/// and the boot maintenance path.
///
/// The IMU is mounted rotated 180 degrees about the board Z axis, corrected on-chip via
/// the feature engine (X -> -X, Y -> -Y, Z -> Z). This must be done while the sensors are
/// inactive (before configuring the accel/gyro) and re-applied on every (re)config, since
/// the mapping is cleared by the soft reset in `imu.init()`.
pub async fn configure_imu(imu: &mut Bmi323<'static, 'static>) -> Result<(), ()> {
    imu.init().await;

    if imu.self_test().await.is_err() {
        defmt::error!("IMU self test failed");
        return Err(());
    }

    if imu.enable_feature_engine().await.is_err() {
        defmt::error!("IMU feature engine enable failed");
        return Err(());
    }
    if imu
        .set_axis_remap(AxisMap::XyzToXyz, true, true, false)
        .await
        .is_err()
    {
        defmt::error!("IMU axis remap failed");
        return Err(());
    }

    // Configure the gyro and map its data-ready interrupt to INT2.
    if imu
        .set_gyro_config(
            GyroMode::ContinuousHighPerformance,
            GyroRange::PlusMinus2000DegPerSec,
            Bandwidth3DbCutoffFreq::AccOdrOver4,
            IMU_ODR,
            DataAveragingWindow::NoFiltering,
        )
        .await
        .is_err()
    {
        defmt::error!("IMU gyro configuration failed");
        return Err(());
    }
    imu.set_gyro_interrupt_mode(InterruptMode::MappedToInt2).await;

    // Configure the accel and map its data-ready interrupt to INT1.
    if imu
        .set_accel_config(
            AccelMode::ContinuousHighPerformance,
            AccelRange::Range4g,
            Bandwidth3DbCutoffFreq::AccOdrOver4,
            IMU_ODR,
            DataAveragingWindow::NoFiltering,
        )
        .await
        .is_err()
    {
        defmt::error!("IMU accel configuration failed");
        return Err(());
    }
    imu.set_accel_interrupt_mode(InterruptMode::MappedToInt1).await;

    imu.set_int1_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull)
        .await;
    imu.set_int2_pin_config(IntPinLevel::ActiveLow, IntPinDriveMode::PushPull)
        .await;
    imu.set_int2_enabled(true).await;

    Ok(())
}

/// Boot maintenance routine. Runs before the heavy tasks are spawned, so the blocking
/// full-sector flash erase/program happens while the system is quiet (it cannot be
/// aborted by the 1 kHz control loop / motor / radio interrupts). Configures the IMU,
/// then either runs a fresh calibration (magenta LED, settle delay, gyro self-cal +
/// accel bias estimate, store) or erases the stored calibration, driving `led_command_pub`
/// for feedback. The caller should halt (await forever) afterwards; a reboot then runs
/// normally. Does nothing for [`ImuBootAction::Normal`].
pub async fn run_boot_maintenance(
    imu: &mut Bmi323<'static, 'static>,
    flash: &mut Flash<'static, Blocking>,
    led_command_pub: &LedCommandPublisher,
    action: ImuBootAction,
) {
    led_command_pub
        .publish(ControlBoardLedCommand::Imu(
            ImuStatusLedCommand::Configuring,
        ))
        .await;
    if configure_imu(imu).await.is_err() {
        led_command_pub
            .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
            .await;
        return;
    }

    match action {
        ImuBootAction::Calibrate => {
            defmt::info!("boot maintenance: calibrating IMU (hold robot upright & still)");
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(
                    ImuStatusLedCommand::Calibrating,
                ))
                .await;
            Timer::after_millis(CALIBRATION_SETTLE_MS).await;
            match run_calibration(imu, flash).await {
                Ok(_) => {
                    led_command_pub
                        .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Ok))
                        .await;
                }
                Err(_) => {
                    defmt::error!("boot IMU calibration failed");
                    led_command_pub
                        .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                        .await;
                }
            }
        }
        ImuBootAction::EraseCalibration => {
            defmt::info!("boot maintenance: erasing stored IMU calibration");
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(
                    ImuStatusLedCommand::Calibrating,
                ))
                .await;
            if erase_calibration(flash).is_ok() {
                defmt::info!("stored IMU calibration erased");
            }
            // Erased -> uncalibrated; show the inop/error color.
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
        }
        ImuBootAction::Normal => {}
    }
}

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

        // At the beginning, assume the IMU is not working yet.
        robot_state.set_imu_inop(true);

        if configure_imu(&mut imu).await.is_err() {
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU configuration failed"),
            ));
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            Timer::after_millis(1000).await;
            continue 'imu_configuration_loop;
        }

        // Apply a stored calibration from flash (the DP offset registers are volatile and
        // cleared by the soft reset in configure_imu). No calibration is run here: if none
        // is stored the IMU stays inoperational. Calibration is triggered from the boot
        // maintenance path (hold enter on boot), which runs before the heavy tasks.
        let calibrated = load_calibration_to_chip(&mut imu, &mut flash).await;
        if calibrated {
            robot_state.set_imu_inop(false);
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Ok))
                .await;
        } else {
            robot_state.set_imu_inop(true);
            led_command_pub
                .publish(ControlBoardLedCommand::Imu(ImuStatusLedCommand::Error))
                .await;
            defmt::warn!(
                "no valid stored IMU calibration; IMU inoperational (hold enter on boot to calibrate)"
            );
            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                create_error_telemetry_from_string("IMU inoperational: not calibrated"),
            ));
        }

        // Clear any stale/transient filter state before starting.
        accel_x_filter.reset();
        accel_y_filter.reset();

        let mut last_inop_telem = Instant::now();

        'imu_data_loop: loop {
            // block on gyro interrupt, active low
            match select(gyro_int.wait_for_falling_edge(), Timer::after_millis(1000)).await {
                Either::First(_) => {
                    // read gyro and accel data (already bias-corrected on-chip via the
                    // data-path offset registers when a calibration is applied).
                    let imu_data = imu.gyro_get_data_rads().await;
                    let accel_data = imu.accel_get_data_mps().await;

                    // While uncalibrated the IMU stays inop and does not publish (its bias
                    // is unknown); keep servicing the hardware and re-report periodically so
                    // the software stack learns the IMU is inoperational over the radio.
                    if !calibrated {
                        let now = Instant::now();
                        if now.duration_since(last_inop_telem).as_millis()
                            >= INOP_ERROR_TELEM_INTERVAL_MS
                        {
                            last_inop_telem = now;
                            telemetry_pub.publish_immediate(TelemetryPacket::ErrorTelemetry(
                                create_error_telemetry_from_string(
                                    "IMU inoperational: not calibrated",
                                ),
                            ));
                        }
                        continue;
                    }

                    // Got an interrupt with a valid calibration, so the IMU is operational.
                    robot_state.set_imu_inop(false);

                    // Publish the (chip-corrected) gyro.
                    gyro_pub.publish_immediate(Vector3::new(imu_data[0], imu_data[1], imu_data[2]));

                    // Low-pass filter the (chip-corrected) accel X/Y to reject motor/wheel
                    // vibration. Z is published unfiltered for responsive tipped detection
                    // below (and is intentionally left with no on-chip offset so it keeps
                    // measuring gravity).
                    let accel_x_unbiased = accel_data[0] as f32;
                    let accel_y_unbiased = accel_data[1] as f32;
                    accel_x_filter.add_sample(accel_x_unbiased);
                    accel_y_filter.add_sample(accel_y_unbiased);
                    let accel_x_filtered =
                        accel_x_filter.filtered_value().unwrap_or(accel_x_unbiased);
                    let accel_y_filtered =
                        accel_y_filter.filtered_value().unwrap_or(accel_y_unbiased);

                    accel_pub.publish_immediate(Vector3::new(
                        accel_x_filtered,
                        accel_y_filtered,
                        accel_data[2] as f32,
                    ));

                    if (accel_data[2] as f32) < ACCEL_TIPPED_Z_MPS2 {
                        if !first_tipped_seen {
                            // If it's the first time a tipping occured, start tracking.
                            first_tipped_seen = true;
                            first_tipped_check_time = Instant::now();
                        } else {
                            // After the first tipped is seen, wait if it has been tipped long enough.
                            let cur_time = Instant::now();
                            if Instant::duration_since(&cur_time, first_tipped_check_time)
                                .as_millis()
                                > TIPPED_MIN_DURATION_MS
                            {
                                robot_state.set_robot_tipped(true);
                            } else {
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

#[allow(clippy::too_many_arguments)]
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

    let imu = build_imu(peri, sck, mosi, miso, txdma, rxdma, bmi323_nss);

    // IMU breakout INT2 is directly connected to the MCU with no hardware PU/PD.
    let accel_int = ExtiInput::new(accel_int_pin, accel_int, Pull::None, crate::SystemIrqs);
    let gyro_int = ExtiInput::new(gyro_int_pin, gyro_int, Pull::None, crate::SystemIrqs);

    // Blocking flash access for reading the persisted on-chip IMU calibration.
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
