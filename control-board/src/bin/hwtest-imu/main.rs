#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::InterruptExecutor;
use embassy_futures::select::{self, Either3};
use embassy_stm32::interrupt;
use embassy_sync::pubsub::{PubSubChannel, WaitResult};

use defmt_rtt as _;

use ateam_control_board::{
    create_dotstar_task, create_imu_task, create_io_task, get_system_config,
    pins::{AccelDataPubSub, GyroDataPubSub, LedCommandPubSub, TelemetryPubSub},
    robot_state::SharedRobotState,
};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();
static LED_COMMAND_PUBSUB: LedCommandPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
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

    let robot_state = ROBOT_STATE.take();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();

    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let mut imu_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let mut imu_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();
    let imu_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();
    let imu_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner, robot_state, p);

    create_dotstar_task!(main_spawner, led_command_subscriber, p);

    // create_audio_task!(main_spawner, robot_state, p);

    create_imu_task!(
        main_spawner,
        robot_state,
        imu_gyro_data_publisher,
        imu_accel_data_publisher,
        imu_led_cmd_publisher,
        imu_telemetry_publisher,
        p
    );

    defmt::info!("=====================================================");
    defmt::info!("IMU hardware test / on-chip calibration validation");
    defmt::info!("Keep the robot UPRIGHT and STATIONARY on a level");
    defmt::info!("surface. The IMU task calibrates on-chip (gyro");
    defmt::info!("self-cal + accel X/Y offset) and persists to flash;");
    defmt::info!("watch the earlier log for 'restored from flash' vs");
    defmt::info!("'calibrated on-chip and stored'. This loop then");
    defmt::info!("verifies the published data is bias-corrected.");
    defmt::info!("=====================================================");

    validate_imu(
        &mut imu_gyro_data_subscriber,
        &mut imu_accel_data_subscriber,
    )
    .await;
}

/// Number of gyro (and, separately, accel) samples averaged per validation report.
/// ~0.5 s at the 1600 Hz IMU ODR.
const VALIDATION_WINDOW_SAMPLES: u32 = 800;

/// Stationary gyro bias must average below this magnitude on every axis (rad/s).
const GYRO_STATIONARY_THRESH_RADS: f32 = 0.03;
/// Upright accel X/Y bias must average below this magnitude (m/s^2).
const ACCEL_XY_THRESH_MPS2: f32 = 0.7;
/// Upright accel Z must average within this band (m/s^2) around gravity (~9.81).
const ACCEL_Z_MIN_MPS2: f32 = 8.0;
const ACCEL_Z_MAX_MPS2: f32 = 11.5;

/// Consumes published gyro/accel data and periodically reports, for a stationary
/// upright robot, whether the on-chip bias correction is producing near-zero gyro
/// and near-zero accel X/Y with accel Z near gravity. Prints PASS/FAIL each window.
async fn validate_imu(
    gyro_sub: &mut ateam_control_board::pins::GyroDataSubscriber,
    accel_sub: &mut ateam_control_board::pins::AccelDataSubscriber,
) {
    let mut gyro_stats = AxisStats::new();
    let mut accel_stats = AxisStats::new();

    loop {
        match select::select3(
            gyro_sub.next_message(),
            accel_sub.next_message(),
            Timer::after_millis(2000),
        )
        .await
        {
            Either3::First(gyro_data) => match gyro_data {
                WaitResult::Lagged(amnt) => {
                    defmt::warn!("gyro data lagged by {}", amnt);
                }
                WaitResult::Message(msg) => {
                    gyro_stats.add(msg[0], msg[1], msg[2]);
                    if gyro_stats.count >= VALIDATION_WINDOW_SAMPLES {
                        let (mx, my, mz) = gyro_stats.means();
                        let pass = mx.abs() < GYRO_STATIONARY_THRESH_RADS
                            && my.abs() < GYRO_STATIONARY_THRESH_RADS
                            && mz.abs() < GYRO_STATIONARY_THRESH_RADS;
                        defmt::info!(
                            "GYRO   mean=[{}, {}, {}] rad/s  peak=[{}, {}, {}]  -> {}",
                            mx,
                            my,
                            mz,
                            gyro_stats.peak_x,
                            gyro_stats.peak_y,
                            gyro_stats.peak_z,
                            if pass { "PASS (near zero)" } else { "FAIL (bias/motion?)" }
                        );
                        gyro_stats.reset();
                    }
                }
            },
            Either3::Second(accel_data) => match accel_data {
                WaitResult::Lagged(amnt) => {
                    defmt::warn!("accel data lagged by {}", amnt);
                }
                WaitResult::Message(msg) => {
                    accel_stats.add(msg[0], msg[1], msg[2]);
                    if accel_stats.count >= VALIDATION_WINDOW_SAMPLES {
                        let (mx, my, mz) = accel_stats.means();
                        let pass = mx.abs() < ACCEL_XY_THRESH_MPS2
                            && my.abs() < ACCEL_XY_THRESH_MPS2
                            && mz > ACCEL_Z_MIN_MPS2
                            && mz < ACCEL_Z_MAX_MPS2;
                        defmt::info!(
                            "ACCEL  mean=[{}, {}, {}] m/s^2 (X/Y~0, Z~9.81)  -> {}",
                            mx,
                            my,
                            mz,
                            if pass {
                                "PASS"
                            } else {
                                "FAIL (bias/tilt/motion?)"
                            }
                        );
                        accel_stats.reset();
                    }
                }
            },
            Either3::Third(_) => {
                defmt::warn!("no IMU data received for 2s (still calibrating or IMU inop?)");
            }
        }
    }
}

/// Running per-axis mean and peak-magnitude accumulator over a validation window.
struct AxisStats {
    sum_x: f32,
    sum_y: f32,
    sum_z: f32,
    peak_x: f32,
    peak_y: f32,
    peak_z: f32,
    count: u32,
}

impl AxisStats {
    fn new() -> Self {
        Self {
            sum_x: 0.0,
            sum_y: 0.0,
            sum_z: 0.0,
            peak_x: 0.0,
            peak_y: 0.0,
            peak_z: 0.0,
            count: 0,
        }
    }

    fn add(&mut self, x: f32, y: f32, z: f32) {
        self.sum_x += x;
        self.sum_y += y;
        self.sum_z += z;
        self.peak_x = self.peak_x.max(x.abs());
        self.peak_y = self.peak_y.max(y.abs());
        self.peak_z = self.peak_z.max(z.abs());
        self.count += 1;
    }

    fn means(&self) -> (f32, f32, f32) {
        let n = self.count.max(1) as f32;
        (self.sum_x / n, self.sum_y / n, self.sum_z / n)
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}
