#![no_std]
#![no_main]

use ateam_common_packets::{bindings::KickRequest, bindings::BasicControl, radio::DataPacket};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;

use ateam_control_board::{create_imu_task, create_io_task, create_shutdown_task, get_system_config, pins::{AccelDataPubSub, BatteryVoltPubSub, CommandsPubSub, GyroDataPubSub, TelemetryPubSub}, robot_state::SharedRobotState, tasks::control_task::start_control_task};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static GYRO_DATA_CHANNEL: GyroDataPubSub = PubSubChannel::new();
static ACCEL_DATA_CHANNEL: AccelDataPubSub = PubSubChannel::new();
static BATTERY_VOLT_CHANNEL: BatteryVoltPubSub = PubSubChannel::new();

static RADIO_UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[interrupt]
unsafe fn CORDIC() {
    RADIO_UART_QUEUE_EXECUTOR.on_interrupt();
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

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P7);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let test_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();

    // Battery Channel
    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();
    let battery_volt_subscriber = BATTERY_VOLT_CHANNEL.subscriber().unwrap();

    // TODO imu channel
    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let control_gyro_data_subscriber = GYRO_DATA_CHANNEL.subscriber().unwrap();

    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();
    let control_accel_data_subscriber = ACCEL_DATA_CHANNEL.subscriber().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner,
        robot_state,
        battery_volt_publisher,
        p);

    create_shutdown_task!(main_spawner,
        robot_state,
        p);

    create_imu_task!(main_spawner,
        robot_state,
        imu_gyro_data_publisher, imu_accel_data_publisher,
        p);

    start_control_task(
        uart_queue_spawner, main_spawner,
        robot_state,
        control_command_subscriber, control_telemetry_publisher, battery_volt_subscriber, control_gyro_data_subscriber, control_accel_data_subscriber,
        p.UART4, p.PA1, p.PA0, p.DMA1_CH3, p.DMA1_CH2, p.PC1, p.PC0,
        p.UART7, p.PF6, p.PF7, p.DMA1_CH5, p.DMA1_CH4, p.PF8, p.PF9,
        p.UART8, p.PE0, p.PE1, p.DMA1_CH7, p.DMA1_CH6, p.PB9, p.PB8,
        p.USART1, p.PB15, p.PB14, p.DMA1_CH1, p.DMA1_CH0, p.PD8, p.PD9,
        p.UART5, p.PB12, p.PB13, p.DMA2_CH3, p.DMA2_CH2, p.PD13, p.PD12).await;


    loop {
        Timer::after_millis(100).await;

        test_command_publisher.publish(DataPacket::BasicControl(BasicControl {
            vel_x_linear:0.2,
            vel_y_linear:0.0,
            vel_z_angular:0.0,
            kick_vel:0.0,
            dribbler_speed:10.0,
            kick_request:KickRequest::KR_DISABLE,
            _bitfield_align_1: Default::default(),
            _bitfield_1: Default::default() 
        })).await;
    }
}