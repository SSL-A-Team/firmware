#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt, pac::Interrupt, time::{hz, khz}, timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _; 

use ateam_control_board::{create_imu_task, create_io_task, create_radio_task, get_system_config, pins::{AccelDataPubSub, CommandsPubSub, GyroDataPubSub, TelemetryPubSub}, robot_state::SharedRobotState, tasks::control_task::start_control_task};


// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
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

    let robot_state = ROBOT_STATE.take();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P6);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

    // TODO imu channel
    let imu_gyro_data_publisher = GYRO_DATA_CHANNEL.publisher().unwrap();
    let imu_accel_data_publisher = ACCEL_DATA_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    // let ch2 = PwmPin::new_ch2(p.PE6, OutputType::PushPull);
    // let mut pwm = SimplePwm::new(p.TIM15, None, Some(ch2), None, None, khz(2), Default::default());
    // let max = pwm.get_max_duty();
    // pwm.enable(Channel::Ch2);


    // pwm.set_duty(Channel::Ch2, max / 2);

    // loop {
    //     pwm.set_frequency(hz(500));
    //     Timer::after_millis(1000).await;
    //     pwm.set_frequency(hz(2000));
    //     Timer::after_millis(1000).await;
    // }

    create_radio_task!(main_spawner, uart_queue_spawner,
        robot_state,
        radio_command_publisher, radio_telemetry_subscriber,
        wifi_credentials,
        p);
            
    create_io_task!(main_spawner,
        robot_state,
        p);

    create_imu_task!(main_spawner,
        imu_gyro_data_publisher, imu_accel_data_publisher,
        p);

    start_control_task(
        uart_queue_spawner, main_spawner, 
        robot_state, 
        control_command_subscriber, control_telemetry_publisher, 
        p.UART4, p.PA1, p.PA0, p.DMA1_CH3, p.DMA1_CH2, p.PC1, p.PC0,
        p.UART7, p.PF6, p.PF7, p.DMA1_CH5, p.DMA1_CH4, p.PF8, p.PF9,
        p.UART8, p.PE0, p.PE1, p.DMA1_CH7, p.DMA1_CH6, p.PB9, p.PB8,
        p.USART1, p.PB15, p.PB14, p.DMA1_CH1, p.DMA1_CH0, p.PD8, p.PD9,
        p.UART5, p.PB12, p.PB13, p.DMA2_CH3, p.DMA2_CH2, p.PD13, p.PD12).await;

    loop {
        Timer::after_millis(10).await;
    }
}