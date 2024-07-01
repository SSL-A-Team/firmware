#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt;

use defmt_rtt as _; 

use ateam_control_board::{create_io_task, get_system_config, pins::BatteryVoltPubSub, robot_state::SharedRobotState};


use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static BATTERY_VOLT_CHANNEL: BatteryVoltPubSub = PubSubChannel::new();

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

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////
    
    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();

    let mut battery_volt_subscriber = BATTERY_VOLT_CHANNEL.subscriber().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner, robot_state, battery_volt_publisher, p);

    loop {
        defmt::info!("Battery Voltage: {}", battery_volt_subscriber.next_message_pure().await);
        Timer::after_millis(1000).await;
    }
}