#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt;

use defmt_rtt as _; 

use ateam_control_board::{create_dotstar_task, create_io_task, get_system_config, pins::LedCommandPubSub, robot_state::SharedRobotState, tasks::dotstar_task::{ControlBoardLedCommand, ControlGeneralLedCommand}};


use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static LED_COMMAND_CHANNEL: LedCommandPubSub = PubSubChannel::new();

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
    
    let led_command_subscriber = LED_COMMAND_CHANNEL.subscriber().unwrap();
    let led_command_publisher = LED_COMMAND_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner, robot_state, p);

    create_dotstar_task!(main_spawner, led_command_subscriber, p);

    // create_audio_task!(main_spawner, robot_state, p);

    led_command_publisher.publish(ControlBoardLedCommand::General(ControlGeneralLedCommand::ShutdownRequested)).await;


    loop {
        Timer::after_millis(1000).await;
    }
}