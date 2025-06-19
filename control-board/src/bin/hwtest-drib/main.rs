#![no_std]
#![no_main]

use ateam_common_packets::{bindings::KickRequest, bindings::BasicControl, radio::DataPacket};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;

use ateam_control_board::{create_io_task, create_kicker_task, get_system_config, pins::{BatteryVoltPubSub, CommandsPubSub}, robot_state::SharedRobotState};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
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
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let test_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();

    // Battery Channel
    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner,
        robot_state,
        battery_volt_publisher,
        p);

    create_kicker_task!(
        main_spawner, uart_queue_spawner,
        robot_state,
        kicker_command_subscriber,
        p);


    loop {
        Timer::after_millis(100).await;

        test_command_publisher.publish(DataPacket::BasicControl(BasicControl {
            vel_x_linear: 0.0,
            vel_y_linear: 0.0,
            vel_z_angular: 0.0,
            kick_vel: 0.0,
            dribbler_speed: 50.0,
            kick_request: KickRequest::KR_DISABLE,
        })).await;
    }
}