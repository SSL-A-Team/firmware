#![no_std]
#![no_main]

use ateam_common_packets::{bindings::BasicControl, bindings::KickRequest, radio::DataPacket};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{interrupt, pac::Interrupt};
use embassy_sync::pubsub::PubSubChannel;

use defmt_rtt as _;

use ateam_control_board::{
    create_io_task, create_kicker_task, get_system_config,
    pins::{CommandsPubSub, KickerTelemetryPubSub},
    robot_state::SharedRobotState,
};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static KICKER_DATA_CHANNEL: KickerTelemetryPubSub = PubSubChannel::new();

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

    let kicker_telemetry_publisher = KICKER_DATA_CHANNEL.publisher().unwrap();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(
        embassy_stm32::interrupt::CEC,
        embassy_stm32::interrupt::Priority::P7,
    );
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let kicker_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let test_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    create_io_task!(main_spawner, robot_state, p);

    create_kicker_task!(
        main_spawner,
        uart_queue_spawner,
        robot_state,
        kicker_command_subscriber,
        kicker_telemetry_publisher,
        p
    );

    loop {
        Timer::after_millis(100).await;

        test_command_publisher
            .publish(DataPacket::BasicControl(BasicControl {
                _bitfield_1: Default::default(),
                _bitfield_align_1: Default::default(),
                last_vision_update_us_hi: 0,
                last_vision_update_us_lo: 0,
                vision_x: 0.0,
                vision_y: 0.0,
                vision_z: 0.0,
                target_x: 0.0,
                target_y: 0.0,
                target_z: 0.0,
                vel_x_linear: 0.0,
                vel_y_linear: 0.0,
                vel_z_angular: 0.0,
                kick_vel: 0.0,
                dribbler_speed: 50.0,
                kick_request: KickRequest::KR_DISABLE,
            }))
            .await;
    }
}
