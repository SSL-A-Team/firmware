#![no_std]
#![no_main]

use ateam_common_packets::{bindings::BasicControl, bindings::KickRequest, radio::DataPacket};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{gpio::{Input, Pull}, interrupt, pac::Interrupt};
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

    // let pg10_read = Input::new(p.PG10, Pull::Up);
    // let pg11_read = Input::new(p.PG11, Pull::Up);
    // let pd0_read = Input::new(p.PD0, Pull::Up);
    // let pd1_read = Input::new(p.PD1, Pull::Up);

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
        Timer::after_millis(10).await;

        if robot_state.hw_init_state_valid() {
            break;
        }

        defmt::info!("waiting for hw init state to be valid");
    }

    loop {
        Timer::after_millis(10).await;

        if robot_state.get_hw_robot_id() == 0 {
            break;
        }

        defmt::info!("wait for user to select robot id 0");
    }

    loop {
        Timer::after_millis(10).await;

        let mut motor_speed_ind = robot_state.get_hw_robot_id();
        if motor_speed_ind > 8 {
            motor_speed_ind = 16 - motor_speed_ind;
        }

        let drib_speed = motor_speed_ind as f32 * 75.0;

        test_command_publisher
            .publish(DataPacket::BasicControl(BasicControl {
                _bitfield_1: Default::default(),
                _bitfield_align_1: Default::default(),
                vel_x_linear: 0.0,
                vel_y_linear: 0.0,
                vel_z_angular: 0.0,
                kick_vel: 0.0,
                dribbler_speed: drib_speed,
                kick_request: KickRequest::KR_DISABLE,
            }))
            .await;
    }
}
