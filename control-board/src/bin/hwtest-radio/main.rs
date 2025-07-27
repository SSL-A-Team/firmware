#![no_std]
#![no_main]

use ateam_common_packets::{bindings::ParameterCommandCode, radio::{DataPacket, TelemetryPacket}};
use embassy_executor::InterruptExecutor;
use embassy_futures::select::{self, Either};
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::{PubSubChannel, WaitResult};

use defmt_rtt as _;

use ateam_control_board::{create_dotstar_task, create_io_task, create_radio_task, get_system_config, pins::{CommandsPubSub, LedCommandPubSub, TelemetryPubSub}, robot_state::SharedRobotState};


// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
// provide embedded panic probe
// use panic_probe as _;
use static_cell::ConstStaticCell;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    // Delay to give it a change to print
    cortex_m::asm::delay(100_000_000);
    cortex_m::peripheral::SCB::sys_reset();
}

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();
static LED_COMMAND_PUBSUB: LedCommandPubSub = PubSubChannel::new();

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

    defmt::info!("robot state taken");

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    // uart queue executor should be highest priority
    // NOTE: maybe this should be all DMA tasks? No computation tasks here
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P5);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let mut control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();
    let radio_led_cmd_publisher = LED_COMMAND_PUBSUB.publisher().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

    let led_command_subscriber = LED_COMMAND_PUBSUB.subscriber().unwrap();


    ///////////////////
    //  start tasks  //
    ///////////////////

    defmt::info!("starting tasks");

    create_io_task!(main_spawner,
        robot_state,
        p);

    create_dotstar_task!(main_spawner,
        led_command_subscriber,
        p);

    create_radio_task!(main_spawner, uart_queue_spawner, uart_queue_spawner,
        robot_state,
        radio_command_publisher, radio_telemetry_subscriber, radio_led_cmd_publisher,
        wifi_credentials,
        p);

    defmt::info!("radio task started");

    loop {
        match select::select(control_command_subscriber.next_message(), Timer::after_millis(1000)).await {
            Either::First(gyro_data) => {
                match gyro_data {
                    WaitResult::Lagged(amnt) => {
                        defmt::warn!("receiving control packets lagged by {}", amnt);
                    }
                    WaitResult::Message(msg) => {
                        match msg {
                            DataPacket::BasicControl(_bc) => {
                                defmt::info!("got command packet");

                                let basic_telem = Default::default();

                                let pkt_wrapped = TelemetryPacket::Basic(basic_telem);
                                control_telemetry_publisher.publish(pkt_wrapped).await;

                                defmt::info!("send basic telem resp");
                            }
                            DataPacket::ParameterCommand(pc) => {
                                defmt::info!("got parameter packet");

                                let mut param_resp = pc;
                                param_resp.command_code = ParameterCommandCode::PCC_ACK;

                                let wrapped_pkt = TelemetryPacket::ParameterCommandResponse(param_resp);
                                control_telemetry_publisher.publish(wrapped_pkt).await;

                                defmt::info!("sent param resp packet");
                            }
                        }
                    }
                }
            }
            Either::Second(_) => {
                // defmt::warn!("packet processing timed out.");
            }
        }
    }
}