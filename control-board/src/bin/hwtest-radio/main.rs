#![no_std]
#![no_main]

use ateam_common_packets::{bindings_radio::{BasicTelemetry, ParameterCommandCode}, radio::{DataPacket, TelemetryPacket}};
use embassy_executor::InterruptExecutor;
use embassy_futures::select::{self, Either};
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::{PubSubChannel, WaitResult};

use defmt_rtt as _; 

use ateam_control_board::{get_system_config, pins::{CommandsPubSub, TelemetryPubSub}, robot_state::RobotState, tasks::{radio_task::start_radio_task, user_io_task::start_io_task}};


// load credentials from correct crate
#[cfg(not(feature = "no-private-credentials"))]
use credentials::private_credentials::wifi::wifi_credentials;
#[cfg(feature = "no-private-credentials")]
use credentials::public_credentials::wifi::wifi_credentials;

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<RobotState> = ConstStaticCell::new(RobotState::new());

static RADIO_C2_CHANNEL: CommandsPubSub = PubSubChannel::new();
static RADIO_TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();

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
    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P5);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let radio_command_publisher = RADIO_C2_CHANNEL.publisher().unwrap();
    let mut control_command_subscriber = RADIO_C2_CHANNEL.subscriber().unwrap();

    // telemetry channel
    let control_telemetry_publisher = RADIO_TELEMETRY_CHANNEL.publisher().unwrap();
    let radio_telemetry_subscriber = RADIO_TELEMETRY_CHANNEL.subscriber().unwrap();

    ///////////////////
    //  start tasks  //
    ///////////////////

    let wifi_network = wifi_credentials[1];
    defmt::info!("connecting with {}, {}", wifi_network.get_ssid(), wifi_network.get_password());
    start_radio_task(
        main_spawner, uart_queue_spawner,
        robot_state,
        radio_command_publisher, radio_telemetry_subscriber,
        wifi_network,
        p.USART10, p.PE2, p.PE3, p.PG13, p.PG14,
        p.DMA2_CH1, p.DMA2_CH0,
        p.PC13, p.PE4).await;

    start_io_task(main_spawner,
        robot_state,
        p.PD6, p.PD5, p.EXTI6, p.EXTI5,
        p.PG7, p.PG6, p.PG5, p.PG4, p.PG3, p.PG2, p.PD15,
        p.PG12, p.PG11, p.PG10, p.PG9,
        p.PF3, p.PF2, p.PF1, p.PF0,
        p.PD0, p.PD1, p.PD3, p.PD4, p.PD14);

    loop {
        Timer::after_millis(1000).await;
    }

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

                                let basic_telem = BasicTelemetry {
                                    sequence_number: 0,
                                    robot_revision_major: 0,
                                    robot_revision_minor: 0,
                                    battery_level: 0.0,
                                    battery_temperature: 0.0,
                                    _bitfield_align_1: [0; 0],
                                    _bitfield_1: BasicTelemetry::new_bitfield_1(
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    ),
                                    motor_0_temperature: 0.0,
                                    motor_1_temperature: 0.0,
                                    motor_2_temperature: 0.0,
                                    motor_3_temperature: 0.0,
                                    motor_4_temperature: 0.0,
                                    kicker_charge_level: 0.0
                                };

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
                defmt::warn!("packet processing timed out.");
            }
        }
    }
}