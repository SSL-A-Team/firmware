#![no_std]
#![no_main]

use ateam_common_packets::{bindings::{BasicTelemetry, ParameterCommandCode}, radio::{DataPacket, TelemetryPacket}};
use embassy_executor::InterruptExecutor;
use embassy_futures::select::{self, Either};
use embassy_stm32::{
    interrupt, pac::Interrupt
};
use embassy_sync::pubsub::{PubSubChannel, WaitResult};

use defmt_rtt as _;

use ateam_control_board::{create_io_task, create_radio_task, get_system_config, pins::{BatteryVoltPubSub, CommandsPubSub, TelemetryPubSub}, robot_state::SharedRobotState};


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

    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();


    ///////////////////
    //  start tasks  //
    ///////////////////

    create_radio_task!(main_spawner, uart_queue_spawner, uart_queue_spawner,
        robot_state,
        radio_command_publisher, radio_telemetry_subscriber,
        wifi_credentials,
        p);

    create_io_task!(main_spawner, robot_state, battery_volt_publisher, p);

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
                // defmt::warn!("packet processing timed out.");
            }
        }
    }
}