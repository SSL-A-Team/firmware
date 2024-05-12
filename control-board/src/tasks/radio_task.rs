
use ateam_common_packets::radio::TelemetryPacket;
use ateam_lib_stm32::make_uart_queues;
use credentials::WifiCredential;
use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    gpio::{Input, Pull},
    usart::{self, Uart}
};
use embassy_sync::pubsub::WaitResult;
use embassy_time::Timer;

use crate::{drivers::radio::RobotRadio, pins::*, SystemIrqs};

pub const RADIO_MAX_TX_PACKET_SIZE: usize = 256;
pub const RADIO_TX_BUF_DEPTH: usize = 4;
pub const RADIO_MAX_RX_PACKET_SIZE: usize = 256;
pub const RADIO_RX_BUF_DEPTH: usize = 4;

make_uart_queues!(RADIO,
    RadioUART, RadioRxDMA, RadioTxDMA,
    RADIO_MAX_RX_PACKET_SIZE, RADIO_RX_BUF_DEPTH,
    RADIO_MAX_TX_PACKET_SIZE, RADIO_TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);


#[embassy_executor::task]
async fn radio_task_entry(
    command_publisher: CommandsPublisher,
    mut telemetry_subscriber: TelemetrySubcriber,
    wifi_network: WifiCredential,
    radio_reset_pin: RadioResetPin,
    radio_ndet_pin: RadioNDetectPin) {

    defmt::info!("radio task startup");

    let radio_ndet = Input::new(radio_ndet_pin, Pull::None);

    let mut radio: RobotRadio<'static, RadioUART, RadioRxDMA, RadioTxDMA, RADIO_MAX_TX_PACKET_SIZE, RADIO_MAX_RX_PACKET_SIZE, RADIO_TX_BUF_DEPTH, RADIO_RX_BUF_DEPTH> = 
        RobotRadio::new(&RADIO_RX_UART_QUEUE, &RADIO_TX_UART_QUEUE, radio_reset_pin).await.unwrap();

    #[allow(unused_labels)]
    'connect_loop: loop {
        if radio_ndet.is_high() {
            defmt::error!("radio appears unplugged.");
            Timer::after_millis(1000).await;
            continue 'connect_loop;
        }

        let res = radio.connect_uart().await;
        if res.is_err() {
            defmt::error!("failed to establish radio UART connection.");
            Timer::after_millis(1000).await;
            continue 'connect_loop;
        }
        defmt::info!("established radio uart coms");

        while radio.connect_to_network(wifi_network).await.is_err() {
            defmt::error!("failed to connect to wifi network.");
            Timer::after_millis(1000).await;
        }
        defmt::info!("radio connected");
    
        let res = radio.open_multicast().await;
        if res.is_err() {
            defmt::error!("failed to establish multicast socket to network.");
            continue 'connect_loop;
        }
        defmt::info!("multicast open");

        // TODO add inbound timeout somewhere, maybe not here.
        'process_packets: loop {
            match select(radio.read_packet(), telemetry_subscriber.next_message()).await {
                Either::First(res) => {
                    if let Ok(c2_pkt) = res {
                        command_publisher.publish_immediate(c2_pkt);
                    } else {
                        defmt::warn!("radio read packet returned an error");
                    }
                }
                Either::Second(telem_msg) => {
                    match telem_msg {
                        WaitResult::Lagged(num_missed_pkts) => {
                            defmt::warn!("radio task missed sending {} outbound packets. Should channel have higher capacity?", num_missed_pkts);
                        },
                        WaitResult::Message(msg) => {
                            match msg {
                                TelemetryPacket::Basic(basic_telem_pkt) => {
                                    let res = radio.send_telemetry(basic_telem_pkt).await;
                                    if res.is_err() {
                                        defmt::warn!("radio task failed to send basic telemetry packet. Is the backing queue too small?");
                                    }
                                }
                                TelemetryPacket::Control(control_telem_pkt) => {
                                    let res = radio.send_control_debug_telemetry(control_telem_pkt).await;
                                    if res.is_err() {
                                        defmt::warn!("radio task failed to send control debug telemetry packet. Is the backing queue too small?");
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if radio_ndet.is_high() {
                defmt::error!("radio was unplugged.");
                break 'process_packets;
            }
        }
    }
}

pub fn start_radio_task(radio_task_spawner: SendSpawner,
        queue_spawner: Spawner,
        command_publisher: CommandsPublisher,
        telemetry_subscriber: TelemetrySubcriber,
        wifi_network: WifiCredential,
        radio_uart: RadioUART,
        radio_uart_rx_pin: RadioUartRxPin,
        radio_uart_tx_pin: RadioUartTxPin,
        _radio_uart_cts_pin: RadioUartCtsPin,
        _radio_uart_rts_pin: RadioUartRtsPin,
        radio_uart_rx_dma: RadioRxDMA,
        radio_uart_tx_dma: RadioTxDMA,
        radio_reset_pin: RadioResetPin,
        radio_ndet_pin: RadioNDetectPin) {
    let radio_uart_config = usart::Config::default();
    let radio_uart = Uart::new(radio_uart, radio_uart_rx_pin, radio_uart_tx_pin, SystemIrqs, radio_uart_tx_dma, radio_uart_rx_dma, radio_uart_config).unwrap();
    let (radio_uart_tx, radio_uart_rx) = Uart::split(radio_uart);

    queue_spawner.spawn(RADIO_RX_UART_QUEUE.spawn_task(radio_uart_rx)).unwrap();
    queue_spawner.spawn(RADIO_TX_UART_QUEUE.spawn_task(radio_uart_tx)).unwrap();

    radio_task_spawner.spawn(radio_task_entry(command_publisher, telemetry_subscriber, wifi_network, radio_reset_pin, radio_ndet_pin)).unwrap();
}