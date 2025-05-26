use core::{mem::MaybeUninit};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::usart::{self, DataBits, Parity, StopBits, Uart};

use ateam_lib_stm32::{idle_buffered_uart_spawn_tasks, static_idle_buffered_uart_nl, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use ateam_common_packets::bindings::{PowerCommandPacket, PowerStatusPacket};
use embassy_time::{Duration, Instant, Ticker};
use crate::{pins::*, power_state::SharedPowerState, SystemIrqs};

const MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<PowerCommandPacket>();
const MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<PowerStatusPacket>();
const RX_BUF_DEPTH: usize = 3;
const TX_BUF_DEPTH: usize = 2;
static_idle_buffered_uart_nl!(COMS, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH);

const POWER_TELEM_PERIOD_MS: u64 = 1000;

#[macro_export]
macro_rules! create_coms_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $shared_power_state:ident, $coms_telemetry_subscriber:ident, $p:ident) => {
        ateam_power_board::tasks::coms_task::start_coms_task($main_spawner, $uart_queue_spawner, $shared_power_state, $coms_telemetry_subscriber,
            $p.USART1, $p.PA10, $p.PA9, $p.DMA1_CH5, $p.DMA1_CH4,
        ).await;
    };
}

#[embassy_executor::task]
async fn coms_task_entry(
    uart: &'static IdleBufferedUart<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
    read_queue: &'static UartReadQueue<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
    write_queue: &'static UartWriteQueue<MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
    shared_power_state: &'static SharedPowerState,
    mut telemetry_subscriber: TelemetrySubscriber,
) {
    let mut loop_rate_ticker = Ticker::every(Duration::from_millis(10));
    let mut latest_telem_packet: Option<PowerStatusPacket> = None;
    let mut last_sent_packet_time = Instant::now();

    loop {
        // read incoming packets
        while let Ok(res) = read_queue.try_dequeue() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<PowerCommandPacket>() {
                defmt::warn!("Got invalid packet of len {:?} (expected {:?}) data: {:?}", buf.len(), core::mem::size_of::<PowerCommandPacket>(), buf);
                continue;
            }
                        // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                let mut command_packet: PowerCommandPacket = { MaybeUninit::zeroed().assume_init() };

                // copy receieved uart bytes into packet
                let state = &mut command_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<PowerCommandPacket>() {
                    *state.offset(i as isize) = buf[i];
                }

                // Update power state from PowerCommandPacket
                if command_packet.force_shutdown() != 0 {
                    shared_power_state.set_shutdown_force(true);
                }

                if command_packet.ready_shutdown() != 0 {
                    shared_power_state.set_shutdown_ready(true);
                }

                if command_packet.request_shutdown() != 0 {
                    shared_power_state.set_shutdown_requested(true);
                }

                if command_packet.cancel_shutdown() != 0 {
                    shared_power_state.set_shutdown_ready(false);
                    shared_power_state.set_shutdown_requested(false);
                }
            }
        }

        // read channel
        loop {
            if let Some(telemetry_packet) = telemetry_subscriber.try_next_message_pure() {
                latest_telem_packet = Some(telemetry_packet);
            } else {
                break
            }
        }

        // send telemetry to control board
        if latest_telem_packet.is_some() && 
           (Instant::now() - last_sent_packet_time >= Duration::from_millis(POWER_TELEM_PERIOD_MS)) {

            let mut telem_packet = latest_telem_packet.unwrap();

            // update fields from current power state
            telem_packet.set_shutdown_requested(0);

            let struct_bytes;
            unsafe {
                struct_bytes = core::slice::from_raw_parts(
                    (&telem_packet as *const PowerStatusPacket) as *const u8,
                    core::mem::size_of::<PowerStatusPacket>(),
                );
            }
            write_queue.enqueue_copy(struct_bytes);
            last_sent_packet_time = Instant::now();
            defmt::trace!("Sent telemetry packet")
        }

        loop_rate_ticker.next().await;
    }
}

pub fn power_uart_config() -> usart::Config {
    let mut power_uart_config = usart::Config::default();
    power_uart_config.baudrate = 115_200;
    power_uart_config.data_bits = DataBits::DataBits8;
    power_uart_config.stop_bits = StopBits::STOP1;
    power_uart_config.parity = Parity::ParityNone;

    power_uart_config
}

pub async fn start_coms_task(spawner: Spawner, uart_queue_spawner: SendSpawner, shared_power_state: &'static SharedPowerState, telemetry_subscriber: TelemetrySubscriber,
    uart: ComsUart, uart_rx_pin: ComsUartRxPin, uart_tx_pin: ComsUartTxPin, uart_rx_dma: ComsDmaRx, uart_tx_dma: ComsDmaTx
    ) {
    let uart_config = power_uart_config();
    let coms_uart = Uart::new(uart, uart_rx_pin, uart_tx_pin, SystemIrqs, uart_tx_dma, uart_rx_dma, uart_config).unwrap();
    COMS_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, COMS, coms_uart);
    spawner.spawn(coms_task_entry(
        &COMS_IDLE_BUFFERED_UART, &COMS_IDLE_BUFFERED_UART.get_uart_read_queue(), &COMS_IDLE_BUFFERED_UART.get_uart_write_queue(), shared_power_state, telemetry_subscriber, 
    )).expect("failed to spawn coms task");
}