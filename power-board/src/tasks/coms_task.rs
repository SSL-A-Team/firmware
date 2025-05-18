use core::{mem::MaybeUninit};
use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{usart, usart::Uart};

use ateam_lib_stm32::{idle_buffered_uart_spawn_tasks, static_idle_buffered_uart_nl, uart::queue::{IdleBufferedUart, UartReadQueue, UartWriteQueue}};
use ateam_common_packets::bindings::{PowerCommandPacket, PowerStatusPacket};
use embassy_time::{Duration, Ticker};
use crate::{pins::*, SystemIrqs};

const MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<PowerCommandPacket>();
const MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<PowerStatusPacket>();
const RX_BUF_DEPTH: usize = 3;
const TX_BUF_DEPTH: usize = 2;
static_idle_buffered_uart_nl!(COMS, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH);

#[macro_export]
macro_rules! create_coms_task {
    ($main_spawner:ident, $uart_queue_spawner:ident, $p:ident) => {
        ateam_power_board::tasks::coms_task::start_coms_task($main_spawner, $uart_queue_spawner,
            $p.USART1, $p.PA10, $p.PA9, $p.DMA1_CH5, $p.DMA1_CH4,
        ).await;
    };
}

#[embassy_executor::task]
async fn coms_task_entry(
    uart: &'static IdleBufferedUart<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
    read_queue: &'static UartReadQueue<MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
    write_queue: &'static UartWriteQueue<MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
) {
    let mut loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    loop {
        // read packets
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
            }
        }
        // read channels
        // send packets
        // write_queue.enqueue();
        loop_rate_ticker.next().await;
    }
}

pub async fn start_coms_task(spawner: Spawner, uart_queue_spawner: SendSpawner, 
    uart: ComsUart, uart_rx_pin: ComsUartRxPin, uart_tx_pin: ComsUartTxPin, uart_rx_dma: ComsDmaRx, uart_tx_dma: ComsDmaTx
    ) {
    let uart_config = usart::Config::default();
    let coms_uart = Uart::new(uart, uart_rx_pin, uart_tx_pin, SystemIrqs, uart_tx_dma, uart_rx_dma, uart_config).unwrap();
    COMS_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, COMS, coms_uart);
    spawner.spawn(coms_task_entry(
        &COMS_IDLE_BUFFERED_UART, &COMS_IDLE_BUFFERED_UART.get_uart_read_queue(), &COMS_IDLE_BUFFERED_UART.get_uart_write_queue()
    )).expect("failed to spawn coms task");
}