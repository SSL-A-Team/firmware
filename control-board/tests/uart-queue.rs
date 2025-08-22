#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use ateam_control_board::{
    queue::{self, Buffer},
    uart_queue::{UartReadQueue, UartWriteQueue},
};
use defmt_rtt as _;
use embassy_stm32::{
    self,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    peripherals::{DMA1_CH0, DMA1_CH1, UART7},
    usart::{self, Uart},
};
use panic_probe as _;
use static_cell::StaticCell;

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [Buffer<8>; 3] = [Buffer::EMPTY; 3];
static QUEUE_TX: UartWriteQueue<UART7, DMA1_CH0, 8, 3> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [Buffer<8>; 3] = [Buffer::EMPTY; 3];
static QUEUE_RX: UartReadQueue<UART7, DMA1_CH1, 8, 3> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

////////////////////////////////////
// To run test, connect hardware: //
//     PF7 <--> PF6               //
////////////////////////////////////

#[defmt_test::tests]
mod tests {
    use crate::*;
    use defmt::{assert, assert_eq};
    use embassy_futures::block_on;

    #[init]
    fn init() {
        let p = embassy_stm32::init(Default::default());
        let config = usart::Config::default();
        let irq = interrupt::take!(UART7);
        let usart = Uart::new(p.UART7, p.PF6, p.PF7, irq, p.DMA1_CH0, p.DMA1_CH1, config);
        let (tx, rx) = usart.split();

        let irq = interrupt::take!(CEC);
        irq.set_priority(interrupt::Priority::P6);
        let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
        let spawner = executor.start();

        spawner.spawn(QUEUE_RX.spawn_task(rx)).unwrap();
        spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap();
    }

    #[test]
    fn test_uart_queue() {
        let data1 = [0, 1, 2, 3, 4, 5, 6, 7];
        let data2 = [8, 9, 10, 11, 12, 13, 14];
        let data3 = [15, 16, 17];

        QUEUE_TX.enqueue_copy(&data1).unwrap();
        QUEUE_TX.enqueue_copy(&data2).unwrap();
        QUEUE_TX.enqueue_copy(&data3).unwrap();

        let result = QUEUE_TX.enqueue_copy(&[]);
        assert!(result == Err(queue::Error::QueueFull));

        block_on(QUEUE_RX.dequeue(|buf| {
            assert_eq!(buf, data1);
        }));
        block_on(QUEUE_RX.dequeue(|buf| {
            assert_eq!(buf, data2);
        }));
        block_on(QUEUE_RX.dequeue(|buf| {
            assert_eq!(buf, data3);
        }));
    }
}
