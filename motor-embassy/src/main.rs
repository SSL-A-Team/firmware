#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]

use defmt::*;
use defmt_rtt as _;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{
    self as _,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    peripherals::{DMA1_CH0, DMA1_CH1, UART7},
};
use motor_embassy::queue;
use motor_embassy::uart_queue::{UartReadQueue, UartWriteQueue};
use panic_probe as _;
use static_cell::StaticCell;

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [queue::Buffer<8>; 4] = [queue::Buffer::EMPTY; 4];
static QUEUE_TX: UartWriteQueue<UART7, DMA1_CH0, 8, 4> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [queue::Buffer<8>; 4] = [queue::Buffer::EMPTY; 4];
static QUEUE_RX: UartReadQueue<UART7, DMA1_CH1, 8, 4> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let p = embassy_stm32::init(Default::default());
    let config = usart::Config::default();
    let int = interrupt::take!(UART7);
    let usart = Uart::new(p.UART7, p.PF6, p.PF7, int, p.DMA1_CH0, p.DMA1_CH1, config);

    let (tx, rx) = usart.split();

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();
    info!("start1");

    spawner.spawn(QUEUE_RX.spawn_task(rx)).unwrap();
    spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap();

    QUEUE_TX.enqueue_copy(&[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
    QUEUE_TX.enqueue_copy(&[8, 9, 10, 11, 12, 13, 14]).unwrap();
    QUEUE_TX.enqueue_copy(&[15, 16, 17]).unwrap();
    info!("tx queued");

    QUEUE_RX
        .dequeue(|buf2| {
            info!("{:?}", buf2);
        })
        .await;
    QUEUE_RX
        .dequeue(|buf2| {
            info!("{:?}", buf2);
        })
        .await;
    QUEUE_RX
        .dequeue(|buf2| {
            info!("{:?}", buf2);
        })
        .await;

    info!("rx dequeued");

    loop {}
}
