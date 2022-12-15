#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]

use defmt::*;
use defmt_rtt as _;

use embassy_futures::join::join;
use embassy_stm32::time::mhz;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{
    self as _,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    peripherals::{DMA1_CH0, DMA1_CH1, USART2},
};
use embassy_time::{Duration, Timer};
use motor_embassy::drivers::radio::RobotRadio;
use motor_embassy::queue;
use motor_embassy::uart_queue::{UartReadQueue, UartWriteQueue};
use panic_probe as _;
use static_cell::StaticCell;

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [queue::Buffer<256>; 4] = [queue::Buffer::EMPTY; 4];
static QUEUE_TX: UartWriteQueue<USART2, DMA1_CH0, 256, 4> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [queue::Buffer<256>; 8] = [queue::Buffer::EMPTY; 8];
static QUEUE_RX: UartReadQueue<USART2, DMA1_CH1, 256, 8> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);
    let config = usart::Config::default();
    let usart = Uart::new(p.USART2, p.PD6, p.PD5, p.DMA1_CH0, p.DMA1_CH1, config);

    let (tx, rx) = usart.split();

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();
    info!("start");

    let int = interrupt::take!(USART2);
    spawner.spawn(QUEUE_RX.spawn_task(rx, int)).unwrap();
    spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap();

    let mut radio = RobotRadio::new(&QUEUE_RX, &QUEUE_TX, p.PA3).await.unwrap();
    info!("radio created");
    radio.connect_to_network().await.unwrap();
    info!("radio connected");

    join(
        (|| async {
            loop {
                radio.send_data(&[b'a', b'b', b'c']).await.unwrap();
                Timer::after(Duration::from_millis(1000)).await;
            }
        })(),
        (|| async {
            loop {
                radio
                    .read_data(|data| {
                        info!("{}", data);
                    })
                    .await
                    .unwrap();
            }
        })(),
    )
    .await;

    loop {}
}
