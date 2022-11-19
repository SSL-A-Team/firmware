#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use defmt_rtt as _;
use defmt::*;
use embassy_stm32::{
    self,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    gpio::{Level, OutputOpenDrain, Pin, Pull, Speed, Output},
    peripherals::{DMA1_CH0, DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7},
    peripherals::{USART3, UART4, UART5, UART7},
    usart::{self, Uart, Parity, StopBits}, time::mhz,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

use motor_embassy::{
    stm32_interface::Stm32Interface,
    queue::{self, Buffer},
    uart_queue::{UartReadQueue, UartWriteQueue},
};


// motor pinout
// FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3
// BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1
// FrontRight Wheel - UART5  - tx pb6,  pb12,    boot pb1,  rst pb2
// BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pa8,  rst pa9

const MAX_TX_PACKET_SIZE: usize = 64;
const MAX_RX_PACKET_SIZE: usize = 64;

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; 3] = [Buffer::EMPTY; 3];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<UART7, DMA1_CH0, MAX_TX_PACKET_SIZE, 3> =
    UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; 3] = [Buffer::EMPTY; 3];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<UART7, DMA1_CH1, MAX_RX_PACKET_SIZE, 3> =
    UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    let mut config = usart::Config::default();
    config.baudrate = 2_000_000;
    config.parity = Parity::ParityEven;
    config.stop_bits = StopBits::STOP0P5;
    let usart = Uart::new(p.UART7, p.PF6, p.PF7, p.DMA1_CH0, p.DMA1_CH1, config);
    let (tx, rx) = usart.split();

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let int = interrupt::take!(UART7);
    spawner.spawn(FRONT_LEFT_QUEUE_RX.spawn_task(rx, int)).unwrap();
    spawner.spawn(FRONT_LEFT_QUEUE_TX.spawn_task(tx)).unwrap();

    let mut boot0_pin = Output::new(p.PG2, Level::Low, Speed::Medium);
    boot0_pin.set_low();

    let mut reset_pin = OutputOpenDrain::new(p.PG3, Level::Low, Speed::Medium, Pull::None);
    reset_pin.set_high();

    // let stm32_interface = Stm32Interface(FRONT_LEFT_QUEUE_RX, FRONT_LEFT_QUEUE_TX, Some(boot0_pin), Some(reset_pin));

    loop {
        info!("toggle pins");
        //boot0_pin.set_low();
        reset_pin.set_low();
        Timer::after(Duration::from_millis(1000)).await;
        //boot0_pin.set_high();
        reset_pin.set_high();
        Timer::after(Duration::from_millis(1000)).await;
    }
}