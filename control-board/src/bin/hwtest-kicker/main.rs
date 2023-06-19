#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use apa102_spi::Apa102;
use ateam_control_board::{
    drivers::kicker::Kicker,
    include_kicker_bin,
    queue::Buffer,
    stm32_interface::{get_bootloader_uart_config, Stm32Interface},
    uart_queue::{UartReadQueue, UartWriteQueue},
};
use defmt::info;
use embassy_stm32::{
    dma::NoDma,
    executor::InterruptExecutor,
    gpio::{Input, Level, Output, Speed, Pull},
    interrupt::{self, InterruptExt},
    peripherals::{DMA2_CH4, DMA2_CH5, USART6},
    spi,
    time::{hz, mhz},
    usart::Uart,
};
use embassy_time::{Duration, Ticker};
use futures_util::StreamExt;
use panic_probe as _;
use smart_leds::{SmartLedsWrite, RGB8};
use static_cell::StaticCell;

mod pins;

include_kicker_bin! {KICKER_FW_IMG, "hwtest-coms"}

const MAX_TX_PACKET_SIZE: usize = 12;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 12;
const RX_BUF_DEPTH: usize = 20;

#[link_section = ".axisram.buffers"]
static mut KICKER_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [Buffer::EMPTY; TX_BUF_DEPTH];
static KICKER_QUEUE_TX: UartWriteQueue<USART6, DMA2_CH4, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut KICKER_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut KICKER_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [Buffer::EMPTY; RX_BUF_DEPTH];
static KICKER_QUEUE_RX: UartReadQueue<USART6, DMA2_CH5, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut KICKER_BUFFERS_RX });

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));

    let p = embassy_stm32::init(stm32_config);

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let dotstar_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );
    let mut dotstar = Apa102::new(dotstar_spi);
    let _ = dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    info!("booted");

    let kicker_det = Input::new(p.PG8, Pull::Up);
    if kicker_det.is_high() {
        defmt::warn!("kicker appears unplugged!");
    }

    let kicker_int = interrupt::take!(USART6);
    let kicker_usart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        kicker_int,
        p.DMA2_CH4,
        p.DMA2_CH5,
        get_bootloader_uart_config(),
    );
    let (kicker_tx, kicker_rx) = kicker_usart.split();
    let kicker_boot0_pin = Output::new(p.PA8, Level::Low, Speed::Medium);
    let kicker_reset_pin = Output::new(p.PA9, Level::Low, Speed::Medium);

    spawner
        .spawn(KICKER_QUEUE_RX.spawn_task(kicker_rx))
        .unwrap();
    spawner
        .spawn(KICKER_QUEUE_TX.spawn_task(kicker_tx))
        .unwrap();

    let kicker_stm32_interface = Stm32Interface::new_noninverted_reset(
        &KICKER_QUEUE_RX,
        &KICKER_QUEUE_TX,
        Some(kicker_boot0_pin),
        Some(kicker_reset_pin),
    );

    info!("flashing kicker...");

    let kicker_firmware_image = KICKER_FW_IMG;
    let mut kicker = Kicker::new(kicker_stm32_interface, kicker_firmware_image);
    let _ = kicker.load_default_firmware_image().await;

    info!("kicker flash complete");

    kicker.set_telemetry_enabled(true);

    let _ = dotstar.write([RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 10, b: 0 }].iter().cloned());

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));
    loop {
        kicker.process_telemetry();

        // TODO print some telemetry or something

        kicker.send_command();

        main_loop_rate_ticker.next().await;
    }
}
