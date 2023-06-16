#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use apa102_spi::Apa102;
use ateam_common_packets::bindings_radio::{BasicControl, KickRequest};
use ateam_control_board::{
    drivers::{radio::TeamColor, rotary::Rotary, shell_indicator::ShellIndicator},
    include_external_cpp_bin,
    queue::Buffer,
    stm32_interface::{get_bootloader_uart_config, Stm32Interface},
    uart_queue::{UartReadQueue, UartWriteQueue},
};
use control::Control;
use defmt::info;
use embassy_stm32::{
    dma::NoDma,
    executor::InterruptExecutor,
    exti::ExtiInput,
    gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed},
    interrupt::{self, InterruptExt},
    peripherals::{DMA2_CH4, DMA2_CH5, USART6},
    spi,
    time::{hz, mhz},
    usart::{self, Uart},
};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;
use panic_probe as _;
use pins::{
    PowerStateExti, PowerStatePin, RadioReset, RadioRxDMA, RadioTxDMA, RadioUART,
    ShutdownCompletePin,
};
use radio::{
    RadioTest, BUFFERS_RX, BUFFERS_TX, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH,
    TX_BUF_DEPTH,
};
use smart_leds::{SmartLedsWrite, RGB8};
use static_cell::StaticCell;

mod control;
mod pins;
mod radio;

static RADIO_TEST: RadioTest<
    MAX_TX_PACKET_SIZE,
    MAX_RX_PACKET_SIZE,
    TX_BUF_DEPTH,
    RX_BUF_DEPTH,
    RadioUART,
    RadioRxDMA,
    RadioTxDMA,
    RadioReset,
> = RadioTest::new(unsafe { &mut BUFFERS_TX }, unsafe { &mut BUFFERS_RX });

// include_external_cpp_bin! {KICKER_FW_IMG, "kicker.bin"}

#[link_section = ".sram4"]
static mut SPI6_BUF: [u8; 4] = [0x0; 4];

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

// static RADIO: Radio<RadioUART, RadioRxDMA, RadioTxDMA> = Radio::new();
static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);
    let config = usart::Config::default();

    info!("system core initialized");

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let mut imu_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );

    let mut dotstar = Apa102::new(imu_spi);
    dotstar.write([RGB8 { r: 10, g: 0, b: 0 }].iter().cloned());

    let front_right_int = interrupt::take!(USART1);
    let front_left_int = interrupt::take!(UART4);
    let back_left_int = interrupt::take!(UART7);
    let back_right_int = interrupt::take!(UART8);
    let drib_int = interrupt::take!(UART5);

    let front_right_usart = Uart::new(
        p.USART1,
        p.PB15,
        p.PB14,
        front_right_int,
        p.DMA1_CH0,
        p.DMA1_CH1,
        get_bootloader_uart_config(),
    );
    let front_left_usart = Uart::new(
        p.UART4,
        p.PA1,
        p.PA0,
        front_left_int,
        p.DMA1_CH2,
        p.DMA1_CH3,
        get_bootloader_uart_config(),
    );
    let back_left_usart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        back_left_int,
        p.DMA1_CH4,
        p.DMA1_CH5,
        get_bootloader_uart_config(),
    );
    let back_right_usart = Uart::new(
        p.UART8,
        p.PE0,
        p.PE1,
        back_right_int,
        p.DMA1_CH6,
        p.DMA1_CH7,
        get_bootloader_uart_config(),
    );
    let drib_usart = Uart::new(
        p.UART5,
        p.PB12,
        p.PB13,
        drib_int,
        p.DMA2_CH2,
        p.DMA2_CH3,
        get_bootloader_uart_config(),
    );

    let ball_detected_thresh = 1.0;
    let mut control = Control::new(
        &spawner,
        front_right_usart,
        front_left_usart,
        back_left_usart,
        back_right_usart,
        drib_usart,
        p.PD8,
        p.PC1,
        p.PF8,
        p.PB9,
        p.PD13,
        p.PD9,
        p.PC0,
        p.PF9,
        p.PB8,
        p.PD12,
        ball_detected_thresh,
    );

    control.load_firmware().await;

    dotstar.write([RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());


    dotstar.write([RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 0, b: 10 }].iter().cloned());

    dotstar.write([RGB8 { r: 0, g: 10, b: 0 }, RGB8 { r: 0, g: 10, b: 0 }].iter().cloned());

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    loop {

        main_loop_rate_ticker.next().await;
    }
}

#[embassy_executor::task]
async fn power_off_task(
    power_state_pin: PowerStatePin,
    exti: PowerStateExti,
    shutdown_pin: ShutdownCompletePin,
) {
    let power_state = Input::new(power_state_pin, Pull::None);
    let mut shutdown = Output::new(shutdown_pin, Level::Low, Speed::Low);
    let mut power_state = ExtiInput::new(power_state, exti);
    power_state.wait_for_falling_edge().await;
    shutdown.set_high();
    loop {}
}
