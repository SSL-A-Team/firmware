#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
// #![feature(async_fn_in_trait)]

use ateam_common_packets::bindings_radio::BasicTelemetry;
use control::Control;
use defmt::info;
use embassy_stm32::{
    executor::InterruptExecutor,
    gpio::{Level, OutputOpenDrain, Pull, Speed},
    interrupt::{self, InterruptExt},
    time::mhz,
    usart::{self, Uart},
};
use embassy_time::{Duration, Timer, Ticker};
use futures_util::StreamExt;
use motor_embassy::{drivers::radio::TeamColor, stm32_interface::get_bootloader_uart_config};
use panic_probe as _;
use pins::{RadioReset, RadioRxDMA, RadioTxDMA, RadioUART};
use radio::{
    RadioTest, BUFFERS_RX, BUFFERS_TX, MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE, RX_BUF_DEPTH,
    TX_BUF_DEPTH,
};
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

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let radio_usart = Uart::new(p.USART2, p.PD6, p.PD5, p.DMA2_CH0, p.DMA2_CH1, config);
    let radio_int = interrupt::take!(USART2);

    let front_right_usart = Uart::new(
        p.UART5,
        p.PB12,
        p.PB6,
        p.DMA1_CH0,
        p.DMA1_CH1,
        get_bootloader_uart_config(),
    );
    let front_left_usart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        p.DMA1_CH2,
        p.DMA1_CH3,
        get_bootloader_uart_config(),
    );
    let back_left_usart = Uart::new(
        p.UART4,
        p.PD0,
        p.PD1,
        p.DMA1_CH4,
        p.DMA1_CH5,
        get_bootloader_uart_config(),
    );
    let back_right_usart = Uart::new(
        p.USART3,
        p.PB11,
        p.PB10,
        p.DMA1_CH6,
        p.DMA1_CH7,
        get_bootloader_uart_config(),
    );

    let mut control = Control::new(
        &spawner,
        front_right_usart,
        front_left_usart,
        back_left_usart,
        back_right_usart,
        interrupt::take!(UART5),
        interrupt::take!(UART7),
        interrupt::take!(UART4),
        interrupt::take!(USART3),
        p.PB1,
        p.PG2,
        p.PG0,
        p.PF4,
        p.PG3,
        p.PB2,
        p.PG1,
        p.PA3,
    );

    control.load_firmware().await;

    let token = unsafe {
        (&mut *(&RADIO_TEST as *const _
            as *mut RadioTest<
                MAX_TX_PACKET_SIZE,
                MAX_RX_PACKET_SIZE,
                TX_BUF_DEPTH,
                RX_BUF_DEPTH,
                RadioUART,
                RadioRxDMA,
                RadioTxDMA,
                RadioReset,
            >))
            .setup(&spawner, radio_usart, radio_int, p.PC0, 0, TeamColor::Blue)
            .await
    };
    spawner.spawn(token).unwrap();

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    loop {
        let latest = RADIO_TEST.get_latest_control();
        control.tick(latest);

        // info!("{}", defmt::Debug2Format(&latest));
        // RADIO_TEST
        //     .send_telemetry(BasicTelemetry {
        //         sequence_number: 0,
        //         robot_revision_major: 0,
        //         robot_revision_minor: 0,
        //         battery_level: 0.,
        //         battery_temperature: 0.,
        //         _bitfield_align_1: [],
        //         _bitfield_1: BasicTelemetry::new_bitfield_1(
        //             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //         ),
        //         motor_0_temperature: 0.,
        //         motor_1_temperature: 0.,
        //         motor_2_temperature: 0.,
        //         motor_3_temperature: 0.,
        //         motor_4_temperature: 0.,
        //         kicker_charge_level: 0.,
        //     })
        //     .await;

        // Timer::after(Duration::from_millis(500)).await;

        main_loop_rate_ticker.next().await;

    }
}
