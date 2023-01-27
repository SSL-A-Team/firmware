#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use ateam_common_packets::bindings_radio::KickRequest;
use control::Control;
use defmt::info;
use embassy_stm32::{
    executor::InterruptExecutor,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt::{self, InterruptExt},
    time::mhz,
    usart::{self, Uart},
};
use embassy_time::{Duration, Ticker, Timer};
use futures_util::StreamExt;
use motor_embassy::{
    drivers::{radio::TeamColor, rotary::Rotary, shell_indicator::ShellIndicator},
    stm32_interface::get_bootloader_uart_config,
};
use panic_probe as _;
use pins::{
    PowerStateExti, PowerStatePin, RadioReset, RadioRxDMA, RadioTxDMA, RadioUART,
    ShutdownCompletePin,
};
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

    spawner
        .spawn(power_off_task(p.PD15, p.EXTI15, p.PD14))
        .unwrap();

    let radio_int = interrupt::take!(USART2);
    let radio_usart = Uart::new(
        p.USART2, p.PD6, p.PD5, radio_int, p.DMA2_CH0, p.DMA2_CH1, config,
    );

    let rotary = Rotary::new(p.PG6, p.PG5, p.PG4, p.PG8);
    let mut shell_indicator = ShellIndicator::new(p.PE10, p.PD11, p.PD12, p.PD13);
    let team_dip0 = Input::new(p.PE12, Pull::Down);

    let mut kicker = Output::new(p.PF9, Level::Low, Speed::High);

    let robot_id = rotary.read();
    info!("id: {}", robot_id);
    shell_indicator.set(robot_id);
    let team = if team_dip0.is_high() {
        TeamColor::Blue
    } else {
        TeamColor::Yellow
    };

    let front_right_int = interrupt::take!(UART5);
    let front_left_int = interrupt::take!(UART7);
    let back_left_int = interrupt::take!(UART4);
    let back_right_int = interrupt::take!(USART3);

    let front_right_usart = Uart::new(
        p.UART5,
        p.PB12,
        p.PB6,
        front_right_int,
        p.DMA1_CH0,
        p.DMA1_CH1,
        get_bootloader_uart_config(),
    );
    let front_left_usart = Uart::new(
        p.UART7,
        p.PF6,
        p.PF7,
        front_left_int,
        p.DMA1_CH2,
        p.DMA1_CH3,
        get_bootloader_uart_config(),
    );
    let back_left_usart = Uart::new(
        p.UART4,
        p.PD0,
        p.PD1,
        back_left_int,
        p.DMA1_CH4,
        p.DMA1_CH5,
        get_bootloader_uart_config(),
    );
    let back_right_usart = Uart::new(
        p.USART3,
        p.PB11,
        p.PB10,
        back_right_int,
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
        p.PB1,
        p.PG2,
        p.PG0,
        p.PF4,
        p.PB2,
        p.PG3,
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
            .setup(&spawner, radio_usart, p.PC0, robot_id, team)
            .await
    };
    spawner.spawn(token).unwrap();

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    let mut last_kicked = 1000;
    loop {
        let latest = RADIO_TEST.get_latest_control();
        let telemetry = control.tick(latest);
        if let Some(telemetry) = telemetry {
            RADIO_TEST.send_telemetry(telemetry).await;
        }

        if let Some(latest) = &latest {
            if last_kicked > 100 && latest.kick_request == KickRequest::KR_KICK_NOW {
                kicker.set_high();
                Timer::after(Duration::from_micros(7000)).await;
                kicker.set_low();
                last_kicked = 0;
            }
        }
        last_kicked = core::cmp::min(last_kicked + 1, 1000);

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
