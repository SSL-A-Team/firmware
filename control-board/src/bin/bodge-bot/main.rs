#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use ateam_common_packets::bindings_radio::{KickRequest, BasicControl};
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
use ateam_control_board::{
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

    // info!("booted");

    // let mut led0 = Output::new(p.PF3, Level::Low, Speed::High);

    // loop {
    //     Timer::after(Duration::from_millis(1000)).await;
    //     led0.toggle();
    // };

    spawner
        .spawn(power_off_task(p.PF5, p.EXTI5, p.PF4))
        .unwrap();

    let radio_int = interrupt::take!(USART10);
    let radio_usart = Uart::new(
        p.USART10, p.PE2, p.PE3, radio_int, p.DMA2_CH0, p.DMA2_CH1, config,
    );

    let rotary = Rotary::new(p.PG9, p.PG10, p.PG11, p.PG12);
    let mut shell_indicator = ShellIndicator::new(p.PD0, p.PD1, p.PD3, p.PD4);
    let team_dip0 = Input::new(p.PD14, Pull::Down);

    // let mut kicker = Output::new(p.PF9, Level::Low, Speed::High);

    let robot_id = rotary.read();
    info!("id: {}", robot_id);
    shell_indicator.set(robot_id);
    let team = if team_dip0.is_high() {
        TeamColor::Blue
    } else {
        TeamColor::Yellow
    };

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

    // loop {}

    // let token = unsafe {
    //     (&mut *(&RADIO_TEST as *const _
    //         as *mut RadioTest<
    //             MAX_TX_PACKET_SIZE,
    //             MAX_RX_PACKET_SIZE,
    //             TX_BUF_DEPTH,
    //             RX_BUF_DEPTH,
    //             RadioUART,
    //             RadioRxDMA,
    //             RadioTxDMA,
    //             RadioReset,
    //         >))
    //         .setup(&spawner, radio_usart, p.PC13, robot_id, team)
    //         .await
    // };
    // spawner.spawn(token).unwrap();

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    let mut last_kicked = 1000;
    loop {
        // let latest = RADIO_TEST.get_latest_control();
        let latest = Some(BasicControl{
            vel_x_linear: 0.,
            vel_y_linear: 0.,
            vel_z_angular: 0.05,
            kick_vel: 0.,
            dribbler_speed: 0.,
            kick_request: 0,
        });
        let telemetry = control.tick(latest);
        if let Some(telemetry) = telemetry {
            // info!("{:?}", defmt::Debug2Format(&telemetry));
        //     RADIO_TEST.send_telemetry(telemetry).await;
        }
        if let Some(latest) = &latest {
            if last_kicked > 100 && latest.kick_request == KickRequest::KR_KICK_NOW {
                // kicker.set_high();
                // Timer::after(Duration::from_micros(3500)).await;
                // kicker.set_low();
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
