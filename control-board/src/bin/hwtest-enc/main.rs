#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]

use control::Control;
use defmt_rtt as _;
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;

use apa102_spi::Apa102;
use ateam_control_board::{
    colors::*,
    stm32_interface::get_bootloader_uart_config};
use embassy_stm32::{
    dma::NoDma,
    gpio::{Input, Level, Output, Pull, Speed},
    interrupt,
    spi,
    time::{hz, mhz},
    usart::Uart,
};
use embassy_executor::InterruptExecutor;
use smart_leds::SmartLedsWrite;
use static_cell::StaticCell;

mod control;

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

// Angular velocity commanded in rads/sec
const ROBOT_VEL_ANGULAR: f32 = 10.;
// Expected RPM read from encoder for commanded velocity when wheels have no load
const EXPECTED_RPM_NO_LOAD: f32 = 370.;
// Allowed tolerance for expected RPM in percent
const EXPECTED_RPM_TOLERANCE: f32 = 0.20;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    // Delay so dotstar and STSPIN can turn on
    Timer::after(Duration::from_millis(50)).await;

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let dot_spi = spi::Spi::new_txonly(
        p.SPI3,
        p.PB3,
        p.PB5,
        NoDma,
        NoDma,
        hz(1_000_000),
        spi::Config::default(),
    );
    let mut dotstar = Apa102::new(dot_spi);
    dotstar
        .write([COLOR_BLUE, COLOR_OFF].iter().cloned())
        .unwrap();

    let btn0 = Input::new(p.PD5, Pull::None);

    let mut led0 = Output::new(p.PF3, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PF2, Level::Low, Speed::Low);
    let mut led2 = Output::new(p.PF1, Level::Low, Speed::Low);
    let mut led3 = Output::new(p.PF0, Level::Low, Speed::Low);

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

    let ret = control.load_firmware().await;
    if let Err(err) = ret {
        if err.front_right {
            defmt::error!("Error flashing FR");
        } else {
            led3.set_high();
        }
        if err.front_left {
            defmt::error!("Error flashing FL");
        } else {
            led0.set_high();
        }
        if err.back_right {
            defmt::error!("Error flashing BR");
        } else {
            led2.set_high();
        }
        if err.back_left {
            defmt::error!("Error flashing BL");
        } else {
            led1.set_high();
        }
        if err.drib {
            defmt::error!("Error flashing DRIB");
        }
        dotstar
            .write([COLOR_RED, COLOR_RED].iter().cloned())
            .unwrap();
        defmt::panic!("Error flashing STSPINs")
    } else {
        defmt::info!("All STSPINs flashed correctly")
    }

    dotstar.write([COLOR_YELLOW].iter().cloned()).unwrap();

    loop {
        while btn0.is_high() {}
        while btn0.is_low() {}

        let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));

        loop {
            main_loop_rate_ticker.next().await;
            control.tick(ROBOT_VEL_ANGULAR, 64.);

            let err_fr = control.front_right_motor.read_is_error();
            let err_fl = control.front_left_motor.read_is_error();
            let err_br = control.back_right_motor.read_is_error();
            let err_bl = control.back_left_motor.read_is_error();
            let err_drib = control.drib_motor.read_is_error();

            let rpm_fr = control.front_right_motor.read_rpm();
            let rpm_fl = control.front_left_motor.read_rpm();
            let rpm_br = control.back_right_motor.read_rpm();
            let rpm_bl = control.back_left_motor.read_rpm();

            let rpm_ok_fr = rpm_fr > EXPECTED_RPM_NO_LOAD * (1. - EXPECTED_RPM_TOLERANCE)
                && rpm_fr < EXPECTED_RPM_NO_LOAD * (1. + EXPECTED_RPM_TOLERANCE);
            let rpm_ok_fl = rpm_fl > EXPECTED_RPM_NO_LOAD * (1. - EXPECTED_RPM_TOLERANCE)
                && rpm_fl < EXPECTED_RPM_NO_LOAD * (1. + EXPECTED_RPM_TOLERANCE);
            let rpm_ok_br = rpm_br > EXPECTED_RPM_NO_LOAD * (1. - EXPECTED_RPM_TOLERANCE)
                && rpm_br < EXPECTED_RPM_NO_LOAD * (1. + EXPECTED_RPM_TOLERANCE);
            let rpm_ok_bl = rpm_bl > EXPECTED_RPM_NO_LOAD * (1. - EXPECTED_RPM_TOLERANCE)
                && rpm_bl < EXPECTED_RPM_NO_LOAD * (1. + EXPECTED_RPM_TOLERANCE);

            if err_fr || err_fl || err_br || err_bl || err_drib {
                dotstar
                    .write([COLOR_BLUE, COLOR_RED].iter().cloned())
                    .unwrap();
            } else if !rpm_ok_fr || !rpm_ok_fl || !rpm_ok_br || !rpm_ok_bl {
                dotstar
                    .write([COLOR_BLUE, COLOR_YELLOW].iter().cloned())
                    .unwrap();
            } else {
                dotstar
                    .write([COLOR_BLUE, COLOR_GREEN].iter().cloned())
                    .unwrap();
            }

            if !err_fl && rpm_ok_fl {
                led0.set_high();
            } else {
                led0.set_low();
            }
            if !err_bl && rpm_ok_bl {
                led1.set_high();
            } else {
                led1.set_low();
            }
            if !err_br && rpm_ok_br {
                led2.set_high();
            } else {
                led2.set_low();
            }
            if !err_fr && rpm_ok_fr {
                led3.set_high();
            } else {
                led3.set_low();
            }

            if btn0.is_low() {
                while btn0.is_low() {}
                break;
            }
        }

        dotstar.write([COLOR_YELLOW].iter().cloned()).unwrap();
    }
}
