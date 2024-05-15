#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(
    maybe_uninit_uninit_array,
    const_maybe_uninit_uninit_array,
    maybe_uninit_slice,
    maybe_uninit_write_slice
)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]
#![feature(const_fn_floating_point_arithmetic)]
#![feature(sync_unsafe_cell)]

use embassy_stm32::{
    bind_interrupts, peripherals, rcc::{
        mux::{Adcsel, Saisel, Sdmmcsel, Spi6sel, Usart16910sel, Usart234578sel, Usbsel},
        AHBPrescaler, APBPrescaler,
        Hse, HseMode,
        Pll, PllDiv, PllMul, PllPreDiv, PllSource,
        Sysclk,
        VoltageScale
    }, time::Hertz, usart, Config
};

pub mod parameter_interface;
pub mod pins;
// pub mod radio;
pub mod robot_state;
pub mod stm32_interface;
pub mod stspin_motor;

pub mod drivers;
pub mod motion;
pub mod tasks;

bind_interrupts!(struct SystemIrqs {
    USART10 => usart::InterruptHandler<peripherals::USART10>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART5 => usart::InterruptHandler<peripherals::UART5>;
});

pub mod colors {
    use smart_leds::RGB8;

    pub const COLOR_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0};
    pub const COLOR_RED: RGB8 = RGB8 { r: 10, g: 0, b: 0};
    pub const COLOR_YELLOW: RGB8 = RGB8 { r: 10, g: 10, b: 0};
    pub const COLOR_GREEN: RGB8 = RGB8 { r: 0, g: 10, b: 0};
    pub const COLOR_BLUE: RGB8 = RGB8 { r: 0, g: 0, b: 10};
}

#[macro_export]
macro_rules! include_external_cpp_bin {
    ($var_name:ident, $bin_file:literal) => {
        pub static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../motor-controller/build/bin/", $bin_file)).len()]
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../motor-controller/build/bin/", $bin_file));
    }
}

#[macro_export]
macro_rules! include_kicker_bin {
    ($var_name:ident, $bin_file:literal) => {
        pub static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../kicker-board/target/thumbv7em-none-eabihf/release/", $bin_file)).len()]
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../kicker-board/target/thumbv7em-none-eabihf/release/", $bin_file));
    }
}

pub const BATTERY_MIN_VOLTAGE: f32 = 19.0;
pub const BATTERY_MAX_VOLTAGE: f32 = 25.2;
pub const BATTERY_BUFFER_SIZE: usize = 10;
pub const ADC_VREFINT_NOMINAL: f32 = 1050.0; // mV

pub const fn adc_raw_to_v(adc_raw: f32) -> f32 {
    adc_raw / ADC_VREFINT_NOMINAL
}

pub const fn adc_v_to_battery_voltage(adc_mv: f32) -> f32 {
    (adc_mv / 2.762) * BATTERY_MAX_VOLTAGE
}

pub fn get_system_config() -> Config {
    let mut config = Config::default();

    // we have an 8Mhz external crystal
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });

    // I'm not actually sure what this does. We dont select it but other
    // examples also don't use it in a mux selection, but they still turn
    // it on.
    config.rcc.csi = true;

    // turn on the hsi48 as a primordial ADC source
    config.rcc.hsi48 = Some(Default::default());

    // configure the PLLs
    // validated in ST Cube MX
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL64, // PllMul::MUL68,
        divp: Some(PllDiv::DIV1), // 544 MHz
        divq: Some(PllDiv::DIV4), // 136 MHz
        divr: Some(PllDiv::DIV2)  // 272 MHz
    });
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL31,
        divp: Some(PllDiv::DIV5), // 49.6 MHz
        divq: Some(PllDiv::DIV2), // 124 MHz
        divr: Some(PllDiv::DIV1)  // 248 MHz
    });
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV2,
        mul: PllMul::MUL93,
        divp: Some(PllDiv::DIV2), // 186 Mhz
        divq: Some(PllDiv::DIV3), // 124 MHz
        divr: Some(PllDiv::DIV3)  // 124 MHz
    });

    // configure core busses
    config.rcc.sys = Sysclk::PLL1_P; // 544 MHz
    config.rcc.d1c_pre = AHBPrescaler::DIV1; // 544 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // 272 MHz

    // configure peripheral busses
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 136 MHz

    // configure peripheral subgroup clock selection muxes
    // this is non exhaustive, if other things are turned on
    // add an entry
    config.rcc.mux.spi123sel = Saisel::PLL1_Q; // 136 MHz
    config.rcc.mux.usart234578sel = Usart234578sel::PCLK1; // 136 MHz
    config.rcc.mux.usart16910sel = Usart16910sel::PCLK2; // 136 MHz
    config.rcc.mux.spi6sel = Spi6sel::PCLK4; // 136 MHz
    config.rcc.mux.sdmmcsel = Sdmmcsel::PLL2_R; // 248 MHz
    config.rcc.mux.adcsel = Adcsel::PLL3_R; // 124 MHz
    config.rcc.mux.usbsel = Usbsel::PLL3_Q; // 124 MHz

    config.rcc.voltage_scale = VoltageScale::Scale0;

    config
}

