use embassy_stm32::{rcc::{AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllSource, Sysclk}, time::Hertz, Config};

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ClkSource {
    InternalOscillator,
    External8MHzOscillator,
}

pub fn get_system_config(clk_src: ClkSource) -> Config {
    let mut config = Config::default();

    let pre_div = if clk_src == ClkSource::External8MHzOscillator {
        // configure the external clock mode
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        // set the pll source to be the high speed external oscillator
        config.rcc.pll_src = PllSource::HSE;

        // 8MHz ext osc means we divide by 4 to get 2MHz root clock
        PllPreDiv::DIV4
    } else {
        // set the pll source to be the high speed intenal oscillator
        config.rcc.pll_src = PllSource::HSI;

        // 16MHz internal osc means we divide by 8 to get a 2MHz root clock
        PllPreDiv::DIV8
    };

    // configure the main PLL
    config.rcc.pll = Some(Pll {
        prediv: pre_div, // root frequency to PLL will be 2MHz after pre_div regardless of source
        mul: PllMul::MUL168, // multiply up by 168 to get 336 MHz
        divp: Some(PllPDiv::DIV2), // 336 MHz / 2 = 168 MHZ p which is feeds sysclk
        divq: Some(PllQDiv::DIV7), // 336 MHz / 7 = 48Mhz which feeds the 48MHz bus exactly
        divr: None, // not using the I2S clock
    });

    // configure the busses
    config.rcc.ahb_pre = AHBPrescaler::DIV1; // don't scale the AHB bus, it has a max frequency of 168 MHz, so it can match the sysclk
    config.rcc.apb1_pre = APBPrescaler::DIV4; // 168 / 4 = 42 MHz which is the max frequency of APB1
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 168 / 2 = 84 MHz which is the max frequency of APB2

    // all configs should be good now, switch the system root clock from the raw HSI (16 MHz) to the PLL (168 MHz)
    config.rcc.sys = Sysclk::PLL1_P;

    return config;
}