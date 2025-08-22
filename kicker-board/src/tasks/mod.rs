use embassy_stm32::{
    rcc::{
        mux::Adcsel, AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv,
        PllQDiv, PllRDiv, PllSource, Sysclk,
    },
    time::Hertz,
    Config,
};

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ClkSource {
    InternalOscillator,
    External8MHzOscillator,
}

pub fn get_system_config(clk_src: ClkSource) -> Config {
    let mut config = Config::default();

    if clk_src == ClkSource::External8MHzOscillator {
        // configure the external clock mode
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });

        // configure the main PLL
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2, // root frequency to PLL will be 4MHz after pre_div regardless of source
            mul: PllMul::MUL85,      // multiply up by 85 to get 340 MHz
            divp: Some(PllPDiv::DIV2), // 340 MHz / 2 = 170 MHz p which is feeds sysclk
            divq: Some(PllQDiv::DIV2), // 340 MHz / 2 = 170 MHz
            divr: Some(PllRDiv::DIV2), // 340 MHz / 2 = 170 MHz
        });
    } else {
        // configure the main PLL
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4, // root frequency to PLL will be 4MHz after pre_div regardless of source
            mul: PllMul::MUL85,      // multiply up by 85 to get 340 MHz
            divp: Some(PllPDiv::DIV2), // 340 MHz / 2 = 170MHz p which is feeds sysclk
            divq: Some(PllQDiv::DIV2), // 340 MHz / 2 = 170MHz
            divr: Some(PllRDiv::DIV2), // 340 MHz / 2 = 170MHz
        });
    };

    config.rcc.mux.adc12sel = Adcsel::PLL1_P;
    config.rcc.mux.adc345sel = Adcsel::PLL1_P;

    // configure the busses
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    // all configs should be good now, switch the system root clock from the raw HSI (16 MHz) to the PLL (168 MHz)
    config.rcc.sys = Sysclk::PLL1_R;

    config
}
