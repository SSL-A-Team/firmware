#![no_std]
#![feature(generic_const_exprs)]

pub mod limits;
pub mod pins;
pub mod tasks;

const ADC_VREF_EXT: f32 = 3.0;
const MV_PER_V: f32 = 1000.0;

pub const fn adc_raw_to_mv(sample: u16) -> f32 {
    (sample as f32 / 4096.0) * ADC_VREF_EXT * MV_PER_V
}

pub const fn adc_raw_vrefint_to_mv(sample: u16, vrefint_sample: u16) -> f32 {
    // From https://www.st.com/resource/en/datasheet/stm32g031g8.pdf
    // 6.3.3 Embedded internal reference voltage
    const VREFINT_MV: f32 = 1212.0; // mV

    adc_raw_to_mv(sample) * (VREFINT_MV / vrefint_sample as f32)
}