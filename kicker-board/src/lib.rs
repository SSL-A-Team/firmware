#![no_std]
#![feature(const_fn_floating_point_arithmetic)]

pub mod pins;
pub mod kick_manager;

pub const ADC_VREFINT_NOMINAL: f32 = 1230.0; // mV

pub const fn adc_raw_to_mv(adc_raw: f32) -> f32 {
    adc_raw / ADC_VREFINT_NOMINAL
}

pub const fn adc_mv_to_battery_voltage(adc_mv: f32) -> f32 {
    adc_mv / 1000.0 * 200.0
}

pub const fn adc_mv_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv / 1000.0 / 1.65 * 25.2
}