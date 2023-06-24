#![no_std]
#![feature(const_mut_refs)]
#![feature(const_fn_floating_point_arithmetic)]
#![feature(async_fn_in_trait)]
#![feature(type_alias_impl_trait)]
#![feature(maybe_uninit_slice)]
#![feature(maybe_uninit_uninit_array)]
#![feature(const_maybe_uninit_uninit_array)]

pub mod pins;
pub mod kick_manager;
pub mod queue;
pub mod uart_queue;
pub mod drivers;


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