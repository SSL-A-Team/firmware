#![no_std]
#![feature(const_mut_refs)]
#![feature(const_fn_floating_point_arithmetic)]
#![feature(type_alias_impl_trait)]
#![feature(maybe_uninit_slice)]
#![feature(maybe_uninit_uninit_array)]
#![feature(const_maybe_uninit_uninit_array)]
#![feature(sync_unsafe_cell)]

pub mod drivers;
pub mod tasks;

pub mod kick_manager;
pub mod pins;
// pub mod queue;
// pub mod uart_queue;

pub const ADC_VREFINT_NOMINAL: f32 = 1230.0; // mV

pub const fn adc_raw_to_v(adc_raw: f32, vrefint_sample: f32) -> f32 {
    adc_raw * ADC_VREFINT_NOMINAL / vrefint_sample / 1000.
}

pub const fn adc_v_to_battery_voltage(adc_mv: f32) -> f32 {
    (adc_mv / 1.65) * 25.2
}

pub const fn adc_200v_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv * 200.0
}

pub const fn adc_12v_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv * 12.0
}

pub const fn adc_6v2_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv * 6.2
}

pub const fn adc_5v0_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv * 5.0
}

pub const fn adc_3v3_to_rail_voltage(adc_mv: f32) -> f32 {
    adc_mv / 1.25 * 3.3
}