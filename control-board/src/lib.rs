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

// pub mod fw_images;
pub mod motion;
pub mod pins;
pub mod stm32_interface;
pub mod stspin_motor;
pub mod parameter_interface;

pub mod drivers;

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
        pub static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../kicker-board/target/thumbv7em-none-eabi/release/", $bin_file)).len()]
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../kicker-board/target/thumbv7em-none-eabi/release/", $bin_file));
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


