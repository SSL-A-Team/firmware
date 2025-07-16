#![no_std]
#![no_main]

#![allow(incomplete_features)]
#![feature(inherent_associated_types)]
#![feature(generic_const_exprs)]

#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(
    maybe_uninit_uninit_array,
    maybe_uninit_slice,
    maybe_uninit_write_slice
)]
#![feature(ptr_metadata)]
#![feature(sync_unsafe_cell)]
#![feature(variant_count)]


use ateam_common_packets::{bindings::{ErrorTelemetry, ParameterDataFormat}, radio::DataPacket};
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
use embassy_time::Instant;

pub mod parameter_interface;
pub mod pins;
pub mod robot_state;
pub mod songs;
pub mod stspin_motor;
pub mod image_hash;

pub mod drivers;
pub mod motion;
pub mod tasks;

bind_interrupts!(pub struct SystemIrqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    USART10 => usart::InterruptHandler<peripherals::USART10>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART9 => usart::InterruptHandler<peripherals::UART9>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

pub const DEBUG_RADIO_UART_QUEUES: bool = false;
pub const DEBUG_MOTOR_UART_QUEUES: bool = false;
pub const DEBUG_POWER_UART_QUEUES: bool = false;
pub const DEBUG_KICKER_UART_QUEUES: bool = false;

const ROBOT_VERSION_MAJOR: u8 = 3;
const ROBOT_VERSION_MINOR: u8 = 1;

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum MotorIndex {
    FrontLeft = 0,
    BackLeft = 1,
    BackRight = 2,
    FrontRight = 3,
}

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

pub const BATTERY_MIN_SAFE_VOLTAGE: f32 = 21.0;
pub const BATTERY_MIN_CRIT_VOLTAGE: f32 = 19.5;
pub const BATTERY_MAX_VOLTAGE: f32 = 26.0;
pub const BATTERY_BUFFER_SIZE: usize = 20;
pub const ADC_TO_BATTERY_DIVIDER: f32 = (11_500.0 + 1_000.0) / 1_000.0; 

pub const fn adc_v_to_battery_voltage(adc_v: f32) -> f32 {
    adc_v * ADC_TO_BATTERY_DIVIDER
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
        divp: Some(PllDiv::DIV7), // 35.8 MHz
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
    // config.rcc.mux.adcsel = Adcsel::PLL3_R; // 124 MHz
    config.rcc.mux.adcsel = Adcsel::PLL2_P;
    config.rcc.mux.usbsel = Usbsel::PLL3_Q; // 124 MHz

    config.rcc.voltage_scale = VoltageScale::Scale0;

    config
}

pub fn is_float_array_safe(arr: &[f32]) -> bool {
    for f in arr {
        if !is_float_safe(*f) {
            return false;
        }
    }
    return true;
}

pub const fn is_float_safe(f: f32) -> bool {
    !f.is_nan() && f.is_finite()
}

pub fn is_command_packet_safe(cmd_pck: DataPacket) -> bool {
    match cmd_pck {
        DataPacket::BasicControl(basic_control) => {
            is_float_safe(basic_control.vel_x_linear) &&
            is_float_safe(basic_control.vel_y_linear) &&
            is_float_safe(basic_control.vel_z_angular) &&
            is_float_safe(basic_control.kick_vel) &&
            is_float_safe(basic_control.dribbler_speed)
        },
        DataPacket::ParameterCommand(parameter_command) => {
            match parameter_command.data_format {
                ParameterDataFormat::F32 => {
                    let float = unsafe { parameter_command.data.f32_ };
                    return is_float_safe(float);
                },
                ParameterDataFormat::MATRIX_F32 => {
                    let arr = unsafe { parameter_command.data.matrix_f32 };
                    return is_float_array_safe(&arr);
                },
                ParameterDataFormat::PID_F32 => {
                    let arr = unsafe { parameter_command.data.pid_f32 };
                    return is_float_array_safe(&arr);
                },
                ParameterDataFormat::PID_LIMITED_INTEGRAL_F32 => {
                    let arr = unsafe { parameter_command.data.pidii_f32 };
                    return is_float_array_safe(&arr);
                },
                ParameterDataFormat::VEC3_F32 => {
                    let arr = unsafe { parameter_command.data.vec3_f32 };
                    return is_float_array_safe(&arr);
                },
                ParameterDataFormat::VEC4_F32 => {
                    let arr = unsafe { parameter_command.data.vec4_f32 };
                    return is_float_array_safe(&arr);
                },
                _ => {
                    defmt::error!("Parameter Command data packet has an unexpected data type");
                    return false
                },
            }
        },
    }
}

pub fn create_error_telemetry_from_string(error_message: &str) -> ErrorTelemetry {
    let mut error_telemetry = ErrorTelemetry::default();
    let bytes = error_message.as_bytes();
    let copy_len = core::cmp::min(bytes.len(), 60);
    error_telemetry.error_message[..copy_len].copy_from_slice(&bytes[..copy_len]);
    error_telemetry.timestamp = Instant::now().as_millis() as u32;  // Overflows after ~50 days of uptime
    error_telemetry
}