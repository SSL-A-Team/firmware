use embedded_hal::digital::v2::OutputPin;
use embedded_sdmmc::Controller;
use stm32h7xx_hal::{
    gpio::{Output, Pin},
    pac, serial, sdmmc::{SdmmcBlockDevice, Sdmmc, SdCard},
};

pub struct STSpin<UART, BOOT: OutputPin, RST: OutputPin> {
    boot: BOOT,
    rst: RST,
    serial: serial::Serial<UART>,
}

pub struct Radio<USART> {
    serial: serial::Serial<USART>,
}

// This is just a placeholder TimeSource. In a real world application
// one would probably use the RTC to provide time.
pub struct TimeSource;

impl embedded_sdmmc::TimeSource for TimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct BodgeBot {
    motor_fl: STSpin<pac::UART7, Pin<'G', 0, Output>, Pin<'G', 1, Output>>,
    motor_fr: STSpin<pac::USART6, Pin<'G', 10, Output>, Pin<'G', 12, Output>>,
    motor_bl: STSpin<pac::UART8, Pin<'B', 8, Output>, Pin<'B', 9, Output>>,
    motor_br: STSpin<pac::USART2, Pin<'D', 4, Output>, Pin<'D', 7, Output>>,
    radio: Radio<pac::USART3>,

    led_g: Pin<'B', 0, Output>,
    led_r: Pin<'B', 14, Output>,
    led_y: Pin<'E', 1, Output>,

    sd_fatfs: Controller<SdmmcBlockDevice<Sdmmc<pac::SDMMC1, SdCard>>, TimeSource>,

    // usb
    // imu
    // acc
    // dotstar
    // gpio
    // dip 0-3
    // btn 0-1
    // batt adc
}
