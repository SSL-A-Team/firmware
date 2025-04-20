#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use ateam_power_board::create_power_task;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{adc::{Adc, AdcChannel, SampleTime},
    gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed, OutputType},
    rcc::Hse, timer::{low_level::CountingMode, simple_pwm::{SimplePwm, PwmPin}, Channel}, 
    Config,
    time::hz};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use ateam_lib_stm32::{drivers::{led::apa102::Apa102,
    audio::{buzzer::Buzzer}}, audio::{note::Beat, tone_player::TonePlayer}};
use smart_leds::colors::{BLUE, WHITE, BLACK};

static mut DOTSTAR_SPI_BUFFER_CELL: [u8; 16] = [0; 16];

pub const TEST_SONG: [Beat; 2] = [
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 587, duration: 250_000 },
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    
    let mut en_12v0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut en_3v3 = Output::new(p.PB7, Level::Low, Speed::Low);
    let mut en_5v0 = Output::new(p.PB8, Level::Low, Speed::Low);

    Timer::after_millis(50).await;
    en_3v3.set_high();
    en_5v0.set_high();
    en_12v0.set_high();

    
    create_power_task!(spawner, p);



    loop {
        Timer::after_millis(1000).await;
    }
}
