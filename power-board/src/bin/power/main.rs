#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use ateam_power_board::{create_power_task, pins::AudioPubSub};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

use ateam_lib_stm32::audio::note::Beat;

pub const TEST_SONG: [Beat; 2] = [
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 587, duration: 250_000 },
];

static AUDIO_PUBSUB: AudioPubSub = PubSubChannel::new();

async fn sequence_power_on(mut en_3v3: Output<'static>, mut en_5v0: Output<'static>, mut en_12v0: Output<'static>) {
    Timer::after_millis(20).await;
    en_3v3.set_high();

    Timer::after_millis(10).await;
    en_5v0.set_high();

    Timer::after_millis(10).await;
    en_12v0.set_high();
}

async fn sequence_power_off(mut en_3v3: Output<'static>, mut en_5v0: Output<'static>, mut en_12v0: Output<'static>) {
    en_12v0.set_low();
    Timer::after_millis(100).await;

    en_5v0.set_low();
    Timer::after_millis(10).await;

    en_3v3.set_low();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("power board startup.");

    let _pwr_btn = Input::new(p.PB15, Pull::None);
    let mut _shutdown_ind = Output::new(p.PA15, Level::High, Speed::Low);
    let mut _kill_sig = OutputOpenDrain::new(p.PA8, Level::High, Speed::Low);

    let en_12v0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let en_3v3 = Output::new(p.PB7, Level::Low, Speed::Low);
    let en_5v0 = Output::new(p.PB8, Level::Low, Speed::Low);

    sequence_power_on(en_3v3, en_5v0, en_12v0).await;

    // Timer::after_millis(50).await;
    // en_3v3.set_high();
    // en_5v0.set_high();
    // en_12v0.set_high();
    
    create_power_task!(spawner, p);

    // TODO: start audio task

    // TODO: start LED animation task

    // TODO: start communications queues

    let mut main_loop_ticker = Ticker::every(Duration::from_secs(1));
    loop {
        // read packets
        // read channels

        // read pwr button

        // send packets

        main_loop_ticker.next().await;
    }
}
