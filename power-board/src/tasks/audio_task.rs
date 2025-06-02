use ateam_lib_stm32::{audio::tone_player::{self, TonePlayer}, drivers::audio::buzzer::Buzzer};

use embassy_stm32::{timer::{simple_pwm::{PwmPin, SimplePwm}, Channel},
                    gpio::OutputType, time::hz};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

use crate::pins::*;

#[macro_export]
macro_rules! create_audio_task {
    ($spawner:ident, $p:ident) => {
        ateam_power_board::tasks::audio_task::audio_task_entry(&$spawner);
    };
}

#[embassy_executor::task]
async fn audio_task_entry(
    mut tone_player: TonePlayer<'static, Buzzer<'static, BuzzerTimer>>,
    mut audio_subscriber: AudioSubscriber,
) {
    defmt::warn!("audio task unimplemented");
    let mut loop_rate_ticker = Ticker::every(Duration::from_millis(10));

    loop {
        // async await AudioCommand from a Subscriber

        // play song

        loop_rate_ticker.next().await;
    }
}

pub async fn start_power_task(
        spawner: &Spawner, 
        audio_subscriber: AudioSubscriber,
        buzzer_pin: BuzzerPwmPin,
        buzzer_timer: BuzzerTimer
    ) {
    let ch1 = PwmPin::new_ch1(buzzer_pin, OutputType::PushPull);
    let pwm = SimplePwm::new(buzzer_timer, Some(ch1), None, None, None, hz(1), Default::default());

    // init tone player
    let audio_driver = Buzzer::new(pwm, Channel::Ch1);
    let tone_player = TonePlayer::new(audio_driver);

    spawner.spawn(audio_task_entry(
        audio_subscriber
    )).expect("failed to spawn audio task");
}