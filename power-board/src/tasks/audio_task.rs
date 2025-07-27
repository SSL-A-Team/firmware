use ateam_lib_stm32::{audio::{songs::SongId, tone_player::TonePlayer, AudioCommand}, drivers::audio::buzzer::Buzzer};

use embassy_stm32::{timer::{simple_pwm::{PwmPin, SimplePwm}, Channel},
                    gpio::OutputType, time::hz};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker, Timer};

use crate::{pins::*, songs::{BALANCE_CONNECTED_SONG, BALANCE_DISCONNECTED_SONG, BATTERY_CRITICAL_SONG, BATTERY_LOW_SONG, BATTERY_UH_OH_SONG, POWER_OFF_SONG, POWER_ON_SONG, SHUTDOWN_REQUESTED_SONG, TEST_SONG}};

#[macro_export]
macro_rules! create_audio_task {
    ($spawner:ident, $audio_subscriber: ident, $p:ident) => {
        ateam_power_board::tasks::audio_task::start_audio_task(
            &$spawner, $audio_subscriber, $p.PC6, $p.TIM3
        );
    };
}

#[embassy_executor::task]
async fn audio_task_entry(
    mut tone_player: TonePlayer<'static, Buzzer<'static, BuzzerTimer>>,
    mut audio_subscriber: AudioSubscriber,
) {
    let mut loop_rate_ticker = Ticker::every(Duration::from_millis(200));

    loop {
        // async await AudioCommand from a Subscriber
        if let Some(audio_command) = audio_subscriber.try_next_message_pure() {
            // play song
            match audio_command {
                AudioCommand::Stop => defmt::warn!("Stop audio not implemented..."),
                AudioCommand::PlaySong(SongId::Test) => {
                    defmt::warn!("test song");
                    let _ = tone_player.load_song(&TEST_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::PowerOn) => {
                    defmt::warn!("power on song");
                    let _ = tone_player.load_song(&POWER_ON_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::PowerOff) => {
                    defmt::warn!("power off song");
                    let _ = tone_player.load_song(&POWER_OFF_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::ShutdownRequested) => {
                    defmt::warn!("shutdown requested song");
                    let _ = tone_player.load_song(&SHUTDOWN_REQUESTED_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::BatteryLow) => {
                    defmt::warn!("battery low song");
                    let _ = tone_player.load_song(&BATTERY_LOW_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::BatteryCritical) => {
                    defmt::warn!("battery critical song");
                    let _ = tone_player.load_song(&BATTERY_CRITICAL_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::BatteryUhOh) => {
                    defmt::warn!("battery uh oh!!!!!");
                    let _ = tone_player.load_song(&BATTERY_UH_OH_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::BalanceConnected) => {
                    defmt::warn!("balance connected song");
                    let _ = tone_player.load_song(&BALANCE_CONNECTED_SONG);
                    tone_player.play_song().await;
                },
                AudioCommand::PlaySong(SongId::BalanceDisconnected) => {
                    defmt::warn!("balance disconnected song");
                    let _ = tone_player.load_song(&BALANCE_DISCONNECTED_SONG);
                    tone_player.play_song().await;
                },
            }

            Timer::after_millis(500).await;
        }

        loop_rate_ticker.next().await;
    }
}

pub fn start_audio_task(
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
        tone_player,
        audio_subscriber
    )).expect("failed to spawn audio task");
}