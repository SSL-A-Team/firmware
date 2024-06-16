use ateam_lib_stm32::{audio::tone_player::TonePlayer, drivers::audio::buzzer::Buzzer};
use embassy_executor::Spawner;
use embassy_stm32::{gpio::OutputType, time::hz, timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}};
use embassy_time::{Duration, Ticker};

use crate::{pins::{BuzzerPin, BuzzerTimer}, robot_state::SharedRobotState, songs::TIPPED_WARNING_SONG};

#[macro_export]
macro_rules! create_audio_task {
    ($main_spawner:ident, $robot_state:ident, $p:ident) => {
        ateam_control_board::tasks::audio_task::start_audio_task(
            &$main_spawner, $robot_state, $p.TIM15, $p.PE6
        );
    };
}

#[embassy_executor::task]
async fn audio_task_entry(
    robot_state: &'static SharedRobotState,
    mut tone_player: TonePlayer<'static, Buzzer<'static, BuzzerTimer>>,
) {
    let mut audio_ticker = Ticker::every(Duration::from_millis(100));

    loop {
        let cur_robot_state = robot_state.get_state();

        if cur_robot_state.robot_tipped {
            defmt::warn!("robot tipped audio");
            let _ = tone_player.load_song(&TIPPED_WARNING_SONG);
            tone_player.play_song().await;
        }

        audio_ticker.next().await;
    }
}

pub fn start_audio_task(
    task_spawner: &Spawner,
    robot_state: &'static SharedRobotState,
    buzzer_timer: BuzzerTimer,
    buzzer_pin: BuzzerPin,
) {
    let ch2 = PwmPin::new_ch2(buzzer_pin, OutputType::PushPull);
    let pwm = SimplePwm::new(buzzer_timer, None, Some(ch2), None, None, hz(1), Default::default());
    
    let audio_driver = Buzzer::new(pwm, Channel::Ch2);
    let tone_player = TonePlayer::new(audio_driver);

    task_spawner.spawn(audio_task_entry(robot_state, tone_player)).unwrap();
}