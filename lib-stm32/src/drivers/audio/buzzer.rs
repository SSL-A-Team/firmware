use embassy_stm32::{time::hz, timer::{simple_pwm::SimplePwm, simple_pwm::SimplePwmChannel, GeneralInstance4Channel}};
use num_traits::clamp;

use super::PlayTone;

pub const BUZZER_MIN_FREQ: u16 = 35;
pub const BUZZER_MAX_FREQ: u16 = 7000;

pub struct Buzzer<'d, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'d, T>,
    channel: SimplePwmChannel<'d, T>,

    min_freq: u16,
    max_freq: u16,
}

impl<'d, T: GeneralInstance4Channel> Buzzer<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, channel: SimplePwmChannel<'d, T>) -> Self {
        Buzzer {
            pwm: pwm,
            channel: channel,
            min_freq: BUZZER_MIN_FREQ,
            max_freq: BUZZER_MAX_FREQ,
        }
    }

    pub fn new_with_fcontrs(pwm: SimplePwm<'d, T>, channel: SimplePwmChannel<'d, T>, min_freq: u16, max_freq: u16) -> Self {
        Buzzer {
            pwm: pwm,
            channel: channel,
            min_freq: min_freq,
            max_freq: max_freq,
        }
    }

    pub fn init(&mut self) {
        let max_duty = self.pwm.max_duty_cycle();
        self.channel.set_duty_cycle(max_duty / 2);
        self.channel.enable();
    }
}

impl<'d, T: GeneralInstance4Channel> PlayTone for Buzzer<'d, T> {
    fn play_tone(&mut self, freq: u16) {
        if freq == 0 {
            self.channel.disable();
        } else {
            let freq = clamp(freq, self.min_freq, self.max_freq);
            self.pwm.set_frequency(hz(freq.into()));
            let max_duty = self.pwm.max_duty_cycle();
            self.channel.set_duty_cycle(max_duty / 2);
            self.channel.enable();
        }
    }

    fn can_play_tone(&self, freq: u16) -> bool {
        self.min_freq < freq && freq < self.max_freq
    }
}