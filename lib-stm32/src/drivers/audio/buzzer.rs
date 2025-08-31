use embassy_stm32::{
    time::hz,
    timer::{
        simple_pwm::{SimplePwm, SimplePwmChannel},
        Channel, GeneralInstance4Channel,
    },
};
use num_traits::clamp;

use super::PlayTone;

pub const BUZZER_MIN_FREQ: u16 = 35;
pub const BUZZER_MAX_FREQ: u16 = 7000;

pub struct Buzzer<'d, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'d, T>,
    channel: Channel,

    min_freq: u16,
    max_freq: u16,
}

impl<'d, T: GeneralInstance4Channel> Buzzer<'d, T> {
    pub fn new(pwm: SimplePwm<'d, T>, channel: Channel) -> Self {
        Self::new_with_fcontrs(pwm, channel, BUZZER_MIN_FREQ, BUZZER_MAX_FREQ)
    }

    pub fn new_with_fcontrs(
        pwm: SimplePwm<'d, T>,
        channel: Channel,
        min_freq: u16,
        max_freq: u16,
    ) -> Self {
        Self {
            pwm,
            channel,
            min_freq,
            max_freq,
        }
    }

    fn get_channel(&mut self) -> SimplePwmChannel<'_, T> {
        match self.channel {
            Channel::Ch1 => self.pwm.ch1(),
            Channel::Ch2 => self.pwm.ch2(),
            Channel::Ch3 => self.pwm.ch3(),
            Channel::Ch4 => self.pwm.ch4(),
        }
    }

    pub fn init(&mut self) {
        let max_duty = self.pwm.max_duty_cycle();
        self.get_channel().set_duty_cycle(max_duty / 2);
        self.get_channel().enable();
    }
}

impl<T: GeneralInstance4Channel> PlayTone for Buzzer<'_, T> {
    fn play_tone(&mut self, freq: u16) {
        if freq == 0 {
            self.get_channel().disable();
        } else {
            let freq = clamp(freq, self.min_freq, self.max_freq);
            self.pwm.set_frequency(hz(freq.into()));
            let max_duty = self.pwm.max_duty_cycle();
            self.get_channel().set_duty_cycle(max_duty / 2);
            self.get_channel().enable();
        }
    }

    fn can_play_tone(&self, freq: u16) -> bool {
        self.min_freq < freq && freq < self.max_freq
    }
}
