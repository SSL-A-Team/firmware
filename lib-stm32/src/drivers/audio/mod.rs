pub mod buzzer;

pub trait PlayTone {
    fn play_tone(&mut self, tone: u16);
    fn can_play_tone(&self, tone: u16) -> bool;
}
