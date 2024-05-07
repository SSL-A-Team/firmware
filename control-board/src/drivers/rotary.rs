use embassy_stm32::gpio::{Input, Pin, Pull};

pub struct Rotary<'a> {
    pin0: Input<'a>,
    pin1: Input<'a>,
    pin2: Input<'a>,
    pin3: Input<'a>,
}

impl<'a> Rotary<'a> {
    pub fn new(pin0: impl Pin, pin1: impl Pin, pin2: impl Pin, pin3: impl Pin) -> Self {
        let pin0 = Input::new(pin0, Pull::Down);
        let pin1 = Input::new(pin1, Pull::Down);
        let pin2 = Input::new(pin2, Pull::Down);
        let pin3 = Input::new(pin3, Pull::Down);

        Self {
            pin0,
            pin1,
            pin2,
            pin3,
        }
    }

    pub fn read(&self) -> u8 {
        (self.pin3.is_high() as u8) << 3
            | (self.pin2.is_high() as u8) << 2
            | (self.pin1.is_high() as u8) << 1
            | (self.pin0.is_high() as u8)
    }
}
