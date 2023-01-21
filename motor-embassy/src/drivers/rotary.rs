use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};

pub struct Rotary<'a, Pin0: Pin, Pin1: Pin, Pin2: Pin, Pin3: Pin> {
    pin0: Input<'a, Pin0>,
    pin1: Input<'a, Pin1>,
    pin2: Input<'a, Pin2>,
    pin3: Input<'a, Pin3>,
}

impl<'a, Pin0: Pin, Pin1: Pin, Pin2: Pin, Pin3: Pin> Rotary<'a, Pin0, Pin1, Pin2, Pin3> {
    pub fn new(pin0: Pin0, pin1: Pin1, pin2: Pin2, pin3: Pin3) -> Self {
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
        (self.pin0.is_high() as u8) << 3
            | (self.pin1.is_high() as u8) << 2
            | (self.pin2.is_high() as u8) << 1
            | (self.pin3.is_high() as u8)
    }
}
