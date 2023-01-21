use embassy_stm32::gpio::{Level, Output, Pin, Speed};

pub struct ShellIndicator<'a, Pin0: Pin, Pin1: Pin, Pin2: Pin, Pin3: Pin> {
    pin0: Output<'a, Pin0>,
    pin1: Output<'a, Pin1>,
    pin2: Output<'a, Pin2>,
    pin3: Output<'a, Pin3>,
}

impl<'a, Pin0: Pin, Pin1: Pin, Pin2: Pin, Pin3: Pin> ShellIndicator<'a, Pin0, Pin1, Pin2, Pin3> {
    // MSB to LSB, LSB "0" start the quadrant C (upper right to lower right 0-3), 0 = pink, 1 = green
    #[rustfmt::skip]
    const ID_TO_PATTERN: [u8; 16] = [
        0x04, 0x06, 0x07, 0x05,
        0x08, 0x0A, 0x0B, 0x09,
        0x0F, 0x00, 0x0C, 0x03,
        0x0E, 0x02, 0x0D, 0x01,
    ];

    pub fn new(pin0: Pin0, pin1: Pin1, pin2: Pin2, pin3: Pin3) -> Self {
        Self {
            pin0: Output::new(pin0, Level::Low, Speed::Low),
            pin1: Output::new(pin1, Level::Low, Speed::Low),
            pin2: Output::new(pin2, Level::Low, Speed::Low),
            pin3: Output::new(pin3, Level::Low, Speed::Low),
        }
    }

    pub fn set(&mut self, robot_id: u8) {
        let shell_bits = Self::ID_TO_PATTERN[robot_id as usize];
        self.pin0.set_level((shell_bits & 0x01 != 0).into());
        self.pin1.set_level((shell_bits & 0x02 != 0).into());
        self.pin2.set_level((shell_bits & 0x04 != 0).into());
        self.pin3.set_level((shell_bits & 0x08 != 0).into());
    }
}
