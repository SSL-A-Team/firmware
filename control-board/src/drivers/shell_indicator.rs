use embassy_stm32::gpio::{Level, Output, Pin, Speed};

pub struct ShellIndicator<'a> {
    fr_pin0: Output<'a>,
    fl_pin1: Output<'a>,
    bl_pin2: Output<'a>,
    br_pin3: Output<'a>,
    team_pin4: Option<Output<'a>>,
}

impl<'a> ShellIndicator<'a> {
    // MSB to LSB, LSB "0" start the quadrant C (upper right to lower right 0-3), 0 = pink, 1 = green
    #[rustfmt::skip]
    const ID_TO_PATTERN: [u8; 16] = [
        0x04, 0x06, 0x07, 0x05,
        0x08, 0x0A, 0x0B, 0x09,
        0x0F, 0x00, 0x0C, 0x03,
        0x0E, 0x02, 0x0D, 0x01,
    ];

    // TODO: refactor pin ordering
    pub fn new(fr_pin0: impl Pin, fl_pin1: impl Pin, br_pin2: impl Pin, bl_pin3: impl Pin, team_pin4: Option<impl Pin>) -> Self {
        let team_pin4: Option<Output<'a>> = if let Some(pin4) = team_pin4 {
            Some(Output::new(pin4, Level::Low, Speed::Low))
        } else {
            None
        };

        Self {
            fr_pin0: Output::new(fr_pin0, Level::Low, Speed::Low),
            fl_pin1: Output::new(fl_pin1, Level::Low, Speed::Low),
            bl_pin2: Output::new(br_pin2, Level::Low, Speed::Low),
            br_pin3: Output::new(bl_pin3, Level::Low, Speed::Low),
            team_pin4: team_pin4,
        }
    }

    pub fn set(&mut self, robot_id: u8, team_is_blue: bool) {
        let shell_bits = Self::ID_TO_PATTERN[robot_id as usize];
        self.fr_pin0.set_level((shell_bits & 0x01 == 0).into());
        self.fl_pin1.set_level((shell_bits & 0x02 == 0).into());
        self.bl_pin2.set_level((shell_bits & 0x04 == 0).into());
        self.br_pin3.set_level((shell_bits & 0x08 == 0).into());
        if self.team_pin4.is_some() {
            if team_is_blue {
                self.team_pin4.as_mut().unwrap().set_high();
            } else {
                self.team_pin4.as_mut().unwrap().set_low();
            }
        }
    }
}
