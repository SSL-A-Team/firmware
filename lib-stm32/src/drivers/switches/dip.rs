use core::{cmp::min, ops::Range};

use embassy_stm32::gpio::{AnyPin, Input, Pull};



pub struct DipSwitch<'a, const PIN_CT: usize> {
    inputs: [Input<'a>; PIN_CT],
    inversion_map: [bool; PIN_CT]
}

impl<'a, const PIN_CT: usize> DipSwitch<'a, PIN_CT> {
    pub fn new_from_pins(pins: [AnyPin; PIN_CT], pull: Pull, inversion_map: Option<[bool; PIN_CT]>) -> DipSwitch<'a, PIN_CT> {
        let inputs = pins.map(|pin| Input::new(pin, pull));

        let inversion_map = if let Some(map) = inversion_map {
            map
        } else { 
            [false; PIN_CT]
        };

        DipSwitch {
            inputs,
            inversion_map
        }
    }

    pub fn read_pin(&self, ind: usize) -> bool {
        let ind = min(ind, PIN_CT);

        if self.inversion_map[ind] {
            self.inputs[ind].is_low()
        } else {
            self.inputs[ind].is_high()
        }
    }

    pub fn read_block(r: Range<i32>) {

    }
}