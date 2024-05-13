use core::{cmp::min, ops::Range};

use embassy_stm32::gpio::{AnyPin, Input, Pull};



pub struct DipSwitch<'a, const PIN_CT: usize> {
    inputs: [Input<'a>; PIN_CT],
    inversion_map: [bool; PIN_CT]
}

impl<'a, const PIN_CT: usize> DipSwitch<'a, PIN_CT> {
    pub const fn new_from_inputs(inputs: [Input<'a>; PIN_CT], inversion_map: Option<[bool; PIN_CT]>) -> DipSwitch<'a, PIN_CT> {
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

    pub fn new_from_pins(pins: [AnyPin; PIN_CT], pull: Pull, inversion_map: Option<[bool; PIN_CT]>) -> DipSwitch<'a, PIN_CT> {
        let inputs = pins.map(|pin| Input::new(pin, pull));
        DipSwitch::new_from_inputs(inputs, inversion_map)
    }

    pub fn read_pin(&self, ind: usize) -> bool {
        let ind = min(ind, PIN_CT);

        if self.inversion_map[ind] {
            self.inputs[ind].is_low()
        } else {
            self.inputs[ind].is_high()
        }
    }

    pub fn read_block(&self, r: &mut Range<i32>) -> u8 {
        if !r.all(|val| 0 <= val && val < PIN_CT as i32) {
            return 0;
        }

        let mut val: u8 = 0;
        for i in r.enumerate() {
            val |= (self.read_pin(i.1 as usize) as u8 & 0x1) << i.0;
        }

        val
    }
}