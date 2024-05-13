
use embassy_stm32::gpio::Input;

pub struct RotaryEncoder<'a, const PIN_CT: usize> {
    pins: &'a [Input<'a>; PIN_CT],
    inversion_map: [bool; PIN_CT],
}

impl<'a, const PIN_CT: usize> RotaryEncoder<'a, PIN_CT> {
    pub const fn new_from_inputs(inputs: &'a [Input<'a>; PIN_CT], inversion_map: Option<[bool; PIN_CT]>) -> RotaryEncoder<'a, PIN_CT> {
        let inversion_map = if let Some(map) = inversion_map {
            map
        } else { 
            [false; PIN_CT]
        };

        RotaryEncoder {
            pins: inputs,
            inversion_map,
        }
    }

    pub fn read_value(&self) -> u8 {
        let mut val: u8 = 0;
        for i in 0..self.pins.len() {
            let pin_state = if self.inversion_map[i] {
                self.pins[i].is_low()
            } else {
                self.pins[i].is_high()
            };

            val |= (pin_state as u8 & 0x1) << i
        }

        return val;
    }
}