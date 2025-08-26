use embassy_stm32::{
    gpio::{AnyPin, Input, Level, Output, Pull, Speed},
    Peri,
};

pub struct Breakbeam<'a> {
    pin_tx: Output<'a>,
    pin_rx: Input<'a>,
}

impl<'a> Breakbeam<'a> {
    pub fn new(pin_tx: Peri<'a, AnyPin>, pin_rx: Peri<'a, AnyPin>) -> Self {
        let pin_tx = Output::new(pin_tx, Level::High, Speed::Low);
        let pin_rx = Input::new(pin_rx, Pull::Down);
        Self { pin_tx, pin_rx }
    }

    #[inline]
    pub fn enable_tx(&mut self) {
        self.pin_tx.set_high();
    }

    #[inline]
    pub fn disable_tx(&mut self) {
        self.pin_tx.set_low();
    }

    #[inline]
    pub fn read(&self) -> bool {
        self.pin_rx.is_low()
    }
}
