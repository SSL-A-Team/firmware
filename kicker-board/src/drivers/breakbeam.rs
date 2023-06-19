use embassy_stm32::gpio::{Output, Input, Level, Speed, Pin, Pull};

pub struct Breakbeam<'a, PinTx: Pin, PinRx: Pin> {
    pin_tx: Output<'a, PinTx>,
    pin_rx: Input<'a, PinRx>,
}

impl<'a, PinTx: Pin, PinRx: Pin> Breakbeam<'a, PinTx, PinRx> {
    pub fn new(pin_tx: PinTx, pin_rx: PinRx) -> Self {
        let pin_tx = Output::new(pin_tx, Level::High, Speed::Low);
        let pin_rx = Input::new(pin_rx, Pull::Down);
        Self {
            pin_tx,
            pin_rx
        }
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
        self.pin_rx.is_high()
    }
}
