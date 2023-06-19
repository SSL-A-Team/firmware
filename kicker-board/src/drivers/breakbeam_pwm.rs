use embassy_stm32::gpio::{Output, Input, Pin, Pull};
use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::pwm::Channel;
use embassy_stm32::time::khz;
use embassy_time::{Duration, Timer};

pub struct Breakbeam<'a, PwmTx: Pin, PinRx: Pin> {
    pin_tx: SimplePwm<'a, PwmTx>,
    pin_rx: Input<'a, PinRx>,
}

impl<'a, PinTx: Pin, PinRx: Pin, Timer: Peripheral> Breakbeam<'a, PinTx, PinRx, > {
    pub fn new(pin_tx: PinTx, pin_rx: PinRx) -> Self {
        let ch_1 = PwmPin::new_ch1(pin_tx);
        let mut pwm_tx = SimplePwm::new(pin_tx, Level::High, Speed::Low);
        let pin_rx = Input::new(pin_rx, Pull::Down);
        Self {
            pwm_tx,
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
