use embassy_stm32::gpio::{Input, Output, OutputType, Pin, Pull};
use embassy_stm32::timer::{
    GeneralInstance4Channel,
    Channel,
    simple_pwm::{
        PwmPin,
        SimplePwm}};
use embassy_stm32::time::khz;
use embassy_stm32::Peripheral;
use embassy_time::{Duration, Timer};

pub struct Breakbeam<'a, Timer: GeneralInstance4Channel> {
    pin_tx: SimplePwm<'a, Timer>,
    pin_rx: Input<'a>,
}

impl<'a, Timer: GeneralInstance4Channel> Breakbeam<'a, Timer> {
    pub fn new(pin_tx: PwmPin<'a, Timer, Channel>, pin_rx: impl Peripheral<P = impl Pin> + 'a) -> Self {
        let ch_1 = PwmPin::new_ch1(pin_tx, OutputType::PushPull);
        let mut pwm_tx = SimplePwm::new(pin_tx, Level::High, Speed::Low);
        
        let pin_rx = Input::new(pin_rx, Pull::Down);

        Self {
            pin_tx: pwm_tx,
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
