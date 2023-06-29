use core::marker::PhantomData;
use core::sync::atomic::AtomicU32;

use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::pwm::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::pwm::{Channel as PwmChannel, CaptureCompare16bitInstance, Channel1Pin, Channel2Pin, Channel3Pin, Channel4Pin};
use embassy_stm32::time::khz;
use embassy_stm32::Peripheral;
use embassy_time::{Duration, Timer};
use embassy_stm32::interrupt::Interrupt;
use embassy_executor::{raw::TaskStorage, SpawnToken};


pub struct BreakbeamPwm<'a,
    PinTx: Pin, 
    PwmTx: CaptureCompare16bitInstance, 
    PinRx: Pin> 
{
    phantom: PhantomData<(PinTx, PinRx)>,
    pwm_tx: SimplePwm<'a, PwmTx>,
    pwm_ch_tx: PwmChannel,
    pwm_max_duty: u16
}


impl<'a,
    PwmTx: CaptureCompare16bitInstance,
    PinTx: Pin + Channel1Pin<PwmTx>,
    PinRx: Pin>
    BreakbeamPwm<'a, PinTx, PwmTx, PinRx> 
{
    pub fn new_ch1(            
        pin_tx: PinTx, 
        pwm_ch_tx: PwmChannel, 
        timer_tx:  impl Peripheral<P = PwmTx> + 'a, 
        freq_tx: u32, 
        pin_rx: PinRx
    ) -> Self 
    {
        let pwm_ch_tx = pwm_ch_tx;
        let pwm_tx = SimplePwm::new(
            timer_tx, 
            Some(PwmPin::new_ch1(pin_tx)), 
            None, 
            None, 
            None, 
            khz(freq_tx));
        
        let pin_rx = Input::new(pin_rx, Pull::Down);    
        let pwm_max_duty = pwm_tx.get_max_duty();

        Self 
        {
            phantom: PhantomData,
            pwm_tx,
            pwm_ch_tx,
            pwm_max_duty
        }
    }
}

impl<'a,
    PwmTx: CaptureCompare16bitInstance,
    PinTx: Pin + Channel2Pin<PwmTx>,
    PinRx: Pin>
    BreakbeamPwm<'a, PinTx, PwmTx, PinRx> 
{
    pub fn new_ch2(            
        pin_tx: PinTx, 
        pwm_ch_tx: PwmChannel, 
        timer_tx:  impl Peripheral<P = PwmTx> + 'a, 
        freq_tx: u32, 
        pin_rx: PinRx
    ) -> Self 
    {
        let pwm_ch_tx = pwm_ch_tx;
        let pwm_tx = SimplePwm::new(
            timer_tx, 
            None, 
            Some(PwmPin::new_ch2(pin_tx)), 
            None, 
            None, 
            khz(freq_tx));
        
        let pin_rx = Input::new(pin_rx, Pull::Down);

        let pwm_max_duty = pwm_tx.get_max_duty();

        Self 
        {
            phantom: PhantomData,
            pwm_tx,
            pwm_ch_tx,
            pwm_max_duty
        }
    }
}


impl<'a,
    PwmTx: CaptureCompare16bitInstance,
    PinTx: Pin + Channel3Pin<PwmTx>,
    PinRx: Pin>
    BreakbeamPwm<'a, PinTx, PwmTx, PinRx> 
{
    pub fn new_ch3(            
        pin_tx: PinTx, 
        pwm_ch_tx: PwmChannel, 
        timer_tx:  impl Peripheral<P = PwmTx> + 'a, 
        freq_tx: u32, 
        pin_rx: PinRx
    ) -> Self 
    {
        let pwm_ch_tx = pwm_ch_tx;
        let mut pwm_tx = SimplePwm::new(
            timer_tx, 
            None, 
            None, 
            Some(PwmPin::new_ch3(pin_tx)), 
            None, 
            khz(freq_tx));
        
        pwm_tx.enable(pwm_ch_tx);

        let pin_rx = Input::new(pin_rx, Pull::Down);

// TODO Finish
        //irq.set_handler(BreakbeamPwm::<PwmTx, PinTx, PinRx, IrqRx>::on_interrupt);
        //irq.unpend();
        //irq.enable();

        let pwm_max_duty = pwm_tx.get_max_duty();

        Self 
        {
            phantom: PhantomData,
            pwm_tx,
            pwm_ch_tx,
            pwm_max_duty
        }
        
    }

    #[inline]
    pub fn enable_tx(&mut self)
    {
        self.pwm_tx.set_duty(self.pwm_ch_tx, self.pwm_max_duty/2);
    }

    #[inline]
    pub fn disable_tx(&mut self)
    {
        self.pwm_tx.set_duty(self.pwm_ch_tx, 0);
    }

    pub fn read(&self) -> bool
    {
        true
    }
}

impl<'a,
    PwmTx: CaptureCompare16bitInstance,
    PinTx: Pin + Channel4Pin<PwmTx>,
    PinRx: Pin>
    BreakbeamPwm<'a, PinTx, PwmTx, PinRx> 
{
    pub fn new_ch4(            
        pin_tx: PinTx, 
        pwm_ch_tx: PwmChannel, 
        timer_tx:  impl Peripheral<P = PwmTx> + 'a, 
        freq_tx: u32, 
        pin_rx: PinRx
    ) -> Self 
    {
        let pwm_ch_tx = pwm_ch_tx;
        let pwm_tx = SimplePwm::new(
            timer_tx, 
            None, 
            None, 
            None, 
            Some(PwmPin::new_ch4(pin_tx)), 
            khz(freq_tx));
        
        let pin_rx = Input::new(pin_rx, Pull::Down);

        let pwm_max_duty = pwm_tx.get_max_duty();

        Self 
        {
            phantom: PhantomData,
            pwm_tx,
            pwm_ch_tx,
            pwm_max_duty
        }
        
    }
}

//breakbeam_ch_impl!(Some(PwmPin::new_ch1(pin_tx)), None, None, None, Channel1Pin, new_breakbeam_ch1);
//breakbeam_ch_impl!(new_ch2, Channel2Pin, new_breakbeam_ch2);
//breakbeam_ch_impl!(new_ch3, Channel3Pin, new_breakbeam_ch3);
//breakbeam_ch_impl!(new_ch4, Channel4Pin, new_breakbeam_ch4);




//impl<'a,
//    PinTx: Pin + Channel3Pin<PwmTx>,
//    PwmTx: CaptureCompare16bitInstance, 
//    PinRx: Pin> 
//    BreakbeamPwm<'a, PinTx, PwmTx, PinRx> {
//
//        pub fn new(
//            pin_tx: impl Peripheral<P = impl Channel3Pin<PwmTx>> + 'a, 
//            pwm_ch_tx: Channel, 
//            timer_tx:  impl Peripheral<P = PwmTx> + 'a, 
//            freq_tx: u32, 
//            pin_rx: PinRx
//        ) -> Self 
//        {
//            let pwm_ch_tx = pwm_ch_tx;
//            let pwm_tx = match pwm_ch_tx {
//                Channel::Ch1 => 
//                {
//                    let ch3 = PwmPin::new_ch3(pin_tx);
//                    SimplePwm::new(timer_tx, None, None, Some(ch3), None, khz(freq_tx))
//                },
//                //Channel::Ch2 => SimplePwm::new(timer_tx, None, Some(PwmPin::new_ch2(pin_tx)), None, None, khz(freq_tx)),
//                //Channel::Ch3 => SimplePwm::new(timer_tx, None, None, Some(PwmPin::new_ch3(pin_tx)), None, khz(freq_tx)),
//                //Channel::Ch4 => SimplePwm::new(timer_tx, None, None, None, Some(PwmPin::new_ch4(pin_tx)), khz(freq_tx))
//            };
//            let pin_rx = Input::new(pin_rx, Pull::Down);
//            Self 
//            {
//                phantom: PhantomData,
//                pwm_tx,
//                pwm_ch_tx,
//                pin_rx
//            }
//        }
//
//    //#[inline]
//    //pub fn enable_tx(&mut self) {
//    //    self.pin_tx.set_high();
//    //}
//    //
//    //#[inline]
//    //pub fn disable_tx(&mut self) {
//    //    self.set_duty(Channel::Ch3, 0);
//    //}
//    //
//    //#[inline]
//    //pub fn read(&self) -> bool {
//    //    self.pin_rx.is_high()
//    //}
//}
