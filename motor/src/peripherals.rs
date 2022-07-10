use stm32h7xx_hal::{
    dma::dma::StreamsTuple,
    gpio::{Output, Pin},
    pac,
    prelude::*,
    rcc,
};
use stspin::STSpin;

mod stspin;
// mod trait_ext;
pub mod stm32_bootloader;
pub mod timeout;

pub struct Peripherals {
    pub stspin_fl: STSpin<pac::USART1, Pin<'A', 11, Output>, Pin<'A', 12, Output>>,
    // pub stspin_fr: STSpin<pac::USART2>,
    // pub stspin_bl: STSpin<pac::USART3>,
    // pub stspin_br: STSpin<pac::UART4>,
    pub streams_dma1: StreamsTuple<pac::DMA1>,
    pub streams_dma2: StreamsTuple<pac::DMA2>,
}

impl Peripherals {
    pub fn init() -> Peripherals {
        let dp = pac::Peripherals::take().unwrap();
        let _cp = cortex_m::peripheral::Peripherals::take().unwrap();

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.MHz())
            .pll1_q_ck(200.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);

        // unsafe { CLOCKS = Some(ccdr.clocks) };

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

        // dp.USART2.cr1

        let stspin_fl = STSpin::init(
            dp.USART1,
            (gpioa.pa9.into_alternate(), gpioa.pa10.into_alternate()),
            ccdr.peripheral.USART1,
            &ccdr.clocks,
            gpioa.pa11.into_push_pull_output(),
            gpioa.pa12.into_push_pull_output(),
        );
        // let stspin_fr = STSpin::init(
        //     dp.USART2,
        //     (gpioa.pa2.into_alternate(), gpioa.pa3.into_alternate()),
        //     ccdr.peripheral.USART2,
        //     &ccdr.clocks,
        //     gpioa.pa4,
        //     gpioa.pa5,
        // );
        // let stspin_bl = STSpin::init(
        //     dp.USART3,
        //     (gpioc.pc10.into_alternate(), gpioc.pc11.into_alternate()),
        //     ccdr.peripheral.USART3,
        //     &ccdr.clocks,
        //     gpioc.pc12,
        //     gpioc.pc13,
        // );
        // let stspin_br = STSpin::init(
        //     dp.UART4,
        //     (gpioa.pa12.into_alternate(), gpioa.pa11.into_alternate()),
        //     ccdr.peripheral.UART4,
        //     &ccdr.clocks,
        //     gpioa.pa13,
        //     gpioa.pa14,
        // );

        let streams_dma1 = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);
        let streams_dma2 = StreamsTuple::new(dp.DMA2, ccdr.peripheral.DMA2);

        Peripherals {
            stspin_fl,
            // stspin_fr,
            // stspin_bl,
            // stspin_br,
            streams_dma1,
            streams_dma2,
        }
    }
}
