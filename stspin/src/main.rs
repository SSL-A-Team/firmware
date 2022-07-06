#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f0xx_hal::{pac, prelude::*, serial::Serial};
use nb::block;

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.configure().sysclk(8.mhz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiof = dp.GPIOF.split(&mut rcc);

    let (mut led, tx, rx) = cortex_m::interrupt::free(|cs| {
        (
            gpiof.pf1.into_push_pull_output(cs),
            gpioa.pa14.into_alternate_af1(cs),
            gpioa.pa15.into_alternate_af1(cs),
        )
    });

    let mut serial = Serial::usart1(dp.USART1, (tx, rx), 115_200.bps(), &mut rcc);

    loop {
        if let Ok(b) = block!(serial.read()) {
            block!(serial.write(b+1)).ok();
        }

        for _ in 0..1_000_00 {
            led.set_high().ok();
        }
        for _ in 0..1_000_00 {
            led.set_low().ok();
        }
    }
}

