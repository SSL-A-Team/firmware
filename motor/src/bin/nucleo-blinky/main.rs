#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use panic_halt as _;
use motor::{block_us, peripherals::timeout};
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);
    timeout::setup_timeout(cp.DWT, &ccdr.clocks);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let mut led_g = gpiob.pb0.into_push_pull_output();
    let mut led_r = gpiob.pb14.into_push_pull_output();
    let mut led_y = gpioe.pe1.into_push_pull_output();

    let mut serial = dp
        .USART2
        .serial(
            (gpiod.pd5.into_alternate(), gpiod.pd6.into_alternate()),
            115_200.bps(),
            ccdr.peripheral.USART2,
            &ccdr.clocks,
        )
        .unwrap();
    serial.write(0x8).unwrap();
    let val = block_us!(serial.read(), 1000).unwrap_or(0);

    // hprintln!("Value: {}", val);

    let mut delay = cp.SYST.delay(ccdr.clocks);
    loop {
        led_g.set_high();
        delay.delay_ms(100u32);
        led_g.set_low();
        led_y.set_high();
        delay.delay_ms(100u32);
        led_y.set_low();
        led_r.set_high();
        delay.delay_ms(100u32);
        led_r.set_low();
        led_y.set_high();
        delay.delay_ms(100u32);
        led_y.set_low();
    }
}
