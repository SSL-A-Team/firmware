#![no_std]
#![no_main]

use cortex_m_rt::entry;

use cortex_m_semihosting::hprintln;
use panic_halt as _;
use motor::{peripherals::{stm32_bootloader, timeout::delay_us}, peripherals::timeout, include_external_bin};
use stm32h7xx_hal::{
    block, pac,
    prelude::*,
    serial::{self, config},
};

include_external_bin!{STSPIN_BINARY, "build/bin/dev3204-wheel.bin"}

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

    let mut led_g = gpiob.pb0.into_push_pull_output();

    let mut boot = gpiod.pd7.into_push_pull_output();
    let mut rst = gpiod.pd4.into_push_pull_output();
    let config = serial::config::Config::new(115_200.bps())
        .parity_even()
        .stopbits(config::StopBits::STOP0P5);
    let mut serial = dp
        .USART2
        .serial(
            (
                gpiod.pd5.into_alternate().internal_pull_up(true),
                gpiod.pd6.into_alternate().internal_pull_up(true),
            ),
            config,
            ccdr.peripheral.USART2,
            &ccdr.clocks,
        )
        .unwrap();

    boot.set_high();
    rst.set_high();
    timeout::delay_us(20000);

    let mut stspin = stm32_bootloader::StmBootloader::new(&mut serial).unwrap();
    // let version = stspin.get_version().unwrap();
    // let id = stspin.get_id().unwrap();
    stspin.erase_all().unwrap();
    stspin.write_flash(&STSPIN_BINARY[..]).unwrap();

    boot.set_low();
    rst.set_low();
    timeout::delay_us(100);
    rst.set_high();

    // hprintln!("{} - {}", version, id);
    led_g.set_high();

    let config = serial::config::Config::new(115_200.bps());
        // .parity_odd();
        // .stopbits(config::StopBits::STOP1);
    serial.reconfigure(config, false);

    timeout::delay_us(10000);

    let mut n = 0;
    loop {
        block!(serial.write(n)).unwrap();
        n = block!(serial.read()).unwrap();
        hprintln!("{}", n);
    }
}
