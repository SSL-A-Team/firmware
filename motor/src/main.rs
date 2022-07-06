#![no_std]
#![no_main]

// use alloc::vec::Vec;
use cortex_m_rt::entry;
use panic_halt as _;
use motor::*;

#[entry]
fn main() -> ! {
    let peripherals = peripherals::Peripherals::init();

    // peripherals.stspin_fr.start_program(peripherals.streams_dma1.0);
    // peripherals.stspin_fl.start_program(peripherals.streams_dma1.1);

    loop { }
}
