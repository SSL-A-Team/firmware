#![no_std]
#![no_main]

use embassy_stm32 as _;
use panic_probe as _;
use defmt_rtt as _;

#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}