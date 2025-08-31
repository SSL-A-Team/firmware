#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_stm32 as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
