#![no_std]

#![feature(trait_alias)]

pub mod peripherals;
pub mod bodge_pinout;
pub mod drivers;

// TODO: put this in better place
// TODO: static file size check
#[macro_export]
macro_rules! include_bin {
    ($var_name:ident, $bin_file:literal) => {
        static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../bin/", $bin_file)).len()] 
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../bin/", $bin_file));
    }
}

#[macro_export]
macro_rules! include_external_bin {
    ($var_name:ident, $bin_file:literal) => {
        static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../", $bin_file)).len()] 
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../", $bin_file));
    }
}

/*
DIP:
    0 - PE0
    1 - PE1
    2 - PE2
    3 - PE3
    4 - PE4
    5 - PE5
BAT_MON - PC3
SPI_IMU:
    MOSI - PA7
    MISO - PA6
    SCK - PA5
    SS1 - PA4
    SS2 - PE11
SPI_DOTSTAR:
    MOSI - PB2
    
*/