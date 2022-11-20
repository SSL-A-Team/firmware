#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(
    maybe_uninit_uninit_array,
    const_maybe_uninit_uninit_array,
    maybe_uninit_slice,
    maybe_uninit_write_slice
)]
#![feature(const_mut_refs)]

pub mod fw_images;
pub mod queue;
pub mod stm32_interface;
pub mod uart_queue;

#[macro_export]
macro_rules! include_external_bin {
    ($var_name:ident, $bin_file:literal) => {
        static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../", $bin_file)).len()] 
            = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../", $bin_file));
    }
}