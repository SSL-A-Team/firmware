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

pub mod queue;
pub mod stm32_interface;
pub mod uart_queue;

// pub use uart_queue::*;