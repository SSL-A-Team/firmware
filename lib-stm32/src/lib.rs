#![no_std]
#![feature(maybe_uninit_uninit_array)]
#![feature(sync_unsafe_cell)]
#![feature(type_alias_impl_trait)]
#![feature(const_maybe_uninit_uninit_array)]
#![feature(maybe_uninit_slice)]
#![feature(unsized_locals)]


pub mod drivers;
pub mod uart;

pub mod queue;

