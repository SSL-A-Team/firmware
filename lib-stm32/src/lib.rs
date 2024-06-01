#![no_std]
#![feature(generic_const_exprs)]
#![feature(maybe_uninit_uninit_array)]
#![feature(sync_unsafe_cell)]
#![feature(type_alias_impl_trait)]
#![feature(const_maybe_uninit_uninit_array)]
#![feature(maybe_uninit_slice)]
#![feature(ptr_metadata)]
#![feature(trait_alias)]

pub mod drivers;
pub mod math;
pub mod uart;

pub mod queue;

// required for exported queue macros
pub extern crate paste;

