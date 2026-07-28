#![no_std]
#![allow(incomplete_features)]
#![allow(clippy::too_many_arguments)]
// too many functions passing pins to device drivers exceed the bound
#![allow(clippy::result_unit_err)]
// pre-existing driver convention; newer clippy (post toolchain bump) flags Result<_, ()> by default

// if "strict" feature is on, promote warnings to errors
#![cfg_attr(feature = "strict", deny(warnings))]
#![feature(generic_const_exprs)]
#![feature(const_precise_live_drops)]
#![feature(type_alias_impl_trait)]
#![feature(ptr_metadata)]

pub mod anim;
pub mod audio;
pub mod drivers;
pub mod filter;
pub mod math;
pub mod model;
pub mod power;
pub mod time;
pub mod uart;
pub mod units;
pub mod util;

// required for exported uart queue macros
pub extern crate paste;
#[doc(hidden)]
pub use ateam_lib_crossarch;
