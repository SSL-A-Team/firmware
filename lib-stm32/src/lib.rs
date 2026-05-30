#![no_std]
#![allow(incomplete_features)]
#![allow(clippy::too_many_arguments)]
// too many functions passing pins to device drivers exceed the bound

// if "strict" feature is on, promote warnings to errors
#![cfg_attr(feature = "strict", deny(warnings))]
#![feature(generic_const_exprs)]
#![feature(const_precise_live_drops)]
#![feature(sync_unsafe_cell)]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(maybe_uninit_slice)]
#![feature(ptr_metadata)]
#![feature(stmt_expr_attributes)]
#![feature(cfg_target_has_atomic_equal_alignment)]
#![feature(trait_alias)]
#![feature(never_type)]

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

// required for exported uart queue macros
pub extern crate paste;
#[doc(hidden)]
pub use ateam_lib_crossarch;
