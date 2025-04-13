#![no_std]

#![allow(incomplete_features)]
#![allow(clippy::too_many_arguments)]  // too many functions passing pins to device drivers exceed the bound

#![feature(generic_const_exprs)]
#![feature(const_precise_live_drops)]
#![feature(maybe_uninit_uninit_array)]
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
pub mod uart;
pub mod power;

pub mod queue;

// required for exported queue macros
pub extern crate paste;

