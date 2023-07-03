#![cfg_attr(not(test), no_std)]
#![feature(type_alias_impl_trait)]
#![feature(associated_type_defaults)]
#![feature(const_cmp)]
#![feature(ptr_metadata)]
#![feature(async_fn_in_trait)]
#![feature(impl_trait_projections)]
#![feature(maybe_uninit_slice)]
#![feature(async_closure)]
#![feature(maybe_uninit_uninit_array)]
#![feature(const_maybe_uninit_uninit_array)]
#![feature(const_mut_refs)]

pub(crate) mod fmt;

pub mod transfer;
pub mod radio;
pub mod task;
pub mod queue;

pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
