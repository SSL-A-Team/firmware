#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(sync_unsafe_cell)]

use embassy_stm32::{bind_interrupts, peripherals, usart};

pub mod config;
pub mod pins;
pub mod tasks;
pub mod power_state;

bind_interrupts!(pub struct SystemIrqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});