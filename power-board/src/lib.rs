#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(sync_unsafe_cell)]
#![feature(impl_trait_in_assoc_type)]

use embassy_stm32::{bind_interrupts, peripherals, usart};

pub mod config;
pub mod pins;
pub mod power_state;
pub mod songs;
pub mod tasks;

const DEBUG_UART_QUEUES: bool = false;

bind_interrupts!(pub struct SystemIrqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2_3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>, embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CH4_5_DMAMUX1_OVR => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH4>, embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH5>;
});
