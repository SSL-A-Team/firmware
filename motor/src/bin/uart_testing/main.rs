#![no_std]
#![no_main]

use cortex_m_rt::entry;
use motor::peripherals::uart::UartDma;
use stm32h7::stm32h743v::USART2;
use stm32h7xx_hal::interrupt;
use cortex_m_semihosting::hprintln;
use panic_halt as _;

static radio_uart: Option<UartDma<USART2, Stream0, Stream1> = None;

#[entry]
fn main() -> ! {
    loop {}
}

#[interrupt]
fn USART2() { }

#[interrupt]
fn DMA1_STR0() { }

#[interrupt]
fn DMA1_STR1() { }