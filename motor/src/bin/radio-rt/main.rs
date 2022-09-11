#![no_std]
#![no_main]

#![feature(trait_alias)]
#![feature(const_for)]

use cortex_m_rt::entry;
use cortex_m::interrupt::Mutex;
use stm32h7::stm32h743v::{USART2, DMA1};
use stm32h7xx_hal::{prelude::*, pac, interrupt, dma::dma::{Stream0, Stream1}, pwr::PwrExt};
use cortex_m_semihosting::hprintln;
use panic_halt as _;
use arr_macro::arr;
use core::cell::RefCell;

use ateam_common_rs::{io_queue::{IoQueue, IoBuffer}, axisram_ioqueue_storage_var, ioqueue_storage, iobuf};
use motor::{uart_dma_backing_storage_var, peripherals::uart::{UartDma, UartDmaBackingStorage}};
use motor::drivers::odin_w26x::OdinW26X;

static device_peripherals: Mutex<RefCell<Option<pac::Peripherals>>> = Mutex::new(RefCell::new(None));
static cortex_peripherals: Mutex<RefCell<Option<cortex_m::peripheral::Peripherals>>> = Mutex::new(RefCell::new(None));




type UnusedDmaType = Stream0<DMA1>;

uart_dma_async_storage!(radio, 512, 4, 512, 2);
type RadioDmaType = UartDma<USART2, UnusedDmaType, Stream0<DMA1>, RADIO_RX_STO_LEN, RADIO_TX_STO_LEN>;


static radio_uart_dma: RadioDmaType = UartDma::new_from_sto(&radio_sto);

//static radio: Option<OdinW26X<USART2, Stream0<DMA1>, Stream1<DMA1>>> = None; 

#[entry]
fn main() -> ! {
    ////////////////////////////////
    //  Setup clocks and periphs  //
    ////////////////////////////////

    system_setup();

    initialize_radio();

    loop {
        //radio_uart_dma.transmit_nonblocking(&[0xAA]);
    }
}

fn system_setup() {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // init system clock to 200MHz
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .pclk1(50.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    cortex_m::interrupt::free(|cs| {
        device_peripherals.borrow(cs).borrow_mut().insert(pac::Peripherals::take().unwrap());
        cortex_peripherals.borrow(cs).borrow_mut().insert(cortex_m::peripheral::Peripherals::take().unwrap());
    });
}


fn initialize_radio() {
}

#[interrupt]
fn USART2() { }

#[interrupt]
fn DMA1_STR0() { 
    //radio_uart_dma.transmit_dma_stream_cb();
}

#[interrupt]
fn DMA1_STR1() { }