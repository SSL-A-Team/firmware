#![no_std]
#![no_main]

use core::borrow::{Borrow, BorrowMut};
use core::{mem, mem::MaybeUninit};
use core::fmt::Write;

use cortex_m_rt::entry;

use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::spi::transfer;
use panic_halt as _;

use motor::{peripherals::{stm32_bootloader, timeout::delay_us}, peripherals::timeout, include_bin};
use stm32h7xx_hal::dma::{PeripheralToMemory, DBTransfer};
use stm32h7xx_hal::dma::dma::{
    Stream0,
    Stream1
};
use stm32h7xx_hal::dma::traits::Stream;
use stm32h7xx_hal::gpio::{Pin, Output};
use stm32h7xx_hal::interrupt;
use stm32h7xx_hal::serial::{Tx, Event};
use stm32h7xx_hal::{
    block, 
    pac,
    prelude::*,
    serial::{self, Serial, config},
};

use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    MemoryToPeripheral, Transfer,
};

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access. In this case we use
// AXI SRAM.
//
// The runtime does not initialise these SRAM banks
const TX_BUFFER_LEN: usize = 16;
const RX_BUFFER_LEN: usize = 16;



#[link_section = ".axisram.buffers"]
static mut RX_BUFFER: [u8; TX_BUFFER_LEN] = [0; TX_BUFFER_LEN];

type RadioDmaTxTrs<'a> = Transfer<
    Stream0<pac::DMA1>,
    Tx<pac::USART2>,
    MemoryToPeripheral,
    //&'static mut [u8; TX_BUFFER_LEN],
    &'static mut [u8],
    DBTransfer,
>;

static mut led_g: Option<Pin<'B', 0, Output>> = None;
static mut led_r: Option<Pin<'B', 14, Output>> = None;
static mut led_y: Option<Pin<'E', 1, Output>> = None;


static mut radio_dma_transfer_pending: bool = false;
static mut radio_dma_transfer: Option<RadioDmaTxTrs> = None;
static mut radio_dma_tx_stream: Option<Stream0<pac::DMA1>> = None;
static mut radio_dma_tx_config: Option<DmaConfig> = None;
static mut radio_uart_tx_ch: Option<Tx<pac::USART2>> = None;
#[link_section = ".axisram.buffers"]
static mut radio_uart_tx_buf: [u8; TX_BUFFER_LEN] = [0; TX_BUFFER_LEN];

#[entry]
fn main() -> ! {
    ////////////////////////////////
    //  Setup clocks and periphs  //
    ////////////////////////////////

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

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let mut lg = gpiob.pb0.into_push_pull_output();
    let mut lr = gpiob.pb14.into_push_pull_output();
    let mut ly = gpioe.pe1.into_push_pull_output();
    unsafe { led_g = Some(lg); };
    unsafe { led_y = Some(ly); };
    unsafe { led_r = Some(lr); };

    ////////////////////
    //  setup serial  //
    ////////////////////

    // create a serial config
    let config = serial::config::Config::new(115_200.bps())
        .parity_none()
        .stopbits(config::StopBits::STOP1);

    // initialize serial
    let mut serial = dp
        .USART2
        .serial(
            (
                gpiod.pd5.into_alternate().internal_pull_up(true),
                gpiod.pd6.into_alternate().internal_pull_up(true),
            ),
            config,
            ccdr.peripheral.USART2,
            &ccdr.clocks,
        )
        .unwrap();

    let (serial_tx, _) = serial.split();
    unsafe { radio_uart_tx_ch = Some(serial_tx) };

    // let tx_buffer: &'static mut [u8; TX_BUFFER_LEN] = {
    //     let buf: &mut [MaybeUninit<u8>; TX_BUFFER_LEN] =
    //         unsafe { mem::transmute(&mut TX_BUFFER) };

    //     for (i, value) in buf.iter_mut().enumerate() {
    //         unsafe { value.as_mut_ptr().write(0xAA); }
    //     }

    //     unsafe { mem::transmute(buf) }
    // };

    //////////////////////////
    //  setup dma transfer  //
    //////////////////////////

    // Setup the DMA transfer on stream 0
    //
    // We need to specify the direction with a type annotation, since DMA
    // transfers both to and from the UART are possible
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);


    let dma_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .transfer_error_interrupt(true);
    unsafe { radio_dma_tx_config = Some(dma_config); };

    let mut tx_dma_stream = streams.0;
    tx_dma_stream.set_transfer_complete_interrupt_enable(true);
    unsafe { radio_dma_tx_stream = Some(tx_dma_stream) };

    unsafe {
    let mut transfer: RadioDmaTxTrs = 
        Transfer::init(
            radio_dma_tx_stream.take().unwrap(), 
            radio_uart_tx_ch.take().unwrap(), 
            &mut radio_uart_tx_buf, 
            None, 
            radio_dma_tx_config.as_ref().unwrap().clone());

            radio_dma_transfer = Some(transfer);
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA1_STR0);
        pac::NVIC::unmask(pac::Interrupt::USART2);
    }

    let mut delay = cp.SYST.delay(ccdr.clocks);

    unsafe {
    loop {
        uart_dma_transfer(&[0xFFu8, 0x00, 0xFF, 0x00]);
        //led_r.unwrap().set_high();
        wait_for_transmission();

        //hprintln!("2");

        delay.delay_us(500u16);

        uart_dma_transfer(&[0xAAu8, 0x00, 0xAA, 00]);
        wait_for_transmission();

        delay.delay_us(500u16);

        uart_dma_transfer(&[0xAAu8, 0x00, 0xAA, 00, 0xAA, 0x00, 0xAA, 00, 0xAA, 0x00, 0xAA, 00, 0xAA, 0x00, 0xAA, 00]);
        wait_for_transmission();

        delay.delay_us(500u16);

    }
    }

    // (tx_dma_ch, serial_tx, dma_tx_buf, _) = transfer.free();
}

unsafe fn uart_dma_transfer(buf: &[u8]) {
    for (i, value) in buf.into_iter().enumerate() {
        if i < radio_uart_tx_buf.len() {
            radio_uart_tx_buf[i] = *value;
        }
    }

    // todo len bounds check

    // move back stuff
    let dma_transfer = radio_dma_transfer.take().unwrap();
    let (stream, serial, _, opt_buf) = dma_transfer.free();
        
    let buf = &mut radio_uart_tx_buf[0..buf.len()];

    let dma_transfer: RadioDmaTxTrs = 
        Transfer::init(
            stream, 
            serial, 
            buf, 
            opt_buf, 
            radio_dma_tx_config.as_ref().unwrap().clone());

    radio_dma_transfer = Some(dma_transfer);

    //let mut dma_transfer = radio_dma_transfer.take().unwrap();
    radio_dma_transfer.as_mut().unwrap().start(|serial| {
         radio_dma_transfer_pending = true;
         serial.enable_dma_tx();
    });

}

fn dma_transfer_pending() -> bool {
    return unsafe { radio_dma_transfer_pending };
}

fn wait_for_transmission() {
    while dma_transfer_pending() {
        cortex_m::asm::nop();
    }
}

#[interrupt]
unsafe fn USART2() {
    let cr1 = &(*pac::USART2::ptr()).cr1;
    let isr = &(*pac::USART2::ptr()).isr;
    let icr = &(*pac::USART2::ptr()).icr;
    // this interrupt fired on frame transfer complete
    if isr.read().tc().bit_is_set() {
        // disable transfer complete interrupts on USART2
        cr1.modify(|_, w| w.tcie().clear_bit());
        // clear USART2 transfer compliete interrupt pending bit
        icr.write(|w| w.tccf().set_bit());

        // flag transfer complete
        radio_dma_transfer_pending = false;
    }

    // TODO hand other errors
}

#[interrupt]
fn DMA1_STR0() {
    unsafe {
        if let Some(tx) = &mut radio_dma_transfer {
            if let Some(lr) = &mut led_r {
                lr.set_high();
            }

            //let mut tx = radio_dma_transfer.take().unwrap();
            if tx.get_transfer_complete_flag() {
                // enable USART2 transfer complete interrupt (will fire when SR is empty)
                (*pac::USART2::ptr()).cr1.modify(|_, w| w.tcie().set_bit());
                // disable DMA interrupts on UART tx
                tx.get_stream().set_transfer_complete_interrupt_enable(false);
                // clear flag for this transaction
                tx.clear_transfer_complete_interrupt();
            }

            // TODO handle other errors
        }
    }
}