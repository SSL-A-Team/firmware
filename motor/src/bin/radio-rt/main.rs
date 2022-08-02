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
static mut radio_dma_tx_config: Option<DmaConfig> = None;
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

    let (radio_uart_tx_ch, _) = serial.split();

    //////////////////////////
    //  setup dma transfer  //
    //////////////////////////

    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let dma_tx_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .transfer_error_interrupt(true);
    unsafe { radio_dma_tx_config = Some(dma_tx_config) }

    let mut radio_dma_tx_stream = streams.0;
    radio_dma_tx_stream.set_transfer_complete_interrupt_enable(true);

    unsafe {
    let transfer: RadioDmaTxTrs = 
        Transfer::init(
            radio_dma_tx_stream, 
            radio_uart_tx_ch, 
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
    let mut p = gpiod.pd7.into_push_pull_output();
    p.set_low();
    delay.delay_us(10u16);

    unsafe {
    loop {
        p.set_high();
        uart_dma_transfer(&[0xFFu8, 0x00, 0xFF, 0x00]);
        p.set_low();
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

enum UartDmaError {
    INSUFFICIENT_TX_BUF_SIZE,
    DMA_BUSY,
    INTERNAL_STATE_ERR,
}

/**
 * 
 */
unsafe fn uart_dma_transfer(tx_buf: &[u8]) -> Result<(), UartDmaError> {
    // check if a transmission is already pending, we don't allow queueing
    if dma_transfer_pending() {
        return Err(UartDmaError::DMA_BUSY);
    }

    // check if the user is trying to send more than the buffer allows
    // TODO: write a function to call this multiple times for longer items
    if tx_buf.len() > radio_uart_tx_buf.len() {
        return Err(UartDmaError::INSUFFICIENT_TX_BUF_SIZE)
    }

    // for (i, value) in buf.into_iter().enumerate() {
    //     if i < radio_uart_tx_buf.len() {
    //         radio_uart_tx_buf[i] = *value;
    //     }
    // }
    radio_uart_tx_buf[..tx_buf.len()].clone_from_slice(tx_buf);

    // this global transfer holds the stream and peripherial references
    // it should only be taken and then restored by this function
    // if it's None, something's gone horribly wrong
    if radio_dma_transfer.is_none() {
        hprintln!("dma transfer state was none when state is strictly managed internal to this function");
        hprintln!("\twas initialization invalid or was the internal state management changed?");
        return Err(UartDmaError::INTERNAL_STATE_ERR)
    }

    // take the old transfer, reclaim ownership of hardware in preparation for the next transfer
    let dma_transfer = radio_dma_transfer.take().unwrap();
    let (stream, serial, _, opt_buf) = dma_transfer.free();

    // create the next transfer
    let dma_transfer: RadioDmaTxTrs = 
        Transfer::init(
            stream, 
            serial, 
            &mut radio_uart_tx_buf[..tx_buf.len()], 
            opt_buf, 
            radio_dma_tx_config.as_ref().unwrap().clone());

    // restore the global state
    radio_dma_transfer = Some(dma_transfer);

    // start the transfer by setting the flag and enabling DMA
    radio_dma_transfer.as_mut().unwrap().start(|serial| {
        radio_dma_transfer_pending = true;
        serial.enable_dma_tx();
    });

    return Ok(());
}

/**
 * 
 */
fn dma_transfer_pending() -> bool {
    return unsafe { radio_dma_transfer_pending };
}

/**
 * 
 */
fn wait_for_transmission() {
    while dma_transfer_pending() {
        cortex_m::asm::nop();
    }
}

/**
 * 
 */
#[interrupt]
fn USART2() {
    // most of this function dereferences global static or needs
    // direct register/raw ptr access so were just put in a big block
    unsafe {
        // cr1 contains the bit enabling/disabling USART interrupts
        let cr1 = &(*pac::USART2::ptr()).cr1;
        // isr tells us which interrupt flag was set (success/err etc)
        let isr = &(*pac::USART2::ptr()).isr;
        // icr allows us to clear the set flag with a write 
        let icr = &(*pac::USART2::ptr()).icr;

        // check which flag caused the interrupt
        if isr.read().tc().bit_is_set() {
            // cause: successful transmission of tx frame

            // disable transfer complete interrupts on USART2
            cr1.modify(|_, w| w.tcie().clear_bit());

            // clear USART2 transfer compliete interrupt pending bit
            icr.write(|w| w.tccf().set_bit());

            // flag transfer complete
            radio_dma_transfer_pending = false;
        }

        // TODO hand other errors
    }
}

/**
 * 
 */
#[interrupt]
fn DMA1_STR0() {
    // most of this function dereferences global static or needs
    // direct register/raw ptr access so were just put in a big block
    unsafe {
        // this should be 
        if let Some(tx) = radio_dma_transfer.as_mut() {
            // if let Some(lr) = &mut led_r {
            //     lr.set_high();
            // }

            // check which flag caused the end of DMA
            if tx.get_transfer_complete_flag() {
                // tx complete flag (aka success) ended the transfer

                // the DMA write is done, but the USART peripherial may still have
                // data in the internal FIFO or output shift register
                // we need to enable to frame transfer complete flag 
                // which will fire an interrupt on USART handler once the last byte
                // in the FIFO has been loaded into the output shift register 

                // enable USART2 transfer complete interrupt (will fire when SR is empty)
                (*pac::USART2::ptr()).cr1.modify(|_, w| w.tcie().set_bit());

                // disable DMA interrupts on UART tx
                tx.get_stream().set_transfer_complete_interrupt_enable(false);

                // clear flag for this DMA transaction
                tx.clear_transfer_complete_interrupt();
            }

            // TODO handle other errors
        }
    }
}