use core::fmt::Write;
use core::marker::PhantomData;
use core::mem::{self, MaybeUninit};
use embedded_dma::{ReadBuffer, WriteBuffer};
use stm32h7xx_hal::dma::traits::{DoubleBufferedConfig, DoubleBufferedStream, TargetAddress};
use stm32h7xx_hal::dma::{ConstDBTransfer, DBTransfer, MemoryToPeripheral};
use stm32h7xx_hal::stm32::{USART1, USART3};
use stm32h7xx_hal::{
    dma::{dma::DmaConfig, traits::Stream, Transfer},
    pac, serial,
};

#[cfg(feature = "stspin_program")]
const STSPIN_BINARY_SIZE: usize = include_bytes!(env!("STSPIN_BINARY")).len();

#[cfg(feature = "stspin_program")]
// TODO: better way to do this?
// DMA hardware limitation of 64KB without chunking
static_assertions::const_assert!(STSPIN_BINARY_SIZE <= 65535);

#[cfg(feature = "stspin_program")]
#[link_section = ".axisram_prog.stspin"]
static STSPIN_BINARY: [u8; STSPIN_BINARY_SIZE] = *include_bytes!(env!("STSPIN_BINARY"));

// TODO: remove this:
#[cfg(not(feature = "stspin_program"))]
#[link_section = ".axisram_prog.stspin"]
static STSPIN_BINARY: [u8; include_bytes!("../steval-spin3201-6step.bin").len()] =
    *include_bytes!("../steval-spin3201-6step.bin");

pub struct STSpinTransfer<USART, STREAM: Stream>
where
    serial::Tx<USART>: TargetAddress<MemoryToPeripheral>,
{
    tx_transfer: UsartDma<USART, STREAM>,
    rx: serial::Rx<USART>,
}

type UsartDma<USART, STREAM> =
    Transfer<STREAM, serial::Tx<USART>, MemoryToPeripheral, &'static [u8], ConstDBTransfer>;

pub trait Serial<USART> {
    fn split(self) -> (serial::Tx<USART>, serial::Rx<USART>);
    fn join(tx: serial::Tx<USART>, rx: serial::Rx<USART>) -> Self;
}

impl Serial<USART1> for serial::Serial<USART1> {
    fn split(self) -> (serial::Tx<USART1>, serial::Rx<USART1>) {
        self.split()
    }
    fn join(tx: serial::Tx<USART1>, rx: serial::Rx<USART1>) -> Self {
        Self::join(tx, rx)
    }
}
impl Serial<USART3> for serial::Serial<USART3> {
    fn split(self) -> (serial::Tx<USART3>, serial::Rx<USART3>) {
        self.split()
    }
    fn join(tx: serial::Tx<USART3>, rx: serial::Rx<USART3>) -> Self {
        Self::join(tx, rx)
    }
}

pub trait SerialTx<USART> {
    fn enable_dma_tx(&mut self);
}
impl SerialTx<USART3> for serial::Tx<USART3> {
    fn enable_dma_tx(&mut self) {
        self.enable_dma_tx()
    }
}

impl<USART, STREAM> STSpinTransfer<USART, STREAM>
where
    serial::Serial<USART>: Serial<USART>,
    serial::Tx<USART>: SerialTx<USART>,
    STREAM: DoubleBufferedStream + Stream<Config = DmaConfig>,
    serial::Tx<USART>: TargetAddress<MemoryToPeripheral>,
    &'static [u8]:
        ReadBuffer<Word = <serial::Tx<USART> as TargetAddress<MemoryToPeripheral>>::MemSize>,
{
    pub fn init(serial: serial::Serial<USART>, stream: STREAM) -> STSpinTransfer<USART, STREAM> {
        let (tx, rx) = serial.split();
        let config = DmaConfig::default().memory_increment(true);
        let tx_transfer = Transfer::init_const(stream, tx, &STSPIN_BINARY[..], None, config);
        STSpinTransfer { tx_transfer, rx }
    }

    pub fn start(&mut self) {
        self.tx_transfer.start(|serial| {
            serial.enable_dma_tx();
        });
    }

    pub fn free(mut self) -> (serial::Serial<USART>, STREAM) {
        let (stream, tx, _, _) = self.tx_transfer.free();
        (Serial::join(tx, self.rx), stream)
    }
}
