
use core::u8;

use ateam_common_rs::io_queue::IoQueue;

use stm32h7xx_hal::{
    dma::{
        self,
        Transfer,
        MemoryToPeripheral,
        PeripheralToMemory,
        DBTransfer, 
        dma::DmaConfig},
    pac,
    prelude::*,
    serial::{
        self,
        Rx,
        Tx}, 
};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SerialTransmissionMode {
    Polling,
    Interrupts,
    DmaInterrupts,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum UartTransmitError {
    // things that are the "user's fault"
    ModeInvalid,
    BufferFull,
    BufferTooSmall,


    // things that are this module's fault
    InitializationStateInvalid,
    InternalStateInvalid,
}

pub struct UartDma<'rx_sto, 'tx_sto, USART, RxDmaStream, TxDmaStream> 
    where 'rx_sto: 'static,
          'tx_sto: 'static,
          RxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          TxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
          serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory> {

    // INBOUND / RX 

    rx_queue: IoQueue<'static>,
    rx_transmission_mode: SerialTransmissionMode,

    // dma record keeping
    rx_dma_transfer: Option<Transfer<RxDmaStream, Rx<USART>, PeripheralToMemory, &'rx_sto mut[u8], DBTransfer>>,

    // OUTBOUND / TX
    // dma_tx_transfer: RadioDmaTxTrs,
    // dma_tx_config: DmaConfig,
    tx_serial: Tx<USART>,

    tx_queue: IoQueue<'static>,
    tx_transmission_mode: SerialTransmissionMode,

    // dma record keeping
    tx_dma_config: Option<DmaConfig>,
    tx_dma_stream: Option<TxDmaStream>,
    tx_dma_transfer: Option<Transfer<TxDmaStream, Tx<USART>, MemoryToPeripheral, &'tx_sto [u8], DBTransfer>>,
    tx_dma_active: bool,
}

impl<'rx_sto, 'tx_sto, USART: serial::SerialExt, RxDmaStream, TxDmaStream> UartDma<'rx_sto, 'tx_sto, USART, RxDmaStream, TxDmaStream> 
    where RxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          TxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          // type mismatch resolving `<[u8] as embedded_dma::WriteTarget>::Word == <stm32h7xx_hal::serial::Tx<USART> as TargetAddress<stm32h7xx_hal::dma::MemoryToPeripheral>>::MemSize`
          // expected associated type `<stm32h7xx_hal::serial::Tx<USART> as TargetAddress<stm32h7xx_hal::dma::MemoryToPeripheral>>::MemSize`
          // found type `u8`
          // consider constraining the associated type `<stm32h7xx_hal::serial::Tx<USART> as TargetAddress<stm32h7xx_hal::dma::MemoryToPeripheral>>::MemSize` to `u8`
          // wtf https://stackoverflow.com/questions/70531785/constraint-associated-type-of-a-generic-associated-type
          serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral, MemSize = u8>,
          serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory> {


    fn init() {
        
    }

    ///////////////////////////
    //  interrupt callbacks  //
    ///////////////////////////

    fn uart_interrupt_cb() {

    }

    fn transmit_dma_stream_cb() {

    }

    fn receive_dma_stream_cb() {

    }

    ////////////////////
    //  tx functions  //
    ////////////////////

    pub fn transmit_initialize_dma(&self, tx_dma_stream: TxDmaStream) {

    }

    pub fn transmit_blocking(&self, tx_buf: &[u8]) -> Result<(), UartTransmitError> {
        unimplemented!();
    }

    pub fn transmit_nonblocking(&mut self, tx_buf: &[u8]) -> Result<(), UartTransmitError> {
        if self.tx_transmission_mode == SerialTransmissionMode::Polling {
            return Err(UartTransmitError::ModeInvalid);
        }

        // check if a transmission is already pending, we don't allow queueing
        if self.tx_queue.full() {
            return Err(UartTransmitError::BufferFull);
        }

        // check if the user is trying to send more than the buffer allows
        // TODO: write a function to call this multiple times for longer items
        if self.tx_queue.length() > tx_buf.len() {
            return Err(UartTransmitError::BufferTooSmall)
        }

        self.tx_queue.write(tx_buf);

        let result = match self.tx_transmission_mode {
            SerialTransmissionMode::Polling => self._transmit_polling(),
            SerialTransmissionMode::Interrupts => self._transmit_interrupts(),
            SerialTransmissionMode::DmaInterrupts => self._transmit_dma(),
        };

        return Ok(());
    }

    fn _transmit_polling(&self) -> Result<(), UartTransmitError> { unimplemented!(); }
    fn _transmit_interrupts(&self) -> Result<(), UartTransmitError> { unimplemented!(); }
    fn _transmit_dma<'tx_buf> (&mut self) -> Result<(), UartTransmitError> {
        if self.tx_transmission_mode != SerialTransmissionMode::DmaInterrupts {
            return Err(UartTransmitError::InitializationStateInvalid);
        }

        // this global transfer holds the stream and peripherial references
        // it should only be taken and then restored by this function
        // if it's None, something's gone horribly wrong
        // if self.tx_dma_transfer.is_none() {
        //     //hprintln!("dma transfer state was none when state is strictly managed internal to this function");
        //     //hprintln!("\twas initialization invalid or was the internal state management changed?");
        //     return Err(UartTransmitError::InternalStateInvalid)
        // }

        // take the old transfer, reclaim ownership of hardware in preparation for the next transfer
        let old_dma_transfer = self.tx_dma_transfer.take().unwrap();
        let (stream, serial, _, _) = old_dma_transfer.free();

        let tx_io_buf = self.tx_queue.peek_top_buf();
        let tx_buf = &tx_io_buf.get_io_buf()[..tx_io_buf.len()];

        // create the next transfer
        let dma_transfer = 
            Transfer::init(
                stream, 
                serial, 
                tx_buf, 
                None, 
                self.tx_dma_config.unwrap());

        // restore the global state
        self.tx_dma_transfer = Some(dma_transfer);

        // start the transfer by setting the flag and enabling DMA
        self.tx_dma_transfer.as_mut().unwrap().start(|serial| {
            self.tx_dma_active = true;
            serial.enable_dma_tx();
        });

        return Ok(());
    }

    fn wait_for_transmission() {
        unimplemented!();
    }

    fn transmit_pending() -> bool {
        unimplemented!();
    }

    ////////////////////
    //  rx functions  //
    ////////////////////
}
