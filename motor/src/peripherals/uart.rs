
use core::{u8, cell::{RefMut, RefCell}};

use ateam_common_rs::io_queue::{IoQueue, IoBuffer};

use stm32h7xx_hal::{
    dma::{
        self,
        Transfer,
        MemoryToPeripheral,
        PeripheralToMemory,
        DBTransfer, 
        dma::DmaConfig},
    pac,
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
pub enum UartTransmitError {
    // things that are the "user's fault"
    ModeInvalid,
    BufferFull,
    BufferTooSmall,


    // things that are this module's fault
    InitializationStateInvalid,
    InternalStateInvalid,
}

pub struct UartDma<USART, RxDmaStream, TxDmaStream, const BUF_SIZE: usize> 
    where RxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          TxDmaStream: dma::traits::Stream<Config = DmaConfig> + dma::traits::DoubleBufferedStream,
          serial::Tx<USART>: dma::traits::TargetAddress<dma::MemoryToPeripheral>,
          serial::Rx<USART>: dma::traits::TargetAddress<dma::PeripheralToMemory> {

    // INBOUND / RX 

    rx_queue: IoQueue<'static, BUF_SIZE>,
    rx_transmission_mode: SerialTransmissionMode,

    // dma record keeping
    rx_dma_transfer: Option<Transfer<RxDmaStream, Rx<USART>, PeripheralToMemory, RefMut<'static, IoBuffer<BUF_SIZE>>, DBTransfer>>,

    // OUTBOUND / TX
    tx_serial: Option<Tx<USART>>,
    tx_queue: IoQueue<'static, BUF_SIZE>,
    tx_transmission_mode: SerialTransmissionMode,
    tx_enabled: bool,

    // dma record keeping
    tx_dma_config: Option<DmaConfig>,
    tx_dma_stream: Option<TxDmaStream>,
    tx_dma_transfer: Option<Transfer<TxDmaStream, Tx<USART>, MemoryToPeripheral, RefMut<'static, IoBuffer<BUF_SIZE>>, DBTransfer>>,
}

impl<USART: serial::SerialExt, RxDmaStream, TxDmaStream, const BUF_SIZE: usize> UartDma<USART, RxDmaStream, TxDmaStream, BUF_SIZE> 
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

    fn get_uart_register_block(&self) -> &'static stm32h7xx_hal::stm32::usart1::RegisterBlock {
        return unsafe { &*USART::ptr() };
    }

    fn uart_interrupt_cb(&mut self) {
        // most of this function dereferences global static or needs
        // direct register/raw ptr access so were just put in a big block

        // cr1 contains the bit enabling/disabling USART interrupts
        let cr1 = &self.get_uart_register_block().cr1;
        // isr tells us which interrupt flag was set (success/err etc)
        let isr = &self.get_uart_register_block().isr;
        // icr allows us to clear the set flag with a write 
        let icr = &self.get_uart_register_block().icr;

        // check which flag caused the interrupt
        if isr.read().tc().bit_is_set() {
            // cause: successful transmission of tx frame

            // disable transfer complete interrupts on USART2
            cr1.modify(|_, w| w.tcie().clear_bit());

            // clear USART2 transfer compliete interrupt pending bit
            icr.write(|w| w.tccf().set_bit());

            self._transmit_dma();
        }
    }

    fn transmit_dma_stream_cb(&mut self) {
        // this should always be Some... how can transfer interrupt complete event fire with no transfer
        if let Some(tx) = self.tx_dma_transfer.as_mut() {
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

                unsafe {
                    // enable USART2 transfer complete interrupt (will fire when SR is empty)
                    (*pac::USART2::ptr()).cr1.modify(|_, w| w.tcie().set_bit());

                    // disable DMA interrupts on UART tx
                    tx.get_stream().set_transfer_complete_interrupt_enable(false);
                }

                // clear flag for this DMA transaction
                tx.clear_transfer_complete_interrupt();
            }

            // TODO handle other errors
        }
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
            SerialTransmissionMode::DmaInterrupts => self.start_transmit_dma(),
        };

        return Ok(());
    }

    fn _transmit_polling(&self) -> Result<(), UartTransmitError> { unimplemented!(); }
    fn _transmit_interrupts(&self) -> Result<(), UartTransmitError> { unimplemented!(); }

    fn start_transmit_dma(&mut self) -> Result<(), UartTransmitError> {
        if self.tx_transmission_mode != SerialTransmissionMode::DmaInterrupts {
            return Err(UartTransmitError::InitializationStateInvalid);
        }

        // there's already a pending transfer, no need to start transmission again
        if self.tx_dma_transfer.is_some() {
            return Ok(())
        }

        return self._transmit_dma();
    }

    // NOTE: this function may be called from an interrupt
    fn _transmit_dma (&mut self) -> Result<(), UartTransmitError> {
        if self.tx_transmission_mode != SerialTransmissionMode::DmaInterrupts {
            return Err(UartTransmitError::InitializationStateInvalid);
        }

        // check if there was a previous transfer, if so free it and reclaim the resources
        if let Some(old_dma_transfer) = self.tx_dma_transfer.take() {
            let (stream, serial, dma_buf, _) = old_dma_transfer.free();

            // we mutably borrowed from the read element to pass to the dma engine
            // its done, so we update the queue bookkeeping and free the MutRef with this function call
            let result = self.tx_queue.finalize_rpeek(dma_buf);

            self.tx_dma_stream = Some(stream);
            self.tx_serial = Some(serial);
        }

        // there's something to send, lets send it
        if self.tx_enabled && !self.tx_queue.empty() {
            if let Ok(dma_buf) = self.tx_queue.peek_mut() {
                // create the next transfer
                let dma_transfer = 
                    Transfer::init(
                        self.tx_dma_stream.take().unwrap(), 
                        self.tx_serial.take().unwrap(), 
                        dma_buf, 
                        None, 
                        self.tx_dma_config.unwrap());

                // restore the global state
                self.tx_dma_transfer = Some(dma_transfer);

                // start the transfer by setting the flag and enabling DMA
                self.tx_dma_transfer.as_mut().unwrap().start(|serial| {
                    serial.enable_dma_tx();
                });

            } else {
                // TODO handle peek error (try borrow mut failed)
            }
        }

        return Ok(());
    }

    fn wait_for_transmission() {
        unimplemented!();
    }

    fn transmit_pending(&self) -> bool {
        return self.tx_dma_transfer.is_some();
    }

    ////////////////////
    //  rx functions  //
    ////////////////////
}
