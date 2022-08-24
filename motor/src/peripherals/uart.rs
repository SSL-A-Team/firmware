
use super::io_queue::IoQueue;

enum UartTransmitError {
    InsufficientBufSize,
    FunctionBusy,
    InitializationStateInvalid,
    InternalStateInvalid,
}

struct Uart<const TX_BUF_LEN: usize, const TX_BUF_DEPTH: usize, const RX_BUF_LEN: usize, const RX_BUF_DEPTH: usize> {

    // outbound/tx data
    // dma_tx_transfer: RadioDmaTxTrs,
    // dma_tx_config: DmaConfig,

    tx_queue: IoQueue<TX_BUF_LEN, TX_BUF_DEPTH>,

    // inbound/rx data
}

impl<const TX_BUF_LEN: usize, const TX_BUF_DEPTH: usize, const RX_BUF_LEN: usize, const RX_BUF_DEPTH: usize> Uart<TX_BUF_LEN, TX_BUF_DEPTH, RX_BUF_LEN, RX_BUF_DEPTH> {
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

    fn transmit_blocking(&self, buf: &[u8]) -> Result<(), UartTransmitError> {
        unimplemented!();
    }

    fn transmit_nonblocking(&self, buf: &[u8]) -> Result<(), UartTransmitError> {
        unimplemented!();
    }

    fn transmit_dma(&self, buf: &[u8]) -> Result<(), UartTransmitError> {
        // check if a transmission is already pending, we don't allow queueing
        if dma_transfer_pending() {
            return Err(UartDmaError::DmaBusy);
        }

        // check if the user is trying to send more than the buffer allows
        // TODO: write a function to call this multiple times for longer items
        if tx_buf.len() > radio_uart_tx_buf.len() {
            return Err(UartDmaError::InsufficientTxBufSize)
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
            return Err(UartDmaError::InternalStateErr)
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

    fn wait_for_transmission() {

    }

    fn transmit_pending() -> bool {

    }

    ////////////////////
    //  rx functions  //
    ////////////////////
}
