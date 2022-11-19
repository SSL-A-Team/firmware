use embassy_stm32::gpio::{OutputOpenDrain, Pin};
use embassy_stm32::usart;

use crate::uart_queue::{UartReadQueue, UartWriteQueue};

pub struct Stm32Interface<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize, 
        const DEPTH_TX: usize,
        Boot0Pin: Pin, 
        ResetPin: Pin
> {
    reader: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
    writer: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
    boot0_pin: Option<OutputOpenDrain<'a, Boot0Pin>>,
    reset_pin: Option<OutputOpenDrain<'a, ResetPin>>,
}

impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_RX: usize, 
        const DEPTH_TX: usize,
        Boot0Pin: Pin, 
        ResetPin: Pin
    > Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin>
{
    pub async fn new(        
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        boot0_pin: Option<OutputOpenDrain<'a, Boot0Pin>>,
        reset_pin: Option<OutputOpenDrain<'a, ResetPin>>
    ) -> Stm32Interface<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_RX, DEPTH_TX, Boot0Pin, ResetPin> {
        Stm32Interface {
            reader: read_queue,
            writer: write_queue,
            boot0_pin,
            reset_pin
        }
    }
}

