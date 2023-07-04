use ateam_common::{transfer::{Writer2, Reader2}, radio::odin_radio::RadioInterfaceControl};
use embassy_stm32::{usart, gpio::{Pin, Output, Level, Speed}};
use embassy_sync::{mutex::Mutex, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::{Timer, Duration};

use crate::{uart_queue::{UartReadQueue, UartWriteQueue}, queue::{DequeueRef, EnqueueRef}, stm32_interface::{update_usart, Kind}};

pub struct RadioInterfaceUart<
    UART: usart::BasicInstance,
    RxDma: usart::RxDma<UART>,
    TxDma: usart::TxDma<UART>,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    ResetPin: Pin,
> {
    read_queue: &'static UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
    write_queue: &'static UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
    reset_pin: Mutex<CriticalSectionRawMutex, Output<'static, ResetPin>>,
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    pub fn new(
        read_queue: &'static UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
        write_queue: &'static UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
        reset_pin: ResetPin,
    ) -> Self {
        let reset_pin = Output::new(reset_pin, Level::Low, Speed::Medium);
        Self {
            read_queue,
            write_queue,
            reset_pin: Mutex::new(reset_pin),
        }
    }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > Writer2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    type DataRefWrite = EnqueueRef<'static, LEN_TX, DEPTH_TX> ;

    async fn write(&self) -> Result<Self::DataRefWrite, ()> {
        self.write_queue.enqueue2()
    }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > Reader2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    type DataRefRead = DequeueRef<'static, LEN_RX, DEPTH_RX> ;

    async fn read(&self) -> Result<Self::DataRefRead, ()> {
        self.read_queue.dequeue2().await
    }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > RadioInterfaceControl
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    async fn reset_radio(&self) {
        let reset_pin = &mut *self.reset_pin.lock().await;
        reset_pin.set_high();
        Timer::after(Duration::from_micros(300)).await;
        reset_pin.set_low();
    }
    async fn config_uart(&self, baudrate: u32, flow_control: bool, data_bits: u8, parity: bool) {
        let r = UART::regs();

        let mut config = usart::Config::default();
        config.baudrate = baudrate;
        config.parity = if parity { usart::Parity::ParityEven } else { usart::Parity::ParityNone };
        update_usart(r, &config, UART::frequency(), Kind::Uart, true, true)
    }
}