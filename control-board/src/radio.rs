use ateam_common::{
    radio::odin_radio::RadioInterfaceControl,
    transfer::{Reader2, Writer2},
};
use embassy_stm32::{
    gpio::{Level, Output, Pin, Speed},
    usart,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{
    queue::{DequeueRef, EnqueueRef},
    stm32_interface::{update_usart, Kind},
    uart_queue::{UartReadQueue, UartWriteQueue},
};

pub struct RadioInterfaceUart<
    UART: usart::BasicInstance,
    RxDma: usart::RxDma<UART>,
    TxDma: usart::TxDma<UART>,
    const LEN_RX: usize,
    const DEPTH_RX: usize,
    const LEN_TX: usize,
    const DEPTH_TX: usize,
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
        const LEN_RX: usize,
        const DEPTH_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        ResetPin: Pin,
    > RadioInterfaceUart<UART, RxDma, TxDma, LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, ResetPin>
{
    pub fn new(
        read_queue: &'static UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
        write_queue: &'static UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
        reset_pin: Output<'static, ResetPin>,
    ) -> Self {
        // let reset_pin = Output::new(reset_pin, Level::Low, Speed::Medium);
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
        const LEN_RX: usize,
        const DEPTH_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        ResetPin: Pin,
    > Writer2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, ResetPin>
{
    type DataRefWrite = EnqueueRef<'static, LEN_TX, DEPTH_TX>;

    async fn write(&self) -> Result<Self::DataRefWrite, ()> {
        self.write_queue.enqueue2()
    }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_RX: usize,
        const DEPTH_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        ResetPin: Pin,
    > Reader2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, ResetPin>
{
    type DataRefRead = DequeueRef<'static, LEN_RX, DEPTH_RX>;

    async fn read(&self) -> Result<Self::DataRefRead, ()> {
        self.read_queue.dequeue2().await
    }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        const LEN_RX: usize,
        const DEPTH_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        ResetPin: Pin,
    > RadioInterfaceControl
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_RX, DEPTH_RX, LEN_TX, DEPTH_TX, ResetPin>
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
        config.parity = if parity {
            usart::Parity::ParityEven
        } else {
            usart::Parity::ParityNone
        };
        update_usart(r, &config, UART::frequency(), Kind::Uart, true, true)
    }
}

// #[macro_export]
// macro_rules! odin_radio {
//     ($uart:ty, $dma_rx:ty, $dma_tx:ty, $reset_pin:ty) => {
//         usart_buffer!(RadioQueue, $uart, $dma_rx, $dma_tx, 256, 20, 256, 10);
//         type RadioInterfaceUartValue =
//             RadioInterfaceUart<$uart, $dma_rx, $dma_tx, 256, 20, 256, 10, $reset_pin>;
//         static ODIN_TASK: TaskStorage<OdinRadio<'static, RadioInterfaceUartValue>> =
//             TaskStorage::new();
//     };
// }

// #[macro_export]
// macro_rules! include_external_cpp_bin {
//     ($tx_, $bin_file:literal) => {
//         #[link_section = ".axisram.buffers"]
//         static mut BUFFERS_TX: [queue::Buffer<256>; 10] = [queue::Buffer::EMPTY; 10];
//         static QUEUE_TX: UartWriteQueue<USART10, DMA2_CH0, 256, 10> =
//             UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

//         #[link_section = ".axisram.buffers"]
//         static mut BUFFERS_RX: [queue::Buffer<256>; 20] = [queue::Buffer::EMPTY; 20];
//         static QUEUE_RX: UartReadQueue<USART10, DMA2_CH1, 256, 20> =
//             UartReadQueue::new(unsafe { &mut BUFFERS_RX });

//         // pub static $var_name: &[u8; include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../motor-controller/build/bin/", $bin_file)).len()]
//         //     = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/../motor-controller/build/bin/", $bin_file));
//     }
// }
