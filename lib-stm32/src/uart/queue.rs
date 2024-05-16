#![warn(async_fn_in_trait)]

use core::{
    cell::SyncUnsafeCell,
    future::Future, sync::atomic::{AtomicBool, Ordering}};

use embassy_stm32::{
    mode::Async,
    usart::{self, UartRx, UartTx}
};
use embassy_executor::{
    raw::TaskStorage,
    SpawnToken};

use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex, signal::Signal
};
use embassy_time::Timer;

use crate::queue::{
    self,
    Buffer,
    DequeueRef, 
    Error,
    Queue
};

#[macro_export]
macro_rules! make_uart_queues {
    ($name:ident, $uart:ty, $uart_rx_dma:ty, $uart_tx_dma:ty, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr, $(#[$m:meta])*) => {
        $crate::paste::paste! {
        // shared mutex allowing safe runtime update to UART config
        const [<$name _UART_PERI_MUTEX>]: embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, bool> = 
            embassy_sync::mutex::Mutex::new(false);

        // tx buffer
        const [<$name _CONST_TX_BUF_VAL>]: core::cell::SyncUnsafeCell<$crate::queue::Buffer<$tx_buffer_size>> = 
            core::cell::SyncUnsafeCell::new($crate::queue::Buffer::EMPTY);
        $(#[$m])*
        static [<$name _TX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$tx_buffer_size>>; $tx_buffer_depth] = 
            [[<$name _CONST_TX_BUF_VAL>]; $tx_buffer_depth];
        static [<$name _TX_UART_QUEUE>]: $crate::uart::queue::UartWriteQueue<$uart, $uart_tx_dma, $tx_buffer_size, $tx_buffer_depth> = 
        $crate::uart::queue::UartWriteQueue::new(&[<$name _TX_BUFFER>], [<$name _UART_PERI_MUTEX>]);

        // rx buffer
        const [<$name _CONST_RX_BUF_VAL>]: core::cell::SyncUnsafeCell<$crate::queue::Buffer<$rx_buffer_size>> = 
            core::cell::SyncUnsafeCell::new($crate::queue::Buffer::EMPTY);
        $(#[$m])*
        static [<$name _RX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$rx_buffer_size>>; $rx_buffer_depth] = 
        [[<$name _CONST_RX_BUF_VAL>]; $rx_buffer_depth];
        static [<$name _RX_UART_QUEUE>]: $crate::uart::queue::UartReadQueue<$uart, $uart_rx_dma, $rx_buffer_size, $rx_buffer_depth> = 
            $crate::uart::queue::UartReadQueue::new(&[<$name _RX_BUFFER>], [<$name _UART_PERI_MUTEX>]);
        }
    };
}

pub struct UartReadQueue<
    UART: usart::BasicInstance,
    DMA: usart::RxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    uart_mutex: Mutex<CriticalSectionRawMutex, bool>,
    uart_config_update_signal: Signal<CriticalSectionRawMutex, bool>,
    queue_rx: Queue<LENGTH, DEPTH>,
    task: TaskStorage<ReadTaskFuture<UART, DMA, LENGTH, DEPTH>>,
}

// TODO: pretty sure shouldn't do this
unsafe impl<
        'a,
        UART: usart::BasicInstance,
        DMA: usart::RxDma<UART>,
        const LENGTH: usize,
        const DEPTH: usize,
    > Send for UartReadQueue<UART, DMA, LENGTH, DEPTH>
{
}

pub type ReadTaskFuture<
    UART: usart::BasicInstance,
    DMA: usart::RxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> = impl Future;

impl<
        'a,
        UART: usart::BasicInstance,
        DMA: usart::RxDma<UART>,
        const LENGTH: usize,
        const DEPTH: usize,
    > UartReadQueue<UART, DMA, LENGTH, DEPTH>
{
    pub const fn new(buffers: &'static[SyncUnsafeCell<Buffer<LENGTH>>; DEPTH], uart_mutex: Mutex<CriticalSectionRawMutex, bool>, uart_config_update_signal: Signal<CriticalSectionRawMutex, bool>) -> Self {
        Self {
            uart_mutex: uart_mutex,
            uart_config_update_signal: uart_config_update_signal,
            queue_rx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn read_task(
        &'static self,
        queue_rx: &'static Queue<LENGTH, DEPTH>,
        mut rx: UartRx<'static, UART, Async>,
    ) -> ReadTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            let mut block_for_config_update = false;

            loop {                
                /*
                 * Multitasking Logic:
                 * 
                 * 
                 */

                if block_for_config_update {
                    // block until we receive a signal telling to unpause the task because a config update is not active
                    while self.uart_config_update_signal.wait().await { }
                    defmt::trace!("UartReadQueue - standing down from rx thread pause for config update.");

                    block_for_config_update = false;
                }

                let mut buf = queue_rx.enqueue().await.unwrap();

                {
                    let _rw_tasks_config_lock = self.uart_mutex.lock().await;

                    // NOTE: this really shouldn't be a timeout, it should be a signal from tx side that a new config
                    // is desired. This works for now but the timeout is hacky.
                    match select(rx.read_until_idle(buf.data()), self.uart_config_update_signal.wait()).await {
                        Either::First(len) => {
                            if let Ok(len) = len {
                                if len == 0 {
                                    defmt::debug!("uart zero");
                                    buf.cancel();
                                } else {
                                    *buf.len() = len;
                                }
                            } else {
                                // Framing and Parity Error occur here
                                defmt::warn!("{}", len);
                                buf.cancel();
                            }
                        },
                        Either::Second(block_for_config) => {
                            defmt::trace!("UartReadQueue - read to idle cancelled to update config.");
                            buf.cancel();

                            block_for_config_update = block_for_config;

                            if !block_for_config {
                                defmt::warn!("UartReadQueue - config update standdown cancelled read to idle. Should this event sequence occur?");
                            }
                        }
                    }
                } // frees the inter-task uart config lock
            }
        }
    }

    pub fn spawn_task(
        &'static self,
        rx: UartRx<'static, UART, Async>,
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.read_task(&self.queue_rx, rx))
    }

    pub fn try_dequeue(&self) -> Result<DequeueRef<LENGTH, DEPTH>, Error> {
        return self.queue_rx.try_dequeue();
    }

    pub async fn dequeue<RET>(&self, fn_write: impl FnOnce(&[u8]) -> RET) -> RET {
        let buf = self.queue_rx.dequeue().await.unwrap();
        fn_write(buf.data())
    }
}

pub struct UartWriteQueue<
    UART: usart::BasicInstance,
    DMA: usart::TxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    uart_mutex: Mutex<CriticalSectionRawMutex, bool>,
    uart_config_update_signal: Signal<CriticalSectionRawMutex, bool>,
    uart_config_applied_update_signal: Signal<CriticalSectionRawMutex, bool>,
    queue_tx: Queue<LENGTH, DEPTH>,
    has_new_uart_config: AtomicBool,
    new_uart_config: Mutex<CriticalSectionRawMutex, Option<usart::Config>>,
    task: TaskStorage<WriteTaskFuture<UART, DMA, LENGTH, DEPTH>>,
}

type WriteTaskFuture<
    UART: usart::BasicInstance,
    DMA: usart::TxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> = impl Future;

impl<
        'a,
        UART: usart::BasicInstance,
        DMA: usart::TxDma<UART>,
        const LENGTH: usize,
        const DEPTH: usize,
    > UartWriteQueue<UART, DMA, LENGTH, DEPTH>
{
    pub const fn new(buffers: &'static [SyncUnsafeCell<Buffer<LENGTH>>; DEPTH],  uart_mutex: Mutex<CriticalSectionRawMutex, bool>, uart_config_update_signal: Signal<CriticalSectionRawMutex, bool>) -> Self {
        Self {
            uart_mutex: uart_mutex,
            uart_config_update_signal: uart_config_update_signal,
            uart_config_applied_update_signal: Signal::new(),
            queue_tx: Queue::new(buffers),
            has_new_uart_config: AtomicBool::new(false),
            new_uart_config: Mutex::new(None),
            task: TaskStorage::new(),
        }
    }

    fn write_task(
        &'static self,
        queue_tx: &'static Queue<LENGTH, DEPTH>,
        mut tx: UartTx<'static, UART, Async>,
    ) -> WriteTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let buf = queue_tx.dequeue().await.unwrap();

                if self.has_new_uart_config.load(Ordering::Relaxed) {
                    // acquire the lock on the config
                    let new_config = self.new_uart_config.lock().await;

                    // rx task probably has a uart read_to_idle pending, meaning it also holds the uart_mutex lock
                    // signal the rx task to break the read_to_idle and spin waiting for a resume signal
                    self.uart_config_update_signal.signal(true);

                    // rx task is now idling an will shortly release the uart_mutex
                    // acquire the lock on the shared UART config
                    let _rw_tasks_config_lock = self.uart_mutex.lock().await;

                    // now that we have exclusive control over the shared hardware, update the config

                    let config_res = tx.set_config(&new_config.unwrap());
                    if config_res.is_err() {
                        defmt::warn!("failed to apply uart config in uart write queue");
                    } else {
                        defmt::debug!("updated config in uart write queue");
                    }

                    // now that the config has taken hold
                    // tell the rx task to resume
                    self.uart_config_update_signal.signal(false);

                    // clear the update pending flag
                    self.has_new_uart_config.store(false, Ordering::Relaxed);

                    // signal the async task that requested the config update it can resume
                    // knowing the config has been applied
                    self.uart_config_applied_update_signal.signal(true);
                } // frees the config update lock and the inter-task uart config lock

                // defmt::info!("invoking API write");
                tx.write(buf.data()).await.unwrap(); // we are blocked here!
                // defmt::info!("passed API write");

                drop(buf);

                // NOTE: we used to check for DMA transaction complete here, but embassy added
                // it some time ago. Doing it twice causes lockup. 
            }
        }
    }

    pub fn spawn_task(&'static self, tx: UartTx<'static, UART, Async>) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.write_task(&self.queue_tx, tx))
    }

    pub fn enqueue(&self, fn_write: impl FnOnce(&mut [u8]) -> usize) -> Result<(), queue::Error> {
        let mut buf = self.queue_tx.try_enqueue()?;
        let len = fn_write(buf.data());
        *buf.len() = len;
        Ok(())
    }

    pub fn enqueue_copy(&self, source: &[u8]) -> Result<(), queue::Error> {
        self.enqueue(|dest| {
            dest[..source.len()].copy_from_slice(source);
            source.len()
        })
    }

    pub async fn update_uart_config(&self, config: usart::Config) {
        {
            let mut new_config = self.new_uart_config.lock().await;
            let _ = new_config.insert(config);
        }
        self.has_new_uart_config.store(true, Ordering::Relaxed);

        self.uart_config_applied_update_signal.wait().await;
    }
}

pub trait Reader {
    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> impl core::future::Future<Output = Result<RET, ()>>;
}

pub trait Writer {
    fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> impl core::future::Future<Output = Result<(), ()>>;
}

impl<
        UART: usart::BasicInstance,
        Dma: usart::RxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Reader for UartReadQueue<UART, Dma, LEN, DEPTH>
{
    async fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> Result<RET, ()> {
        Ok(self.dequeue(|buf| fn_read(buf)).await)
    }
}

impl<
        UART: usart::BasicInstance,
        Dma: usart::TxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Writer for UartWriteQueue<UART, Dma, LEN, DEPTH>
{
    async fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> Result<(), ()> {
        self.enqueue(|buf| fn_write(buf)).or(Err(()))
    }
}
