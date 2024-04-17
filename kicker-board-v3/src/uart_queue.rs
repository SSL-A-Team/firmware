use crate::queue::{self, Buffer, DequeueRef, Error, Queue};

use core::{cell::UnsafeCell, future::Future};
use defmt::info;
use embassy_executor::{raw::TaskStorage, SpawnToken};
use embassy_stm32::{mode::Async, usart::{self, UartRx, UartTx}};

pub struct UartReadQueue<
    UART: usart::BasicInstance,
    DMA: usart::RxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
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
    pub const fn new(buffers: UnsafeCell<[Buffer<LENGTH>; DEPTH]>) -> Self {
        Self {
            queue_rx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn read_task(
        queue_rx: &'static Queue<LENGTH, DEPTH>,
        mut rx: UartRx<'static, UART, Async>,
    ) -> ReadTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let mut buf = queue_rx.enqueue().await.unwrap();
                let len = rx
                    .read_until_idle(buf.data())
                    .await;
                    // .unwrap();
                    // TODO: this
                if let Ok(len) = len {
                    if len == 0 {
                        info!("uart zero");
                        buf.cancel();
                    } else {
                        *buf.len() = len;
                    }
                } else {
                    info!("{}", len);
                    buf.cancel();
                }
            }
        }
    }

    pub fn spawn_task(
        &'static self,
        rx: UartRx<'static, UART, Async>,
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| Self::read_task(&self.queue_rx, rx))
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
    queue_tx: Queue<LENGTH, DEPTH>,
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
    pub const fn new(buffers: UnsafeCell<[Buffer<LENGTH>; DEPTH]>) -> Self {
        Self {
            queue_tx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn write_task(
        queue_tx: &'static Queue<LENGTH, DEPTH>,
        mut tx: UartTx<'static, UART, Async>,
    ) -> WriteTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let buf = queue_tx.dequeue().await.unwrap();
                // defmt::info!("invoking API write");
                tx.write(buf.data()).await.unwrap(); // we are blocked here!
                // defmt::info!("passed API write");

                drop(buf);
                // unsafe {
                //     // TODO: what does this do again?
                //     while !UART::regs().isr().read().tc() {}
                //     UART::regs().cr1().modify(|w| w.set_te(false));
                //     while UART::regs().isr().read().teack() {}
                //     UART::regs().cr1().modify(|w| w.set_te(true));
                // }
            }
        }
    }

    pub fn spawn_task(&'static self, tx: UartTx<'static, UART, Async>) -> SpawnToken<impl Sized> {
        self.task.spawn(|| Self::write_task(&self.queue_tx, tx))
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
}

pub trait Reader {
    async fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> Result<RET, ()>;
}

pub trait Writer {
    async fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> Result<(), ()>;
}

impl<
        UART: usart::BasicInstance,
        Dma: usart::RxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Reader for crate::uart_queue::UartReadQueue<UART, Dma, LEN, DEPTH>
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
    > Writer for crate::uart_queue::UartWriteQueue<UART, Dma, LEN, DEPTH>
{
    async fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> Result<(), ()> {
        self.enqueue(|buf| fn_write(buf)).or(Err(()))
    }
}
