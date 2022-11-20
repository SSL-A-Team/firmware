use crate::queue::{self, Buffer, Queue};

use core::future::Future;
use embassy_executor::{raw::TaskStorage, SpawnToken};
use embassy_stm32::{
    usart::{self, UartRx, UartTx},
    Peripheral,
};

pub struct UartReadQueue<
    'a,
    UART: usart::BasicInstance,
    DMA: usart::RxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    queue_rx: Queue<'a, LENGTH, DEPTH>,
    task: TaskStorage<ReadTaskFuture<UART, DMA, LENGTH, DEPTH>>,
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
    > UartReadQueue<'a, UART, DMA, LENGTH, DEPTH>
{
    pub const fn new(buffers: &'a mut [Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            queue_rx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn read_task(
        queue_rx: &'static Queue<'a, LENGTH, DEPTH>,
        mut rx: UartRx<'a, UART, DMA>,
        mut int: UART::Interrupt,
    ) -> ReadTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let mut buf = queue_rx.enqueue().await.unwrap();
                let len = rx
                    .read_to_idle(buf.data(), (&mut int).into_ref())
                    .await
                    .unwrap();
                *buf.len() = len;
            }
        }
    }

    pub fn spawn_task(
        &'static self,
        rx: UartRx<'a, UART, DMA>,
        int: UART::Interrupt,
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| Self::read_task(&self.queue_rx, rx, int))
    }

    pub async fn dequeue<RET>(&self, fn_write: impl FnOnce(&[u8]) -> RET) -> RET {
        let buf = self.queue_rx.dequeue().await.unwrap();
        fn_write(buf.data())
    }
}

pub struct UartWriteQueue<
    'a,
    UART: usart::BasicInstance,
    DMA: usart::TxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    queue_tx: Queue<'a, LENGTH, DEPTH>,
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
    > UartWriteQueue<'a, UART, DMA, LENGTH, DEPTH>
{
    pub const fn new(buffers: &'a mut [Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            queue_tx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn write_task(
        queue_tx: &'static Queue<'a, LENGTH, DEPTH>,
        mut tx: UartTx<'a, UART, DMA>,
    ) -> WriteTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let buf = queue_tx.dequeue().await.unwrap();
                tx.write(buf.data()).await.unwrap();
                drop(buf);
                unsafe {
                    // TODO: what does this do again?
                    while !UART::regs().isr().read().tc() {}
                    UART::regs().cr1().modify(|w| w.set_te(false));
                    while UART::regs().isr().read().teack() {}
                    UART::regs().cr1().modify(|w| w.set_te(true));
                }
            }
        }
    }

    pub fn spawn_task(&'static self, tx: UartTx<'a, UART, DMA>) -> SpawnToken<impl Sized> {
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

pub trait Reader<'a> {
    type F<RET, FN: FnOnce(&[u8]) -> RET>: Future<Output = Result<RET, ()>>
    where
        Self: 'a;

    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN>;
}

pub trait Writer<'a> {
    type F<FN: FnOnce(&mut [u8]) -> usize>: Future<Output = Result<(), ()>>
    where
        Self: 'a;

    fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN>;
}

impl<
        'a,
        UART: usart::BasicInstance,
        Dma: usart::RxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Reader<'a> for crate::uart_queue::UartReadQueue<'a, UART, Dma, LEN, DEPTH>
{
    type F<RET, FN: FnOnce(&[u8]) -> RET> = impl Future<Output = Result<RET, ()>> where Self: 'a;

    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&'a self, fn_read: FN) -> Self::F<RET, FN> {
        async { Ok(self.dequeue(|buf| fn_read(buf)).await) }
    }
}

impl<
        'a,
        UART: usart::BasicInstance,
        Dma: usart::TxDma<UART>,
        const LEN: usize,
        const DEPTH: usize,
    > Writer<'a> for crate::uart_queue::UartWriteQueue<'a, UART, Dma, LEN, DEPTH>
{
    type F<FN: FnOnce(&mut [u8]) -> usize> = impl Future<Output = Result<(), ()>> where Self: 'a;

    fn write<FN: FnOnce(&mut [u8]) -> usize>(&'a self, fn_write: FN) -> Self::F<FN> {
        async { self.enqueue(|buf| fn_write(buf)).or(Err(())) }
    }
}
