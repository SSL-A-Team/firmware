use crate::queue;

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
    queue_rx: queue::Queue<'a, LENGTH, DEPTH>,
    task: TaskStorage<ReadTaskFuture<UART, DMA, LENGTH, DEPTH>>,
}

type ReadTaskFuture<
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
    pub const fn new(buffers: &'a mut [queue::Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            queue_rx: queue::Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn read_task(
        queue_rx: &'static queue::Queue<'a, LENGTH, DEPTH>,
        mut rx: UartRx<'a, UART, DMA>,
        mut int: UART::Interrupt,
    ) -> ReadTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let rx_ref = &mut rx;
                let int_ref = (&mut int).into_ref();
                queue_rx
                    .enqueue(async move |buf2| rx_ref.read_to_idle(buf2, int_ref).await.unwrap())
                    .await;
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

    pub async fn dequeue<F: Future>(
        &'a self,
        fn_write: impl FnOnce(&'a [u8]) -> F,
    ) -> <F as Future>::Output {
        self.queue_rx.dequeue(fn_write).await
    }
}

pub struct UartWriteQueue<
    'a,
    UART: usart::BasicInstance,
    DMA: usart::TxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    queue_tx: queue::Queue<'a, LENGTH, DEPTH>,
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
    pub const fn new(buffers: &'a mut [queue::Buffer<LENGTH>; DEPTH]) -> Self {
        Self {
            queue_tx: queue::Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn write_task(
        queue_tx: &'static queue::Queue<'a, LENGTH, DEPTH>,
        mut tx: UartTx<'a, UART, DMA>,
    ) -> WriteTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                let tx_ref = &mut tx;
                queue_tx
                    .dequeue(async move |buf2| {
                        tx_ref.write(buf2).await.unwrap();
                    })
                    .await;
                unsafe {
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

    pub fn enqueue(&self, source: &[u8]) -> Result<(), queue::Error> {
        self.queue_tx.try_enqueue_copy(source)
    }
}
