#![warn(async_fn_in_trait)]

use core::{
    cell::SyncUnsafeCell,
    future::Future,
};

use embassy_stm32::{
    mode::Async,
    usart::{self, UartRx, UartTx}
};
use embassy_executor::{
    raw::TaskStorage,
    SpawnToken};

use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::{Mutex, MutexGuard},
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult},
};
use embassy_time::{Duration, Instant, Timer};

use crate::queue::{
    self,
    Buffer,
    DequeueRef, 
    Error,
    Queue
};

#[macro_export]
macro_rules! make_uart_queue_pair {
    ($name:ident, $uart:ty, $uart_rx_dma:ty, $uart_tx_dma:ty, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr, $(#[$m:meta])*) => {
        $crate::paste::paste! {
        // shared mutex allowing safe runtime update to UART config
        const [<$name _UART_PERI_MUTEX>]: embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, bool> = 
            embassy_sync::mutex::Mutex::new(false);
        static [<$name _UART_SYNC_PUBSUB>]: $crate::uart::queue::UartQueueSyncPubSub = embassy_sync::pubsub::PubSubChannel::new();

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

#[macro_export]
macro_rules! queue_pair_register_signals {
    ($name:ident) => {
        $crate::paste::paste! {
        [<$name _TX_UART_QUEUE>].attach_pubsub(
            [<$name _UART_SYNC_PUBSUB>].publisher().unwrap(),
            [<$name _UART_SYNC_PUBSUB>].subscriber().unwrap()).await;
        }
    }
}

#[macro_export]
macro_rules! queue_pair_rx_task {
    ($name:ident, $uart_rx:ident) => {
        $crate::paste::paste! {
        [<$name _RX_UART_QUEUE>].spawn_task_with_pubsub($uart_rx, &[<$name _UART_SYNC_PUBSUB>])
        }
    }
}

#[macro_export]
macro_rules! queue_pair_tx_task {
    ($name:ident, $uart_tx:ident) => {
        $crate::paste::paste! {
        [<$name _TX_UART_QUEUE>].spawn_task_with_pubsub($uart_tx, &[<$name _UART_SYNC_PUBSUB>])
        }
    }
}

#[macro_export]
macro_rules! queue_pair_register_and_spawn {
    ($spawner:ident, $name:ident, $uart_rx:ident, $uart_tx:ident) => {
        $crate::paste::paste! {
        $crate::queue_pair_register_signals!($name);
        $spawner.spawn($crate::queue_pair_rx_task!($name, $uart_rx)).unwrap();
        $spawner.spawn($crate::queue_pair_tx_task!($name, $uart_tx)).unwrap();
        }
    };
}

pub type UartQueueSyncPubSub = PubSubChannel<CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;
type UartQueueConfigSyncPub = Publisher<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;
type UartQueueConfigSyncSub = Subscriber<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum UartTaskCommand {
    Pause,
    UnpauseSuccess,
    UnpauseFailure,
}

pub struct UartReadQueue<
    UART: usart::BasicInstance,
    DMA: usart::RxDma<UART>,
    const LENGTH: usize,
    const DEPTH: usize,
> {
    uart_mutex: Mutex<CriticalSectionRawMutex, bool>,
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
    pub const fn new(buffers: &'static[SyncUnsafeCell<Buffer<LENGTH>>; DEPTH],
            uart_mutex: Mutex<CriticalSectionRawMutex, bool>        ) -> Self {
        Self {
            uart_mutex: uart_mutex,
            queue_rx: Queue::new(buffers),
            task: TaskStorage::new(),
        }
    }

    fn read_task(
        &'static self,
        queue_rx: &'static Queue<LENGTH, DEPTH>,
        mut rx: UartRx<'static, UART, Async>,
        mut uart_config_signal_subscriber: UartQueueConfigSyncSub,
    ) -> ReadTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            // if you panic here you spawned multiple ReadQueues from the same instance
            // that isn't allowed
            let mut pause_task_for_config_update = false;

            // look like this might be taking too long to acquire between read_to_idle calls
            // need to acquire this upfront and be smarter about releasing and reacq it
            let mut rw_tasks_config_lock: Option<MutexGuard<'static, CriticalSectionRawMutex, bool>> = Some(self.uart_mutex.lock().await);
            let mut time_ended_trx = Instant::now();

            loop {                
                // block if/until we receive a signal telling to unpause the task because a config update is not active
                while pause_task_for_config_update {
                    defmt::trace!("UartReadQueue - pausing rx task for config update");

                    // release the uart hw mutex lock by dropping the MutexGuard
                    drop(rw_tasks_config_lock.take().unwrap());

                    match uart_config_signal_subscriber.next_message().await { 
                        WaitResult::Lagged(amnt) => {
                            defmt::debug!("UartReadQueue - lagged {} processing config signal", amnt)
                        }
                        WaitResult::Message(task_command) => {
                            if task_command == UartTaskCommand::UnpauseSuccess || task_command == UartTaskCommand::UnpauseFailure {
                                defmt::trace!("UartReadQueue - resuming rx thread paused for config update.");
                                pause_task_for_config_update = false;

                                // we are told to resume, reacquire the lock
                                rw_tasks_config_lock = Some(self.uart_mutex.lock().await);
                            }
                        }
                    }
                } 


                // get enqueue ref to pass to the DMA layer
                let mut buf = queue_rx.enqueue().await.unwrap();
                let read_to_idle_start_time = Instant::now();
                match select(rx.read_until_idle(buf.data()), uart_config_signal_subscriber.next_message()).await {
                    Either::First(len) => {
                        defmt::info!("elapsed time to process and restart read_to_idle was: {}", read_to_idle_start_time - time_ended_trx);

                        time_ended_trx = Instant::now();

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
                    Either::Second(config_signal_result) => {
                        defmt::trace!("UartReadQueue - read to idle cancelled to update config.");
                        // clear the buffer record keeping, the transaction may have been interrupted
                        buf.cancel();

                        match config_signal_result {
                            WaitResult::Lagged(amnt) => {
                                defmt::trace!("UartReadQueue - lagged {} processing config update signal while blocked on read_to_idle", amnt);
                            }
                            WaitResult::Message(task_command) => {
                                if task_command == UartTaskCommand::Pause {
                                    pause_task_for_config_update = true;
                                } else {
                                    defmt::warn!("UartReadQueue - config update standdown cancelled read to idle. Should this event sequence occur?");
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn spawn_task(
        &'static self,
        rx: UartRx<'static, UART, Async>,
        uart_config_signal_subscriber: UartQueueConfigSyncSub
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.read_task(&self.queue_rx, rx, uart_config_signal_subscriber))
    }

    pub fn spawn_task_with_pubsub(
        &'static self,
        rx: UartRx<'static, UART, Async>,
        uart_config_signal_pubsub: &'static UartQueueSyncPubSub
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.read_task(&self.queue_rx, rx, uart_config_signal_pubsub.subscriber().unwrap()))
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
    uart_config_signal_publisher: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncPub>>,
    uart_config_signal_subscriber: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncSub>>,
    queue_tx: Queue<LENGTH, DEPTH>,
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
    pub const fn new(buffers: &'static [SyncUnsafeCell<Buffer<LENGTH>>; DEPTH],
            uart_mutex: Mutex<CriticalSectionRawMutex, bool>
            ) -> Self {
        Self {
            uart_mutex: uart_mutex,
            uart_config_signal_publisher: Mutex::new(None),
            uart_config_signal_subscriber: Mutex::new(None),
            queue_tx: Queue::new(buffers),
            new_uart_config: Mutex::new(None),
            task: TaskStorage::new(),
        }
    }

    pub async fn attach_pubsub(&self,
        uart_config_signal_publisher: UartQueueConfigSyncPub,
        uart_config_signal_subscriber: UartQueueConfigSyncSub
    ) {
        let mut pb = self.uart_config_signal_publisher.lock().await;
        *pb = Some(uart_config_signal_publisher);

        let mut sub = self.uart_config_signal_subscriber.lock().await;
        *sub = Some(uart_config_signal_subscriber);
    }

    fn write_task(
        &'static self,
        queue_tx: &'static Queue<LENGTH, DEPTH>,
        mut tx: UartTx<'static, UART, Async>,
        uart_config_signal_publisher: UartQueueConfigSyncPub,
        mut uart_config_signal_subscriber: UartQueueConfigSyncSub,
    ) -> WriteTaskFuture<UART, DMA, LENGTH, DEPTH> {
        async move {
            loop {
                // the tx task primarily blocks on queue_tx.queue(), e.g. waiting for other async tasks
                // to enqueue data. Use a select to break waiting if another task signals there's a UART
                // config update. They probably want the update before the next data is enqueued
                match select(queue_tx.dequeue(), uart_config_signal_subscriber.next_message()).await {
                    // we are dequeing data
                    Either::First(dq_res) => {
                        if let Ok(buf) = dq_res {
                            // write the message to the DMA
                            tx.write(buf.data()).await.unwrap(); // we are blocked here!

                            // drop the buffer, cleans up queue
                            drop(buf);

                            // NOTE: we used to check for DMA transaction complete here, but embassy added
                            // it some time ago. Doing it twice causes lockup. 
                        } else {
                            defmt::warn!("UartWriteQueue - result of dequeue for DMA TX was error");
                        }
                    }
                    // we are processing a request to update the UART hardware config
                    Either::Second(config_signal_res) => {
                        match config_signal_res {
                            WaitResult::Lagged(amnt) => {
                                defmt::trace!("UartWriteQueue - lagged {} while processing signal to pause for config update", amnt);
                            }
                            WaitResult::Message(task_command) => {
                                // we received a task command, possible asking us to pasue for a new config 
                                let mut success = false;
                                if task_command == UartTaskCommand::Pause {
                                    { // open a scope so we can explicitly track the config mutex lifetime
                                        // acquire the lock on the config
                                        defmt::debug!("UartWriteQueue - waiting for config lock");
                                        let mut new_config = self.new_uart_config.lock().await;

                                        // rx task should have also received the pause request and is now (or will be shortly) 
                                        // idling and will release the uart config mutex which it nominally holds during read_to_idle
                                        // we can now acquire the lock on the shared UART config
                                        defmt::debug!("UartWriteQueue - waiting for hardware lock");
                                        let _rw_tasks_config_lock = self.uart_mutex.lock().await;
                                    
                                        // now that we have exclusive control over the shared hardware, update the config
                                        defmt::debug!("UartWriteQueue - applying new hardware config");
                                        if new_config.is_some() {
                                            let config_res = tx.set_config(&new_config.unwrap());
                                            if config_res.is_err() {
                                                defmt::warn!("UartWriteQueue - failed to apply uart config in uart write queue");
                                            } else {
                                                defmt::debug!("UartWriteQueue - updated config in uart write queue");
                                                success = true;
                                            }
                                        } else {
                                            defmt::warn!("UartWriteQueue - task was signaled to update config but none was provided.")
                                        }

                                        // clear config
                                        *new_config = None;
                                    } // free uart hardware config and new config mutexes

                                    defmt::debug!("UartWriteQueue - released locks");

                                    // we've either succeeded or not succeeded
                                    // regardless, clean up by signaling other tasks to unpause
                                    if success {
                                        uart_config_signal_publisher.publish(UartTaskCommand::UnpauseSuccess).await;
                                    } else {
                                        uart_config_signal_publisher.publish(UartTaskCommand::UnpauseFailure).await;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn spawn_task(&'static self, 
        tx: UartTx<'static, UART, Async>,
        uart_config_signal_publisher: UartQueueConfigSyncPub,
        uart_config_signal_subscriber: UartQueueConfigSyncSub,
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.write_task(&self.queue_tx, tx, uart_config_signal_publisher, uart_config_signal_subscriber))
    }

    pub fn spawn_task_with_pubsub(&'static self, 
        tx: UartTx<'static, UART, Async>,
        uart_config_signal_pubsub: &'static UartQueueSyncPubSub
    ) -> SpawnToken<impl Sized> {
        self.task.spawn(|| self.write_task(&self.queue_tx, tx, uart_config_signal_pubsub.publisher().unwrap(), uart_config_signal_pubsub.subscriber().unwrap()))
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

    pub async fn update_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        // acquire the config lock and insert the config
        {
            let mut new_config = self.new_uart_config.lock().await;
            let _ = new_config.insert(config);
        } // drop config lock

        // signal the tasks to pause and apply the config before the next write
        // tx task will acquire the config lock and do the application once the rx task
        // releases the hardware uart lock predominantly held by read_to_idle.
        {
            let config_signal_pub = self.uart_config_signal_publisher.lock().await;
            config_signal_pub.as_ref().unwrap().publish(UartTaskCommand::Pause).await;
        }

        // multiple tasks (not the queues) could call this from a multi-prio context
        // so we need to acquire the lock on the subscriber
        #[allow(unused_assignments)] // value isn't read but is retunred
        let mut ret_val: Result<(), ()> = Err(());
        {
            let mut success_subscriber = self.uart_config_signal_subscriber.lock().await;
            
            // wait for tasks to indicate success
            loop {
                let success_result = success_subscriber.as_mut().unwrap().next_message().await;
                match success_result { 
                    WaitResult::Lagged(amnt) => {
                        defmt::debug!("UartQueue - lagged {} waiting for status response from config update", amnt);
                    }
                    WaitResult::Message(task_command_reply) => {
                        if task_command_reply == UartTaskCommand::Pause {
                            // we are probably back processing our own command to Pause
                            defmt::debug!("UartQueue - received spurious value waiting for response");
                        }

                        // tx thread will release locks and send the Unpause command indicating success
                        if task_command_reply == UartTaskCommand::UnpauseSuccess {
                            ret_val = Ok(());
                            break;
                        }
                    }
                }
            }
        } // subscriber lock freed here

        return ret_val;
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
