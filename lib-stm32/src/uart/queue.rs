#![warn(async_fn_in_trait)]

use core::future::Future;

use embassy_stm32::{
    mode::Async,
    usart::{self, UartRx, UartTx}
};

use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::{Mutex, MutexGuard},
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult},
};

use crate::queue::{
    self,
    DequeueRef, 
    Error,
    Queue
};

#[macro_export]
macro_rules! static_idle_buffered_uart_nl {
    ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr) => {
    // ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr) => {
        $crate::paste::paste! {
            // $(#[$m])*
            static [<$name:upper _TASK_SYNC_MUTEX>]: $crate::uart::queue::IdleBufferedUartTaskSyncMutex =  embassy_sync::mutex::Mutex::new(false);
            // $(#[$m])*
            static [<$name:upper _IDLE_BUFFERED_UART>]: $crate::uart::queue::IdleBufferedUart<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth> = 
                $crate::uart::queue::IdleBufferedUart::new($crate::queue::Queue::new(), $crate::queue::Queue::new(), &[<$name:upper _TASK_SYNC_MUTEX>]);
                // $crate::uart::queue::IdleBufferedUart::new(&[<$name:upper _TASK_SYNC_MUTEX>]);


            static [<$name:upper _READ_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartReadFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth>> = 
                embassy_executor::raw::TaskStorage::new();
            static [<$name:upper _WRITE_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartWriteFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth>> = 
                embassy_executor::raw::TaskStorage::new();
        }
    }
}

#[macro_export]
macro_rules! static_idle_buffered_uart {
    ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr, $(#[$m:meta])*) => {
    // ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr) => {
        $crate::paste::paste! {
            // $(#[$m])*
            static [<$name:upper _TASK_SYNC_MUTEX>]: $crate::uart::queue::IdleBufferedUartTaskSyncMutex =  embassy_sync::mutex::Mutex::new(false);
            // $(#[$m])*
            static [<$name:upper _IDLE_BUFFERED_UART>]: $crate::uart::queue::IdleBufferedUart<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth> = 
                $crate::uart::queue::IdleBufferedUart::new($crate::queue::Queue::new(), $crate::queue::Queue::new(), &[<$name:upper _TASK_SYNC_MUTEX>]);
                // $crate::uart::queue::IdleBufferedUart::new(&[<$name:upper _TASK_SYNC_MUTEX>]);


            static [<$name:upper _READ_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartReadFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth>> = 
                embassy_executor::raw::TaskStorage::new();
            static [<$name:upper _WRITE_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartWriteFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth>> = 
                embassy_executor::raw::TaskStorage::new();
        }
    }
}

#[macro_export]
macro_rules! idle_buffered_uart_read_task {
    ($name:ident, $uart_rx:ident) => {
        $crate::paste::item! {
            [<$name:upper _READ_TASK_STORAGE>].spawn(|| { [<$name:upper _IDLE_BUFFERED_UART>].read_task($uart_rx) } )
        }
    };
}

#[macro_export]
macro_rules! idle_buffered_uart_write_task {
    ($name:ident, $uart_tx:ident) => {
        $crate::paste::item! {
            [<$name:upper _WRITE_TASK_STORAGE>].spawn(|| { [<$name:upper _IDLE_BUFFERED_UART>].write_task($uart_tx) })
        }
    };
}

#[macro_export]
macro_rules! idle_buffered_uart_spawn_tasks {
    ($spawner:ident, $name:ident, $uart:ident) => {
        $crate::paste::item! {
            let ([<$name:lower _uart_tx>], [<$name:lower _uart_rx>]) = Uart::split($uart);
            $spawner.spawn($crate::idle_buffered_uart_read_task!($name, [<$name:lower _uart_rx>])).unwrap();
            $spawner.spawn($crate::idle_buffered_uart_write_task!($name, [<$name:lower _uart_tx>])).unwrap();
        }
    };
}

pub type IdleBufferedUartTaskSyncMutex = Mutex<CriticalSectionRawMutex, bool>;

type UartQueueSyncPubSub = PubSubChannel<CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;
type UartQueueConfigSyncPub = Publisher<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;
type UartQueueConfigSyncSub = Subscriber<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 3, 2>;

pub type IdleBufferedUartReadFuture<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
> = impl Future;

pub type IdleBufferedUartWriteFuture<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
> = impl Future;

pub type ReadTaskFuture<
    const LENGTH: usize,
    const DEPTH: usize,
> = impl Future;

pub type WriteTaskFuture<
    const LENGTH: usize,
    const DEPTH: usize,
> = impl Future;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum UartTaskCommand {
    Pause,
    UnpauseSuccess,
    UnpauseFailure,
}

pub struct IdleBufferedUart<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
> {
    uart_read_queue: UartReadQueue<RLEN, RDEPTH>,
    uart_write_queue: UartWriteQueue<WLEN, WDEPTH>,
    uart_config: Mutex<CriticalSectionRawMutex, Option<usart::Config>>,
    uart_config_signal: UartQueueSyncPubSub,
    uart_config_signal_publisher: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncPub>>,
    uart_config_signal_subscriber: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncSub>>,
}

impl <
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
    > IdleBufferedUart<RLEN, RDEPTH, WLEN, WDEPTH> 
{
    pub const fn new(
        read_queue: Queue<RLEN, RDEPTH>,
        write_queue: Queue<WLEN, WDEPTH>,
        uart_task_mutex: &'static IdleBufferedUartTaskSyncMutex,
    ) -> Self {
        IdleBufferedUart { 
            uart_read_queue: UartReadQueue::new(read_queue, uart_task_mutex),
            uart_write_queue: UartWriteQueue::new(write_queue, uart_task_mutex),
            uart_config: Mutex::new(None),
            uart_config_signal: PubSubChannel::new(),
            uart_config_signal_publisher: Mutex::new(None),
            uart_config_signal_subscriber: Mutex::new(None),
        }
    }

    pub fn get_uart_read_queue(&'static self) -> &'static UartReadQueue<RLEN, RDEPTH> {
        &self.uart_read_queue
    }

    pub fn read_task(
        &'static self,
        rx: UartRx<'static, Async>,
    ) -> IdleBufferedUartReadFuture<RLEN, RDEPTH, WLEN, WDEPTH> {
        async move {
            self.uart_read_queue.read_task(rx, self.uart_config_signal.subscriber().unwrap()).await
        }
    }

    pub fn get_uart_write_queue(&'static self) -> &'static UartWriteQueue<WLEN, WDEPTH> {
        &self.uart_write_queue
    }

    pub fn write_task(
        &'static self,
        tx: UartTx<'static, Async>,
    ) -> IdleBufferedUartWriteFuture<RLEN, RDEPTH, WLEN, WDEPTH> {
        async move {
            self.uart_write_queue.write_task(tx, self.uart_config_signal.publisher().unwrap(), self.uart_config_signal.subscriber().unwrap()).await
        }
    }

    pub fn init(&'static self) {
        // defmt::info!("init write queue");
        // self.uart_write_queue.attach_pubsub(
        //     self.uart_config_signal.publisher().unwrap(),
        //     self.uart_config_signal.subscriber().unwrap()).await;

        defmt::info!("init pub");
        {   
            if let Ok(mut p) = self.uart_config_signal_publisher.try_lock() {
                defmt::info!("set p");
                p.replace(self.uart_config_signal.publisher().unwrap());
            }
        }

        defmt::info!("init sub");
        {
            if let Ok(mut s) = self.uart_config_signal_subscriber.try_lock() {
                defmt::info!("set s");
                s.replace(self.uart_config_signal.subscriber().unwrap());
            }
        }
    }

    pub async fn update_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        // acquire the config lock and insert the config
        {
            let mut new_config = self.uart_config.lock().await;
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

        ret_val
    }
}

pub struct UartReadQueue<
    const LENGTH: usize,
    const DEPTH: usize,
> {
    uart_mutex: &'static Mutex<CriticalSectionRawMutex, bool>,
    queue_rx: Queue<LENGTH, DEPTH>,
}

impl<
        const LENGTH: usize,
        const DEPTH: usize,
    > UartReadQueue<LENGTH, DEPTH>
{
    pub const fn new(
            queue: Queue<LENGTH, DEPTH>,
            uart_mutex: &'static Mutex<CriticalSectionRawMutex, bool>) -> Self {
        Self {
            uart_mutex,
            queue_rx: queue,
        }
    }

    pub fn read_task(
        &'static self,
        // queue_rx: &'static Queue<LENGTH, DEPTH>,
        mut rx: UartRx<'static, Async>,
        mut uart_config_signal_subscriber: UartQueueConfigSyncSub,
    ) -> ReadTaskFuture<LENGTH, DEPTH> {
        async move {
            // if you panic here you spawned multiple ReadQueues from the same instance
            // that isn't allowed
            let mut pause_task_for_config_update = false;

            // look like this might be taking too long to acquire between read_to_idle calls
            // need to acquire this upfront and be smarter about releasing and reacq it
            let mut rw_tasks_config_lock: Option<MutexGuard<'static, CriticalSectionRawMutex, bool>> = Some(self.uart_mutex.lock().await);

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
                let mut buf = self.queue_rx.enqueue().await.unwrap();
                match select(rx.read_until_idle(buf.data()), uart_config_signal_subscriber.next_message()).await {
                    Either::First(len) => {
                        if let Ok(len) = len {
                            if len == 0 {
                                // defmt::debug!("uart zero");
                                buf.cancel();
                            } else {
                                *buf.len() = len;
                            }
                        } else {
                            // Framing and Parity Error occur here
                            // defmt::warn!("{}", len);
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



    pub fn can_dequque(&self) -> bool {
        self.queue_rx.can_dequeue()
    }

    pub fn try_dequeue(&self) -> Result<DequeueRef<LENGTH, DEPTH>, Error> {
        self.queue_rx.try_dequeue()
    }

    pub async fn dequeue<RET>(&self, fn_write: impl FnOnce(&[u8]) -> RET) -> RET {
        let buf = self.queue_rx.dequeue().await.unwrap();
        fn_write(buf.data())
    }
}

pub struct UartWriteQueue<
    const LENGTH: usize,
    const DEPTH: usize,
> {
    uart_mutex: &'static Mutex<CriticalSectionRawMutex, bool>,
    // uart_config_signal_publisher: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncPub>>,
    // uart_config_signal_subscriber: Mutex<CriticalSectionRawMutex, Option<UartQueueConfigSyncSub>>,
    queue_tx: Queue<LENGTH, DEPTH>,
    new_uart_config: Mutex<CriticalSectionRawMutex, Option<usart::Config>>,
}

impl<
        const LENGTH: usize,
        const DEPTH: usize,
    > UartWriteQueue<LENGTH, DEPTH>
{
    pub const fn new(
            queue: Queue<LENGTH, DEPTH>,
            uart_mutex: &'static Mutex<CriticalSectionRawMutex, bool>,
            ) -> Self {
        Self {
            uart_mutex,
            // uart_config_signal_publisher: Mutex::new(None),
            // uart_config_signal_subscriber: Mutex::new(None),
            queue_tx: queue,
            new_uart_config: Mutex::new(None),
        }
    }

    // pub async fn attach_pubsub(&self,
    //     uart_config_signal_publisher: UartQueueConfigSyncPub,
    //     uart_config_signal_subscriber: UartQueueConfigSyncSub
    // ) {
    //     let mut pb = self.uart_config_signal_publisher.lock().await;
    //     *pb = Some(uart_config_signal_publisher);

    //     let mut sub = self.uart_config_signal_subscriber.lock().await;
    //     *sub = Some(uart_config_signal_subscriber);
    // }

    pub fn write_task(
        &'static self,
        mut tx: UartTx<'static, Async>,
        uart_config_signal_publisher: UartQueueConfigSyncPub,
        mut uart_config_signal_subscriber: UartQueueConfigSyncSub,
    ) -> WriteTaskFuture<LENGTH, DEPTH> {
        async move {
            loop {
                // the tx task primarily blocks on queue_tx.queue(), e.g. waiting for other async tasks
                // to enqueue data. Use a select to break waiting if another task signals there's a UART
                // config update. They probably want the update before the next data is enqueued
                match select(self.queue_tx.dequeue(), uart_config_signal_subscriber.next_message()).await {
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
    fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> impl core::future::Future<Output = Result<RET, ()>>;
}

pub trait Writer {
    fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> impl core::future::Future<Output = Result<(), ()>>;
}

impl<
        const LEN: usize,
        const DEPTH: usize,
    > Reader for UartReadQueue<LEN, DEPTH>
{
    async fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> Result<RET, ()> {
        Ok(self.dequeue(|buf| fn_read(buf)).await)
    }
}

impl<
        const LEN: usize,
        const DEPTH: usize,
    > Writer for UartWriteQueue<LEN, DEPTH>
{
    async fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> Result<(), ()> {
        self.enqueue(|buf| fn_write(buf)).or(Err(()))
    }
}
