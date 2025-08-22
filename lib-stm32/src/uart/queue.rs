#![warn(async_fn_in_trait)]

use core::{
    future::Future,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_stm32::{
    mode::Async,
    usart::{self, UartRx, UartTx},
};

use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult},
};
use embassy_time::Timer;

use crate::queue::{self, DequeueRef, Error, Queue};

#[macro_export]
macro_rules! static_idle_buffered_uart_nl {
    ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr, $debug:expr) => {
        $crate::paste::paste! {
            static [<$name _RX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$rx_buffer_size>>; $rx_buffer_depth] =
                [const { core::cell::SyncUnsafeCell::new($crate::queue::Buffer::<$rx_buffer_size>::new()) }; $rx_buffer_depth];
            static [<$name _TX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$tx_buffer_size>>; $tx_buffer_depth] =
                [const { core::cell::SyncUnsafeCell::new($crate::queue::Buffer::<$tx_buffer_size>::new()) }; $tx_buffer_depth];

            static [<$name:upper _IDLE_BUFFERED_UART>]: $crate::uart::queue::IdleBufferedUart<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug> =
                $crate::uart::queue::IdleBufferedUart::new(
                $crate::queue::Queue::new(&[<$name _RX_BUFFER>]),
                $crate::queue::Queue::new(&[<$name _TX_BUFFER>])
            );

            static [<$name:upper _READ_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartReadFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug>> =
                embassy_executor::raw::TaskStorage::new();
            static [<$name:upper _WRITE_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartWriteFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug>> =
                embassy_executor::raw::TaskStorage::new();
        }
    }
}

#[macro_export]
macro_rules! static_idle_buffered_uart {
    ($name:ident, $rx_buffer_size:expr, $rx_buffer_depth:expr, $tx_buffer_size:expr, $tx_buffer_depth:expr, $debug:expr, $(#[$m:meta])*) => {
        $crate::paste::paste! {
            $(#[$m])*
            static [<$name _RX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$rx_buffer_size>>; $rx_buffer_depth] =
                [const { core::cell::SyncUnsafeCell::new($crate::queue::Buffer::<$rx_buffer_size>::new()) }; $rx_buffer_depth];
            $(#[$m])*
            static [<$name _TX_BUFFER>]: [core::cell::SyncUnsafeCell<$crate::queue::Buffer<$tx_buffer_size>>; $tx_buffer_depth] =
                [const { core::cell::SyncUnsafeCell::new($crate::queue::Buffer::<$tx_buffer_size>::new()) }; $tx_buffer_depth];

            static [<$name:upper _IDLE_BUFFERED_UART>]: $crate::uart::queue::IdleBufferedUart<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug> =
                $crate::uart::queue::IdleBufferedUart::new(
                $crate::queue::Queue::new(&[<$name _RX_BUFFER>]),
                $crate::queue::Queue::new(&[<$name _TX_BUFFER>])
            );

            static [<$name:upper _READ_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartReadFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug>> =
                embassy_executor::raw::TaskStorage::new();
            static [<$name:upper _WRITE_TASK_STORAGE>]: embassy_executor::raw::TaskStorage<
                $crate::uart::queue::IdleBufferedUartWriteFuture<$rx_buffer_size, $rx_buffer_depth, $tx_buffer_size, $tx_buffer_depth, $debug>> =
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

type UartQueueSyncCommandPubSub = PubSubChannel<CriticalSectionRawMutex, UartTaskCommand, 1, 2, 1>;
type UartQueueSyncCommandSub =
    Subscriber<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 2, 1>;
type UartQueueSyncCommandPub =
    Publisher<'static, CriticalSectionRawMutex, UartTaskCommand, 1, 2, 1>;

type UartQueueSyncResponsePubSub =
    PubSubChannel<CriticalSectionRawMutex, UartTaskResponse, 2, 1, 2>;
type UartQueueSyncResponseSub =
    Subscriber<'static, CriticalSectionRawMutex, UartTaskResponse, 2, 1, 2>;
type UartQueueSyncResponsePub =
    Publisher<'static, CriticalSectionRawMutex, UartTaskResponse, 2, 1, 2>;

pub type IdleBufferedUartReadFuture<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
    const DEBUG: bool,
> = impl Future;

pub type IdleBufferedUartWriteFuture<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
    const DEBUG: bool,
> = impl Future;

pub type ReadTaskFuture<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool> = impl Future;

pub type WriteTaskFuture<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool> = impl Future;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum UartTaskCommand {
    Pause,
    UpdateConfig(usart::Config),
    Unpause,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum UartTaskResponse {
    ReadTaskAcceptingConfig,
    WriteTaskPaused,
    UpdateSuccessful,
    UpdateFailed,
    ReadTaskRunning,
    WriteTaskRunning,
}

pub struct IdleBufferedUart<
    const RLEN: usize,
    const RDEPTH: usize,
    const WLEN: usize,
    const WDEPTH: usize,
    const DEBUG: bool,
> {
    uart_read_queue: UartReadQueue<RLEN, RDEPTH, DEBUG>,
    uart_write_queue: UartWriteQueue<WLEN, WDEPTH, DEBUG>,
    uart_config_initialized: AtomicBool,
    uart_config_update_in_progress: AtomicBool,
    uart_config_command_pubsub: UartQueueSyncCommandPubSub,
    uart_config_response_pubsub: UartQueueSyncResponsePubSub,
    uart_config_command_publisher: Mutex<CriticalSectionRawMutex, Option<UartQueueSyncCommandPub>>,
    uart_config_response_subscriber:
        Mutex<CriticalSectionRawMutex, Option<UartQueueSyncResponseSub>>,
}

impl<
        const RLEN: usize,
        const RDEPTH: usize,
        const WLEN: usize,
        const WDEPTH: usize,
        const DEBUG: bool,
    > IdleBufferedUart<RLEN, RDEPTH, WLEN, WDEPTH, DEBUG>
{
    pub const fn new(read_queue: Queue<RLEN, RDEPTH>, write_queue: Queue<WLEN, WDEPTH>) -> Self {
        IdleBufferedUart {
            uart_read_queue: UartReadQueue::new(read_queue),
            uart_write_queue: UartWriteQueue::new(write_queue),
            uart_config_initialized: AtomicBool::new(false),
            uart_config_update_in_progress: AtomicBool::new(false),
            uart_config_command_pubsub: PubSubChannel::new(),
            uart_config_response_pubsub: PubSubChannel::new(),
            uart_config_command_publisher: Mutex::new(None),
            uart_config_response_subscriber: Mutex::new(None),
        }
    }

    pub fn get_uart_read_queue(&'static self) -> &'static UartReadQueue<RLEN, RDEPTH, DEBUG> {
        &self.uart_read_queue
    }

    pub fn read_task(
        &'static self,
        rx: UartRx<'static, Async>,
    ) -> IdleBufferedUartReadFuture<RLEN, RDEPTH, WLEN, WDEPTH, DEBUG> {
        async move {
            self.uart_read_queue
                .read_task(
                    rx,
                    self.uart_config_command_pubsub
                        .subscriber()
                        .expect("uart read task command sub failed"),
                    self.uart_config_response_pubsub
                        .publisher()
                        .expect("uart read task reponse pub failed"),
                )
                .await
        }
    }

    pub fn get_uart_write_queue(&'static self) -> &'static UartWriteQueue<WLEN, WDEPTH, DEBUG> {
        &self.uart_write_queue
    }

    pub fn write_task(
        &'static self,
        tx: UartTx<'static, Async>,
    ) -> IdleBufferedUartWriteFuture<RLEN, RDEPTH, WLEN, WDEPTH, DEBUG> {
        async move {
            self.uart_write_queue
                .write_task(
                    tx,
                    self.uart_config_command_pubsub
                        .subscriber()
                        .expect("uart write task command sub failed"),
                    self.uart_config_response_pubsub
                        .publisher()
                        .expect("uart write task reponse pub failed"),
                )
                .await
        }
    }

    pub fn init(&'static self) {
        if let Ok(mut command_pub) = self.uart_config_command_publisher.try_lock() {
            command_pub.replace(self.uart_config_command_pubsub.publisher().expect(
                "uart config command publisher already consumed. Did the init guard fail?",
            ));
        }

        if let Ok(mut response_sub) = self.uart_config_response_subscriber.try_lock() {
            response_sub.replace(self.uart_config_response_pubsub.subscriber().expect(
                "uart config response subscriber already consumed. Did the init guard fail?",
            ));
        }

        self.uart_config_initialized.store(true, Ordering::SeqCst);
    }

    pub async fn update_uart_config(&self, config: usart::Config) -> Result<(), ()> {
        #[cfg(target_has_atomic)]
        if let Err(_) = self.uart_config_update_in_progress.compare_exchange(
            false,
            true,
            Ordering::Acquire,
            Ordering::Relaxed,
        ) {
            return Err(());
        }

        #[cfg(not(target_has_atomic))]
        {
            let mut early_ret = false;
            critical_section::with(|_| {
                let update_in_progress = self.uart_config_update_in_progress.load(Ordering::SeqCst);
                if update_in_progress {
                    early_ret = true;
                } else {
                    self.uart_config_update_in_progress
                        .store(true, Ordering::SeqCst);
                }
            });

            if early_ret {
                return Err(());
            }
        }

        let mut config_update_success = false;

        defmt::debug!("updating uart config");

        // command/response mutex scope
        {
            let mut command_pub_mutex_guard = self.uart_config_command_publisher.lock().await;
            let command_pub = command_pub_mutex_guard
                .as_mut()
                .expect("command pub uninitialized");

            let mut response_sub_mutex_guard = self.uart_config_response_subscriber.lock().await;
            let response_sub = response_sub_mutex_guard
                .as_mut()
                .expect("response sub uninitialized");

            // tell read and write tasks to pause
            defmt::trace!("instructing uart tasks for pause for config update");
            command_pub.publish(UartTaskCommand::Pause).await;

            // wait for confirmation that both are paused
            let mut rx_task_paused = false;
            let mut tx_task_paused = false;
            while !(rx_task_paused && tx_task_paused) {
                match response_sub.next_message_pure().await {
                    UartTaskResponse::ReadTaskAcceptingConfig => {
                        rx_task_paused = true;
                        defmt::trace!("uart read task paused, accepting config update");
                    }
                    UartTaskResponse::WriteTaskPaused => {
                        tx_task_paused = true;
                        defmt::trace!("uart write task paused for config update");
                    }
                    _ => {
                        defmt::warn!(
                            "received spurious value while waiting for uart tasks to pause"
                        );
                    }
                }
            }

            defmt::trace!("uart tasks ready for config update");

            // send new config
            command_pub
                .publish(UartTaskCommand::UpdateConfig(config))
                .await;

            // get confirmation the read queue updated the config
            'config_update_loop: loop {
                match response_sub.next_message_pure().await {
                    UartTaskResponse::UpdateSuccessful => {
                        config_update_success = true;
                        break 'config_update_loop;
                    }
                    UartTaskResponse::UpdateFailed => {
                        break 'config_update_loop;
                    }
                    _ => {
                        defmt::warn!(
                            "received spurious value while waiting for read task to update config"
                        );
                    }
                }
            }

            // tell tasks to unpause
            command_pub.publish(UartTaskCommand::Unpause).await;

            // confirm tasks are running
            let mut rx_task_running = false;
            let mut tx_task_running = false;
            while !(rx_task_running && tx_task_running) {
                match response_sub.next_message_pure().await {
                    UartTaskResponse::ReadTaskRunning => {
                        rx_task_running = true;
                        defmt::trace!("uart read task resumed");
                    }
                    UartTaskResponse::WriteTaskRunning => {
                        tx_task_running = true;
                        defmt::trace!("uart write task resumed");
                    }
                    _ => {
                        defmt::warn!(
                            "received spurious value while waiting for uart tasks to resume"
                        );
                    }
                }
            }

            // pub and sub mutex guards dropped here
        }

        #[cfg(target_has_atomic)]
        if let Err(_) = self.uart_config_update_in_progress.compare_exchange(
            true,
            false,
            Ordering::SeqCst,
            Ordering::Acquire,
        ) {
            defmt::error!(
                "uart config update state became inchoerant. This should not be possible."
            );
        }

        #[cfg(not(target_has_atomic))]
        {
            critical_section::with(|_| {
                let update_in_progress = self.uart_config_update_in_progress.load(Ordering::SeqCst);
                if !update_in_progress {
                    defmt::error!(
                        "uart config update state became inchoerant. This should not be possible."
                    );
                }

                self.uart_config_update_in_progress
                    .store(false, Ordering::SeqCst);
            });
        }

        if config_update_success {
            Ok(())
        } else {
            Err(())
        }

        // // acquire the config lock and insert the config
        // {
        //     let mut new_config = self.uart_config.lock().await;
        //     let _ = new_config.insert(config);
        // } // drop config lock

        // // signal the tasks to pause and apply the config before the next write
        // // tx task will acquire the config lock and do the application once the rx task
        // // releases the hardware uart lock predominantly held by read_to_idle.
        // {
        //     let config_signal_pub = self.uart_config_signal_publisher.lock().await;
        //     config_signal_pub.as_ref().unwrap().publish(UartTaskCommand::Pause).await;
        // }

        // // multiple tasks (not the queues) could call this from a multi-prio context
        // // so we need to acquire the lock on the subscriber
        // #[allow(unused_assignments)] // value isn't read but is retunred
        // let mut ret_val: Result<(), ()> = Err(());
        // {
        //     let mut success_subscriber = self.uart_config_signal_subscriber.lock().await;

        //     // wait for tasks to indicate success
        //     loop {
        //         let success_result = success_subscriber.as_mut().unwrap().next_message().await;
        //         match success_result {
        //             WaitResult::Lagged(amnt) => {
        //                 defmt::debug!("UartQueue - lagged {} waiting for status response from config update", amnt);
        //             }
        //             WaitResult::Message(task_command_reply) => {
        //                 if task_command_reply == UartTaskCommand::Pause {
        //                     // we are probably back processing our own command to Pause
        //                     defmt::debug!("UartQueue - received spurious value waiting for response");
        //                 }

        //                 // tx thread will release locks and send the Unpause command indicating success
        //                 if task_command_reply == UartTaskCommand::UnpauseSuccess {
        //                     ret_val = Ok(());
        //                     break;
        //                 }
        //             }
        //         }
        //     }
        // } // subscriber lock freed here

        // ret_val
    }
}

pub struct UartReadQueue<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool> {
    queue_rx: Queue<LENGTH, DEPTH>,
}

impl<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool>
    UartReadQueue<LENGTH, DEPTH, DEBUG>
{
    pub const fn new(queue: Queue<LENGTH, DEPTH>) -> Self {
        Self { queue_rx: queue }
    }

    fn read_task(
        &'static self,
        // queue_rx: &'static Queue<LENGTH, DEPTH>,
        mut rx: UartRx<'static, Async>,
        mut uart_config_command_subscriber: UartQueueSyncCommandSub,
        uart_config_response_publisher: UartQueueSyncResponsePub,
    ) -> ReadTaskFuture<LENGTH, DEPTH, DEBUG> {
        async move {
            // if you panic here you spawned multiple ReadQueues from the same instance
            // that isn't allowed
            // let mut pause_task_for_config_update = false;

            // look like this might be taking too long to acquire between read_to_idle calls
            // need to acquire this upfront and be smarter about releasing and reacq it
            // let mut rw_tasks_config_lock: Option<MutexGuard<'static, CriticalSectionRawMutex, bool>> = Some(self.uart_mutex.lock().await);

            loop {
                // block if/until we receive a signal telling to unpause the task because a config update is not active
                // while pause_task_for_config_update {
                //     defmt::trace!("UartReadQueue - pausing rx task for config update");

                //     // release the uart hw mutex lock by dropping the MutexGuard
                //     drop(rw_tasks_config_lock.take().unwrap());

                //     match uart_config_signal_subscriber.next_message().await {
                //         WaitResult::Lagged(amnt) => {
                //             defmt::debug!("UartReadQueue - lagged {} processing config signal", amnt)
                //         }
                //         WaitResult::Message(task_command) => {
                //             if task_command == UartTaskCommand::UnpauseSuccess || task_command == UartTaskCommand::UnpauseFailure {
                //                 defmt::trace!("UartReadQueue - resuming rx thread paused for config update.");
                //                 pause_task_for_config_update = false;

                //                 // we are told to resume, reacquire the lock
                //                 rw_tasks_config_lock = Some(self.uart_mutex.lock().await);
                //             }
                //         }
                //     }
                // }

                // get enqueue ref to pass to the DMA layer
                // let mut buf = self.queue_rx.enqueue().await.unwrap();
                // if self.queue_rx.is_full() {
                //     defmt::warn!("a queue is discarding data");
                // }

                let mut buf = self.queue_rx.try_enqueue_override().expect("multiple threads concurrently attempted to bind an internal queue buffer to dma");
                match select(
                    rx.read_until_idle(buf.data()),
                    uart_config_command_subscriber.next_message(),
                )
                .await
                {
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
                    }
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
                                    // send config ready
                                    uart_config_response_publisher
                                        .publish(UartTaskResponse::ReadTaskAcceptingConfig)
                                        .await;

                                    // wait for issuance of new config
                                    if let UartTaskCommand::UpdateConfig(config) =
                                        uart_config_command_subscriber.next_message_pure().await
                                    {
                                        if let Err(_cfg_err) = rx.set_config(&config) {
                                            defmt::error!(
                                                "uart read task config update failed {:?}, {:?}",
                                                _cfg_err as u8,
                                                usart::ConfigError::DataParityNotSupported as u8
                                            );
                                            uart_config_response_publisher
                                                .publish(UartTaskResponse::UpdateFailed)
                                                .await;
                                        } else {
                                            defmt::trace!("uart read task updated config");
                                            uart_config_response_publisher
                                                .publish(UartTaskResponse::UpdateSuccessful)
                                                .await;
                                        }
                                    } else {
                                        defmt::error!("invalid config update command");
                                    };

                                    // wait for command to resume
                                    if let UartTaskCommand::Unpause =
                                        uart_config_command_subscriber.next_message_pure().await
                                    {
                                        defmt::trace!(
                                            "uart read task resuming after config update"
                                        );
                                    } else {
                                        defmt::error!("invalid config resume command");
                                    }

                                    // report resuming
                                    uart_config_response_publisher
                                        .publish(UartTaskResponse::ReadTaskRunning)
                                        .await;
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

pub struct UartWriteQueue<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool> {
    queue_tx: Queue<LENGTH, DEPTH>,
}

impl<const LENGTH: usize, const DEPTH: usize, const DEBUG: bool>
    UartWriteQueue<LENGTH, DEPTH, DEBUG>
{
    pub const fn new(queue: Queue<LENGTH, DEPTH>) -> Self {
        Self { queue_tx: queue }
    }

    fn write_task(
        &'static self,
        mut tx: UartTx<'static, Async>,
        mut uart_config_command_subscriber: UartQueueSyncCommandSub,
        uart_config_response_publisher: UartQueueSyncResponsePub,
    ) -> WriteTaskFuture<LENGTH, DEPTH, DEBUG> {
        async move {
            loop {
                // the tx task primarily blocks on queue_tx.queue(), e.g. waiting for other async tasks
                // to enqueue data. Use a select to break waiting if another task signals there's a UART
                // config update. They probably want the update before the next data is enqueued
                match select(
                    self.queue_tx.dequeue(),
                    uart_config_command_subscriber.next_message(),
                )
                .await
                {
                    // we are dequeing data
                    Either::First(dq_res) => {
                        if DEBUG {
                            defmt::info!("uart q write task dequeueing");
                            Timer::after_micros(1000).await;
                        }

                        if let Ok(buf) = dq_res {
                            if DEBUG {
                                defmt::info!("uart q write task DMA write");
                            }

                            // write the message to the DMA
                            tx.write(buf.data()).await.unwrap(); // we are blocked here!

                            // drop the buffer, cleans up queue
                            drop(buf);

                            if DEBUG {
                                defmt::info!("uart q write task DMA done");
                            }

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
                                if task_command == UartTaskCommand::Pause {
                                    uart_config_response_publisher
                                        .publish(UartTaskResponse::WriteTaskPaused)
                                        .await;

                                    if let UartTaskCommand::UpdateConfig(_) =
                                        uart_config_command_subscriber.next_message_pure().await
                                    {
                                        defmt::trace!("uart write task received config, deferring to read for update");
                                    } else {
                                        defmt::error!("uart write task received invalid command expecting config update");
                                    }

                                    // read task should perform the update, expect unpause command soon
                                    if let UartTaskCommand::Unpause =
                                        uart_config_command_subscriber.next_message_pure().await
                                    {
                                        defmt::trace!("uart write task received unpause");
                                    } else {
                                        defmt::error!("uart write task received invalid command expecting unpause");
                                    }

                                    uart_config_response_publisher
                                        .publish(UartTaskResponse::WriteTaskRunning)
                                        .await;
                                } else {
                                    defmt::warn!(
                                        "uart write task received spurious command during write."
                                    );
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
    fn read<RET, FN: FnOnce(&[u8]) -> RET>(
        &self,
        fn_read: FN,
    ) -> impl core::future::Future<Output = Result<RET, ()>>;
}

pub trait Writer {
    fn write<FN: FnOnce(&mut [u8]) -> usize>(
        &self,
        fn_write: FN,
    ) -> impl core::future::Future<Output = Result<(), ()>>;
}

impl<const LEN: usize, const DEPTH: usize, const DEBUG: bool> Reader
    for UartReadQueue<LEN, DEPTH, DEBUG>
{
    async fn read<RET, FN: FnOnce(&[u8]) -> RET>(&self, fn_read: FN) -> Result<RET, ()> {
        Ok(self.dequeue(|buf| fn_read(buf)).await)
    }
}

impl<const LEN: usize, const DEPTH: usize, const DEBUG: bool> Writer
    for UartWriteQueue<LEN, DEPTH, DEBUG>
{
    async fn write<FN: FnOnce(&mut [u8]) -> usize>(&self, fn_write: FN) -> Result<(), ()> {
        self.enqueue(|buf| fn_write(buf)).or(Err(()))
    }
}
