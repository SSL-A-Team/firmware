use core::cell::RefCell;

use ateam_common_packets::bindings_radio::{BasicControl, BasicTelemetry};
use core::future::Future;
use defmt::*;
use embassy_executor::{raw::TaskStorage, SendSpawner, SpawnToken};
use embassy_stm32::{gpio::Pin, interrupt::Interrupt, usart, Peripheral};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Duration;
use motor_embassy::{
    drivers::radio::{RobotRadio, TeamColor},
    queue,
    uart_queue::{UartReadQueue, UartWriteQueue},
};

pub const MAX_TX_PACKET_SIZE: usize = 256;
pub const TX_BUF_DEPTH: usize = 4;
pub const MAX_RX_PACKET_SIZE: usize = 256;
pub const RX_BUF_DEPTH: usize = 4;

#[link_section = ".axisram.buffers"]
pub static mut BUFFERS_TX: [queue::Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] =
    [queue::Buffer::EMPTY; TX_BUF_DEPTH];
#[link_section = ".axisram.buffers"]
pub static mut BUFFERS_RX: [queue::Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] =
    [queue::Buffer::EMPTY; RX_BUF_DEPTH];

type ControlTaskFuture<
    UART: usart::BasicInstance + Send,
    RxDma: usart::RxDma<UART> + Send,
    TxDma: usart::TxDma<UART> + Send,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    ResetPin: Pin + Send + Sync,
> where
    UART::Interrupt: Send,
= impl Future + Sync;

// pub struct Radio<
//     UART: usart::BasicInstance + Send,
//     RxDma: usart::RxDma<UART> + Send,
//     TxDma: usart::TxDma<UART> + Send,
//     ResetPin: Pin + Send,
// > where
//     UART::Interrupt: Send,
// {
//     // queue_rx: UartReadQueue<'static, UART, RxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>,
//     // queue_tx: UartWriteQueue<'static, UART, TxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>,
//     task: TaskStorage<ControlTaskFuture<UART, RxDma, TxDma, ResetPin>>,

//     radio: RefCell<
//         Option<
//             RobotRadio<
//                 'static,
//                 UART,
//                 RxDma,
//                 TxDma,
//                 MAX_RX_PACKET_SIZE,
//                 MAX_TX_PACKET_SIZE,
//                 RX_BUF_DEPTH,
//                 TX_BUF_DEPTH,
//                 ResetPin,
//             >,
//         >,
//     >,
// }

// TODO probably
// unsafe impl<
//         UART: usart::BasicInstance + Send,
//         RxDma: usart::RxDma<UART> + Send,
//         TxDma: usart::TxDma<UART> + Send,
//         ResetPin: Pin + Send,
//     > Send for Radio<UART, RxDma, TxDma, ResetPin>
// where
//     UART::Interrupt: Send,
// {
// }
// unsafe impl<
//         UART: usart::BasicInstance + Send,
//         RxDma: usart::RxDma<UART> + Send,
//         TxDma: usart::TxDma<UART> + Send,
//         ResetPin: Pin + Send,
//     > Sync for Radio<UART, RxDma, TxDma, ResetPin>
// where
//     UART::Interrupt: Send,
// {
// }

/*
 * Uart tx (Radio)
 *   mut - write task
 * Uart rx (Radio)
 *   mut - read task
 * Write Queue
 *   [mut] - write task
 *   [mut] - control task (through radio)
 * Read Queue
 *   [mut] - read task
 *   [mut] - radio read task
 * Radio State
 *   mut - radio read task
 * Latest Control
 *   mut - radio read task
 *   mut - control task
 */

pub struct RadioTest<
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    UART: usart::BasicInstance + Send,
    RxDma: usart::RxDma<UART> + Send,
    TxDma: usart::TxDma<UART> + Send,
    ResetPin: Pin + Send + Sync,
> where
    UART::Interrupt: Send,
{
    queue_tx: UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
    queue_rx: UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
    task: TaskStorage<
        ControlTaskFuture<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>,
    >,
    latest_control: Mutex<CriticalSectionRawMutex, Option<BasicControl>>,
    radio:  Option<
        RobotRadio<'static, UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>,
    >,
}

// impl<
//         const LEN_TX: usize,
//         const LEN_RX: usize,
//         const DEPTH_TX: usize,
//         const DEPTH_RX: usize,
//         UART: usart::BasicInstance + Send,
//         RxDma: usart::RxDma<UART> + Send,
//         TxDma: usart::TxDma<UART> + Send,
//         ResetPin: Pin + Send + Sync,
//     > RadioTest<LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, UART, RxDma, TxDma, ResetPin>
// where
//     UART::Interrupt: Send,
// {
// }

impl<
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        UART: usart::BasicInstance + Send,
        RxDma: usart::RxDma<UART> + Send,
        TxDma: usart::TxDma<UART> + Send,
        ResetPin: Pin + Send + Sync,
    > RadioTest<LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, UART, RxDma, TxDma, ResetPin>
where
    UART::Interrupt: Send,
{
    pub const fn new(
        tx_buffer: &'static mut [queue::Buffer<LEN_TX>; DEPTH_TX],
        rx_buffer: &'static mut [queue::Buffer<LEN_RX>; DEPTH_RX],
    ) -> Self {
        RadioTest {
            queue_rx: UartReadQueue::new(rx_buffer),
            queue_tx: UartWriteQueue::new(tx_buffer),
            task: TaskStorage::new(),
            latest_control: Mutex::new(None),
            radio: None,
        }
    }

    // setup uart
    // setup tasks
    pub async fn setup(
        &'static mut self,
        spawner: &SendSpawner,
        uart: usart::Uart<'static, UART, TxDma, RxDma>,
        int: UART::Interrupt,
        reset_pin: impl Peripheral<P = ResetPin> + 'static,
        id: u8,
        team: TeamColor,
    ) -> SpawnToken<impl Sized> {
        let (tx, rx) = uart.split();

        spawner.spawn(self.queue_rx.spawn_task(rx, int)).unwrap();
        spawner.spawn(self.queue_tx.spawn_task(tx)).unwrap();

        let mut radio = RobotRadio::new(&self.queue_rx, &self.queue_tx, reset_pin)
            .await
            .unwrap();

        info!("radio created");
        radio.connect_to_network().await.unwrap();
        info!("radio connected");

        radio.open_multicast().await.unwrap();
        info!("multicast open");

        loop {
            info!("sending hello");
            radio.send_hello(id, team).await.unwrap();
            let hello = radio.wait_hello(Duration::from_millis(1000)).await;

            match hello {
                Ok(hello) => {
                    info!(
                        "recieved hello resp to: {}.{}.{}.{}:{}",
                        hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
                    );
                    radio.close_peer().await.unwrap();
                    info!("multicast peer closed");
                    radio.open_unicast(hello.ipv4, hello.port).await.unwrap();
                    info!("unicast open");
                    break;
                }
                Err(_) => {}
            }
        }

        {
            self.radio = Some(radio);
            // let mut radio_self = self.radio.lock().await;
            // *radio_self = Some(radio);
        }


        // self.task.spawn(|| self.control_task())
        self.task.spawn(|| Self::control_task(self.radio.as_ref().unwrap(), &self.latest_control))
        // spawner.spawn(token).unwrap();

    }

    // fn spawn_task(&'static self, ) -> SpawnToken<impl Sized> {

    // }

    fn control_task(
        // &'static self
        radio: &'static RobotRadio<
            'static,
            UART,
            RxDma,
            TxDma,
            LEN_TX,
            LEN_RX,
            DEPTH_TX,
            DEPTH_RX,
            ResetPin,
        >,
        latest_control: &'static Mutex<CriticalSectionRawMutex, Option<BasicControl>>,
    ) -> ControlTaskFuture<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin> {
        async move {
            loop {
                let control = radio.read_control().await;
                if let Ok(control) = control {
                    let mut latest_control = latest_control.lock().await;
                    *latest_control = Some(control);
                }
            }
        }
    }

    // connect to network
    // open multicast
    // send hello request
    // wait hello response
    // close multicast
    // open unicast

    // pub async fn connect(&mut self) {
    //     if let Some(radio) = &mut self.radio {
    //         radio.connect_to_network().await.unwrap();
    //     }
    // }

    // write telemetry to queue
    pub async fn send_telemetry(&self, telemetry: BasicTelemetry) {
        self.radio.as_ref().unwrap().send_telemetry(telemetry).await.unwrap();
    }

    // fetch latest stored control value
    pub fn get_latest_control(&self) -> Option<BasicControl> {
        let mut latest_control = self.latest_control.try_lock();
        if let Ok(latest_control) = &mut latest_control {
            latest_control.take()
        } else {
            None
        }
    }
}

// impl<
//         UART: usart::BasicInstance + Send,
//         RxDma: usart::RxDma<UART> + Send,
//         TxDma: usart::TxDma<UART> + Send,
//         ResetPin: Pin + Send,
//     > Radio<UART, RxDma, TxDma, ResetPin>
// where
//     UART::Interrupt: Send,
// {
//     pub const fn new() -> Self {
//         Radio {
//             // queue_rx: UartReadQueue::new(unsafe { &mut BUFFERS_RX }),
//             // queue_tx: UartWriteQueue::new(unsafe { &mut BUFFERS_TX }),
//             radio: RefCell::new(None),
//             task: TaskStorage::new(),
//         }
//     }

//     fn control_task(radio: &'static Self) -> ControlTaskFuture<UART, RxDma, TxDma, ResetPin> {
//         async move { loop {} }
//     }

//     // fn spawn_task(
//     //     &'static self,
//     //     // rx: UartRx<'a, UART, DMA>,
//     //     // int: UART::Interrupt,
//     // ) -> SpawnToken<impl Sized> {
//     //     self.task.spawn(|| Self::control_task(self))
//     // }

//     // pub fn spawn_task(
//     //     &'static self,
//     //     rx: UartRx<'a, UART, DMA>,
//     //     int: UART::Interrupt,
//     // ) -> SpawnToken<impl Sized> {
//     //     self.task.spawn(|| Self::read_task(&self.queue_rx, rx, int))
//     // }

//     pub async fn start(
//         &'static self,
//         spawner: &SendSpawner,
//         uart: usart::Uart<'static, UART, TxDma, RxDma>,
//         reset_pin: impl Peripheral<P = ResetPin> + 'static,
//     ) {
//         let (tx, rx) = uart.split();
//         let int = unsafe { <UART::Interrupt as Interrupt>::steal() };
//         // spawner.spawn(self.queue_rx.spawn_task(rx, int)).unwrap();
//         // spawner.spawn(self.queue_tx.spawn_task(tx)).unwrap();

//         // let mut radio = RobotRadio::new(&self.queue_rx, &self.queue_tx, reset_pin)
//         //     .await
//         //     .unwrap();

//         // info!("radio created");
//         // radio.connect_to_network().await.unwrap();
//         // info!("radio connected");

//         // loop {
//         //     info!("sending hello");
//         //     radio.send_hello(0, TeamColor::Blue).await.unwrap();
//         //     let hello = radio.wait_hello(Duration::from_millis(1000)).await;

//         //     match hello {
//         //         Ok(hello) => {
//         //             info!(
//         //                 "recieved hello resp to: {}.{}.{}.{}:{}",
//         //                 hello.ipv4[0], hello.ipv4[1], hello.ipv4[2], hello.ipv4[3], hello.port
//         //             );
//         //             radio.close_peer().await.unwrap();
//         //             info!("multicast peer closed");
//         //             radio.open_unicast(hello.ipv4, hello.port).await.unwrap();
//         //             info!("unicast open");
//         //             break;
//         //         }
//         //         Err(_) => {}
//         //     }
//         // }

//         // self.radio.replace(Some(radio));

//         // self.task.spawn(|| Self::control_task(&self));

//         // spawner.spawn(self.spawn_task()).unwrap()

//         // loop {
//         //     let control = radio.read_control().await;
//         //     if let Ok(control) = control {
//         //         info!("{:?}", defmt::Debug2Format(&control));
//         //     }
//         // }
//     }
// }

// pub async fn setup_radio<
//     'a,
//     UART: usart::BasicInstance + core::marker::Send,
//     ResetPin: Pin,
//     const LENGTH_RX: usize,
//     const DEPTH_RX: usize,
//     DMA_RX: usart::RxDma<UART> + core::marker::Send,
//     const LENGTH_TX: usize,
//     const DEPTH_TX: usize,
//     DMA_TX: usart::TxDma<UART> + core::marker::Send,
// >(
//     spawner: &SendSpawner,
//     uart: usart::Uart<'a, UART, DMA_TX, DMA_RX>,
//     // queue_rx: &'static UartReadQueue<'a, UART, DMA_RX, LENGTH_RX, DEPTH_RX>,
//     queue_tx: &'static UartWriteQueue<'a, UART, DMA_TX, LENGTH_TX, DEPTH_TX>,
//     uart_int: RadioUART::Interrupt,
//     reset_pin: impl Peripheral<P = ResetPin>,
// ) where UART::Interrupt: Send {
//     let (tx, rx) = uart.split();

//     // TODO: hardcoded USART2
//     // let int = interrupt::take!(USART2);
//     spawner.spawn(QUEUE_RX.spawn_task(rx, uart_int)).unwrap();
//     spawner.spawn(queue_tx.spawn_task(tx)).unwrap();

//     let mut radio = RobotRadio::new(&QUEUE_RX, &queue_tx, reset_pin)
//         .await
//         .unwrap();

//     info!("radio created");
//     radio.connect_to_network().await.unwrap();
//     info!("radio connected");

//     loop {}
// }
