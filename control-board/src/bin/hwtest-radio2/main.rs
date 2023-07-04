#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]
#![feature(async_fn_in_trait)]

use core::cell::UnsafeCell;

use ateam_common::{
    radio::{
        odin_radio::{OdinRadio, RadioInterfaceControl},
        radio::{Ipv4Addr, Radio},
        robot_radio::{RobotRadio, RobotRadioTask, TeamColor}, self,
    },
    task::TaskStorage,
    transfer::{Reader, Writer, Reader2, Writer2},
};
use ateam_control_board::{
    queue::{self, EnqueueRef, DequeueRef},
    uart_queue::{UartReadQueue, UartWriteQueue}, stm32_interface::{configure_usart, Kind},
};
// use cortex_m::interrupt;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{SpawnToken, InterruptExecutor};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;

use embassy_stm32::{pac::{self, Interrupt}, bind_interrupts, peripherals};
use embassy_stm32::{
    self as _,
    // executor::InterruptExecutor,
    gpio::{Level, Output, Pin, Speed},
    interrupt,
    peripherals::{DMA1_CH0, DMA1_CH1, DMA2_CH0, DMA2_CH1, PC13, USART10, USART2},
    time::mhz,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_stm32::interrupt::InterruptExt;
use embassy_time::{Duration, Ticker, Timer};
// use futures_util::stream::stream::StreamExt;
use futures_util::StreamExt;
use static_cell::StaticCell;

struct RadioInterfaceUart<
    UART: usart::BasicInstance,
    RxDma: usart::RxDma<UART>,
    TxDma: usart::TxDma<UART>,
    // const TX_BUF: usize,
    // const RX_BUF: usize,
    const LEN_TX: usize,
    const LEN_RX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    ResetPin: Pin,
> {
    // tx: Mutex<CriticalSectionRawMutex, UartTx<'static, UART, TxDma>>,
    // rx: Mutex<CriticalSectionRawMutex, UartRx<'static, UART, RxDma>>,
    // buf_tx: UnsafeCell<&'static mut [u8; TX_BUF]>,
    // buf_rx: UnsafeCell<&'static mut [u8; RX_BUF]>,
    read_queue: &'static UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
    write_queue: &'static UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
    reset_pin: Mutex<CriticalSectionRawMutex, Output<'static, ResetPin>>,
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        // const TX_BUF: usize,
        // const RX_BUF: usize,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    fn new(
        // uart: usart::Uart<'static, UART, TxDma, RxDma>,
        // buf_tx: &'static mut [u8; TX_BUF],
        // buf_rx: &'static mut [u8; RX_BUF],
        read_queue: &'static UartReadQueue<'static, UART, RxDma, LEN_RX, DEPTH_RX>,
        write_queue: &'static UartWriteQueue<'static, UART, TxDma, LEN_TX, DEPTH_TX>,
        reset_pin: ResetPin,
    ) -> Self {
        let mut reset_pin = Output::new(reset_pin, Level::Low, Speed::Medium);
        // let (tx, rx) = uart.split();
        // Self {
        //     tx: Mutex::new(tx),
        //     rx: Mutex::new(rx),
        //     buf_tx: UnsafeCell::new(buf_tx),
        //     buf_rx: UnsafeCell::new(buf_rx),
        //     reset_pin: Mutex::new(reset_pin),
        // }
        Self {
            read_queue,
            write_queue,
            reset_pin: Mutex::new(reset_pin),
        }
    }
}

// impl<
//         UART: usart::BasicInstance,
//         RxDma: usart::RxDma<UART>,
//         TxDma: usart::TxDma<UART>,
//         // const TX_BUF: usize,
//         // const RX_BUF: usize,
//         const LEN_TX: usize,
//         const LEN_RX: usize,
//         const DEPTH_TX: usize,
//         const DEPTH_RX: usize,
//         ResetPin: Pin,
//     > Reader
//     for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
// {
//     async fn read<RET, FN: FnOnce(&[u8]) -> Result<RET, ()>>(
//         &self,
//         fn_read: FN,
//     ) -> Result<RET, ()> {
//         // let mut rx = self.rx.lock().await;
//         // let buf = unsafe { &mut **self.buf_rx.get() };
//         // let size = loop {
//         //     let size = rx.read_until_idle(buf).await.or(Err(()))?;
//         //     if size != 0 {
//         //         break size;
//         //     }
//         // };
//         // info!("read {}", size);
//         // Ok(fn_read(&buf[..size]).unwrap())

//         info!("r");

//         Ok(self.read_queue.dequeue(|buf| fn_read(buf).unwrap()).await)
//     }
// }

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        // const TX_BUF: usize,
        // const RX_BUF: usize,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > Writer2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    type DataRefWrite = EnqueueRef<'static, LEN_TX, DEPTH_TX> ;

    async fn write<'a>(&'a self) -> Result<Self::DataRefWrite, ()> {
        self.write_queue.enqueue2()
    }

    // async fn write<FN: FnOnce(&mut [u8]) -> Result<usize, ()>>(
    //     &self,
    //     fn_write: FN,
    // ) -> Result<(), ()> {
    //     // let mut tx = self.tx.lock().await;
    //     // let buf = unsafe { &mut **self.buf_tx.get() };
    //     // let size = fn_write(buf)?;
    //     // info!("write {}", size);
    //     // tx.write(&buf[..size]).await;
    //     // Ok(())

    //     self.write_queue.enqueue(|buf| {
    //         let size = fn_write(buf).unwrap();
    //         // info!("write {}", size);
    //         size
    //     });
    //     Ok(())
    // }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        // const TX_BUF: usize,
        // const RX_BUF: usize,
        const LEN_TX: usize,
        const LEN_RX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        ResetPin: Pin,
    > Reader2
    for RadioInterfaceUart<UART, RxDma, TxDma, LEN_TX, LEN_RX, DEPTH_TX, DEPTH_RX, ResetPin>
{
    type DataRefRead = DequeueRef<'static, LEN_RX, DEPTH_RX> ;

    async fn read<'a>(&'a self) -> Result<Self::DataRefRead, ()> {
        self.read_queue.dequeue2().await
    }
//     async fn read<RET, FN: FnOnce(&[u8]) -> Result<RET, ()>>(
//         &self,
//         fn_read: FN,
//     ) -> Result<RET, ()> {
//         // let mut rx = self.rx.lock().await;
//         // let buf = unsafe { &mut **self.buf_rx.get() };
//         // let size = loop {
//         //     let size = rx.read_until_idle(buf).await.or(Err(()))?;
//         //     if size != 0 {
//         //         break size;
//         //     }
//         // };
//         // info!("read {}", size);
//         // Ok(fn_read(&buf[..size]).unwrap())

//         info!("r");

//         Ok(self.read_queue.dequeue(|buf| fn_read(buf).unwrap()).await)
//     }
}

impl<
        UART: usart::BasicInstance,
        RxDma: usart::RxDma<UART>,
        TxDma: usart::TxDma<UART>,
        // const TX_BUF: usize,
        // const RX_BUF: usize,
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
        // disable the uart. Can't modify parity and baudrate while module is enabled

        let mut config = usart::Config::default();
        config.baudrate = baudrate;
        config.parity = if parity { usart::Parity::ParityEven } else { usart::Parity::ParityNone };
        configure_usart(r, &config, UART::frequency(), Kind::Uart, true, true)

        // let div = (UART::frequency().0 + (baudrate / 2)) / baudrate * UART::MULTIPLIER;
        // unsafe {
        //     let r = UART::regs();
        //     r.cr1().modify(|w| {
        //         w.set_ue(false);
        //     });
        //     r.brr().modify(|w| {
        //         w.set_brr(div);
        //     });
        //     r.cr1().modify(|w| {
        //         w.set_ue(true);
        //         w.set_m0(if parity {
        //             pac::usart::vals::M0::BIT9
        //         } else {
        //             pac::usart::vals::M0::BIT8
        //         });
        //         w.set_pce(parity);
        //         w.set_ps(pac::usart::vals::Ps::EVEN);
        //     });
        // };
    }
}

// #[link_section = ".axisram.buffers"]
// static mut BUFFERS_TX: [u8; 256] = [0; 256];
// #[link_section = ".axisram.buffers"]
// static mut BUFFERS_RX: [u8; 256] = [0; 256];

// static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<Interrupt::CEC>> = StaticCell::new();
// static EXECUTOR_OTHER: StaticCell<InterruptExecutor<Interrupt::FMC>> = StaticCell::new();

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [queue::Buffer<256>; 10] = [queue::Buffer::EMPTY; 10];
static QUEUE_TX: UartWriteQueue<USART10, DMA2_CH0, 256, 10> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [queue::Buffer<256>; 20] = [queue::Buffer::EMPTY; 20];
static QUEUE_RX: UartReadQueue<USART10, DMA2_CH1, 256, 20> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

type RadioInterfaceUartValue =
    RadioInterfaceUart<USART10, DMA2_CH0, DMA2_CH1, 256, 256, 10, 20, PC13>;
static ODIN_TASK: TaskStorage<OdinRadio<'static, RadioInterfaceUartValue>> = TaskStorage::new();

struct RobotMock;
impl RobotRadio for RobotMock {
    const WIFI_SSID: &'static str = "A-Team Field";
    const WIFI_PASS: Option<&'static str> = Some("plancomestogether");
    // const WIFI_SSID: &'static str = "Evan iPhone";
    // const WIFI_PASS: Option<&'static str> = Some("2ehbp9vcvdee");
    const MULTICAST_ADDR: Ipv4Addr = Ipv4Addr {
        octets: [224, 4, 20, 71],
    };
    const MULTICAST_PORT: u16 = 42069;
    const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);
    const HELLO_RATE: Duration = Duration::from_millis(1000);

    fn get_robot_id(&self) -> u8 {
        // log::info!("get robot id");
        0
    }
    fn get_robot_color(&self) -> TeamColor {
        TeamColor::Blue
    }
}

bind_interrupts!(struct Irqs {
    USART10 => usart::InterruptHandler<peripherals::USART10>;
});

static ROBOT: RobotMock = RobotMock;
static RADIO_TASK: TaskStorage<
    RobotRadioTask<RobotMock, OdinRadio<'static, RadioInterfaceUartValue>>,
> = TaskStorage::new();

static EXECUTOR_UART_QUEUE: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_OTHER: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    EXECUTOR_UART_QUEUE.on_interrupt()
}

#[interrupt]
unsafe fn FMC() {
    EXECUTOR_OTHER.on_interrupt()
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    // Delay so dotstar can turn on
    Timer::after(Duration::from_millis(50)).await;

    let mut radio_config = usart::Config::default();
    radio_config.stop_bits = usart::StopBits::STOP1;
    radio_config.detect_previous_overrun = true;
    let radio_usart = Uart::new(
        p.USART10,
        p.PE2,
        p.PE3,
        Irqs,
        p.DMA2_CH0,
        p.DMA2_CH1,
        radio_config,
    );
    let radio_reset = p.PC13;

    let (tx, rx) = radio_usart.split();

    {
        interrupt::CEC.set_priority(interrupt::Priority::P6);
        let spawner = EXECUTOR_UART_QUEUE.start(Interrupt::CEC);

        spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap(); // TODO: make low priority?
        spawner.spawn(QUEUE_RX.spawn_task(rx)).unwrap();
    }

    let interface = RadioInterfaceUart::new(
        &QUEUE_RX,
        &QUEUE_TX,
        // radio_usart,
        // unsafe { &mut BUFFERS_TX },
        // unsafe { &mut BUFFERS_RX },
        radio_reset,
    );
    let interface = unsafe { core::mem::transmute(&interface) };
    let odin_radio = OdinRadio::<'static, RadioInterfaceUartValue>::new(interface);
    let odin_radio: &OdinRadio<_> = unsafe { core::mem::transmute(&odin_radio) };

    {
        interrupt::FMC.set_priority(interrupt::Priority::P6);
        let spawner = EXECUTOR_OTHER.start(Interrupt::FMC);

        spawner.spawn(ODIN_TASK.spawn(odin_radio)).unwrap();
        let robot_radio = RobotRadioTask::new(&ROBOT, odin_radio);
        // let token = RADIO_TASK.spawn(robot_radio);
        spawner
            .spawn(RADIO_TASK.spawn(robot_radio))
            // .spawn(token)
            .unwrap();
    }

    // odin_radio
    //     .connect_to_network(
    //         "A-Team Field",
    //         Some("plancomestogether"),
    //         Some("Robot Test"),
    //     )
    //     .await;

    // let multicast_addr = Ipv4Addr {
    //     octets: [224, 4, 20, 71],
    // };
    // let multicast_socket = odin_radio.open_udp(multicast_addr, 42069, 42069).await.unwrap();
    // info!("multicast open");
    // let unicast_addr = Ipv4Addr {
    //     octets: [172, 16, 1, 171],
    // };
    // let unicast_socket = odin_radio.open_udp(unicast_addr, 42069, 42069).await.unwrap();
    // Timer::after(Duration::from_millis(500)).await;
    // odin_radio.list_peers().await;

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await;
    }
}
