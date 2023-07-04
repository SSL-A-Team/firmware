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
    uart_queue::{UartReadQueue, UartWriteQueue}, stm32_interface::{update_usart, Kind}, radio::RadioInterfaceUart,
};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{SpawnToken, InterruptExecutor};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;

use embassy_stm32::{pac::{self, Interrupt}, bind_interrupts, peripherals};
use embassy_stm32::{
    self as _,
    gpio::{Level, Output, Pin, Speed},
    interrupt,
    peripherals::{DMA1_CH0, DMA1_CH1, DMA2_CH0, DMA2_CH1, PC13, USART10, USART2},
    time::mhz,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_stm32::interrupt::InterruptExt;
use embassy_time::{Duration, Ticker, Timer};

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

struct RobotRadioReal;

impl RobotRadio for RobotRadioReal {
    const WIFI_SSID: &'static str = "A-Team Field";
    const WIFI_PASS: Option<&'static str> = Some("plancomestogether");
    const MULTICAST_ADDR: Ipv4Addr = Ipv4Addr {
        octets: [224, 4, 20, 71],
    };
    const MULTICAST_PORT: u16 = 42069;
    const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);
    const HELLO_RATE: Duration = Duration::from_millis(1000);

    fn get_robot_id(&self) -> u8 {
        0
    }
    fn get_robot_color(&self) -> TeamColor {
        TeamColor::Blue
    }

    fn get_robot_name(&self) -> Option<heapless::String<30>> {
        Some("Robot Test".into())
    }
}

bind_interrupts!(struct Irqs {
    USART10 => usart::InterruptHandler<peripherals::USART10>;
});

static ROBOT: RobotRadioReal = RobotRadioReal;
static RADIO_TASK: TaskStorage<
    RobotRadioTask<RobotRadioReal, OdinRadio<'static, RadioInterfaceUartValue>>,
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
        spawner
            .spawn(RADIO_TASK.spawn(robot_radio))
            .unwrap();
    }

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        ticker.next().await;
    }
}
