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
        self,
        odin_radio::{OdinRadio, RadioInterfaceControl},
        radio::{Ipv4Addr, Radio},
        robot_radio::{RobotRadio, RobotRadioTask, TeamColor},
    },
    task::TaskStorage,
    transfer::{Reader, Reader2, Writer, Writer2},
};
use ateam_control_board::{
    control::Control,
    pins::{RobotPeripherals, DotstarRobotWrite},
    queue::{self, DequeueRef, EnqueueRef},
    radio::RadioInterfaceUart,
    stm32_interface::{update_usart, Kind},
    uart_queue::{UartReadQueue, UartWriteQueue},
    usart_buffer,
};
use cortex_m::{delay, asm::delay};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, SpawnToken};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;

use embassy_stm32::{interrupt::InterruptExt, rcc::AdcClockSource};
use embassy_stm32::{
    self as _,
    gpio::{Level, Output, Pin, Speed},
    interrupt,
    peripherals::{DMA1_CH0, DMA1_CH1, DMA2_CH0, DMA2_CH1, PC13, USART10, USART2},
    time::mhz,
    usart::{self, Uart, UartRx, UartTx},
};
use embassy_stm32::{
    bind_interrupts,
    pac::{self, Interrupt},
    peripherals,
};
use embassy_time::{Duration, Ticker, Timer};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::pubsub::PubSubChannel;

usart_buffer!(RadioQueue, USART10, DMA2_CH1, DMA2_CH0, 256, 20, 256, 10);

static ROBOT: RobotRadioReal = RobotRadioReal;
// pub sub channel for the gyro vals
// CAP queue size, n_subs, n_pubs
static GYRO_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 2, 2, 2> = PubSubChannel::new();

// pub sub channel for the battery raw adc vals
// CAP queue size, n_subs, n_pubs
static BATTERY_CHANNEL: PubSubChannel<ThreadModeRawMutex, f32, 2, 2, 2> = PubSubChannel::new();

type RadioInterfaceUartValue =
    RadioInterfaceUart<USART10, DMA2_CH0, DMA2_CH1, 256, 20, 256, 10, PC13>;
static ODIN_TASK: TaskStorage<OdinRadio<'static, RadioInterfaceUartValue>> = TaskStorage::new();
static RADIO_TASK: TaskStorage<
    RobotRadioTask<RobotRadioReal, OdinRadio<'static, RadioInterfaceUartValue>>,
> = TaskStorage::new();

struct RobotRadioReal;

impl RobotRadio for RobotRadioReal {
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
        0
    }
    fn get_robot_color(&self) -> TeamColor {
        TeamColor::Blue
    }

    fn get_robot_name(&self) -> Option<heapless::String<30>> {
        Some("Robot Test".into())
    }
}

#[link_section = ".sram4"]
static mut SPI6_BUF: [u8; 4] = [0x0; 4];

static EXECUTOR_UART_QUEUE: InterruptExecutor = InterruptExecutor::new();
// static EXECUTOR_OTHER: InterruptExecutor = InterruptExecutor::new();

pub static mut PIN_TOGGLE: Option<Output<'static, peripherals::PF0>> = None;

use embassy_stm32::dma::bdma::Channel;
use smart_leds::colors::{GREEN, RED, self};

#[interrupt]
unsafe fn CEC() {
    if let Some(pin) = &mut PIN_TOGGLE {
        pin.set_high();
    }
    EXECUTOR_UART_QUEUE.on_interrupt();
    if let Some(pin) = &mut PIN_TOGGLE {
        pin.set_low();
    }
}

// #[interrupt]
// unsafe fn FMC() {
//     EXECUTOR_OTHER.on_interrupt()
// }

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    stm32_config.rcc.per_ck = Some(mhz(64));
    stm32_config.rcc.adc_clock_source = AdcClockSource::PerCk;
    let p = embassy_stm32::init(stm32_config);

    let pin = unsafe { peripherals::PF0::steal() };
    unsafe {PIN_TOGGLE = Some(Output::new(pin, embassy_stm32::gpio::Level::Low, embassy_stm32::gpio::Speed::High));}

    let mut rp = RobotPeripherals::new(p);

    // Delay so dotstar can turn on
    Timer::after(Duration::from_millis(50)).await;

    rp.dotstar.write(colors::YELLOW, colors::YELLOW);

    // YELLOW YELLOW : boot up, initializing
    // RED/GREEN __ : something disconnected
    // GREEN (b)YELLOW : connecting to network
    // GREEN (B)BLUE : connected to network
    // GREEN (b)GREEN : connected to computer
    // RED RED : irrecoverable error

    interrupt::CEC.set_priority(interrupt::Priority::P6);
    let uart_spawner = EXECUTOR_UART_QUEUE.start(Interrupt::CEC);

    let ball_detected_thresh = 1.0;
    let mut control = Control::new(
        &uart_spawner,
        rp.motor_fr_usart,
        rp.motor_fl_usart,
        rp.motor_bl_usart,
        rp.motor_br_usart,
        rp.motor_d_usart,
        rp.motor_fr_boot0,
        rp.motor_fl_boot0,
        rp.motor_bl_boot0,
        rp.motor_br_boot0,
        rp.motor_d_boot0,
        rp.motor_fr_reset,
        rp.motor_fl_reset,
        rp.motor_bl_reset,
        rp.motor_br_reset,
        rp.motor_d_reset,
        ball_detected_thresh,
    );

    let interface =
        RadioInterfaceUart::new(&RadioQueue::QUEUE_RX, &RadioQueue::QUEUE_TX, rp.radio_reset);
    let interface = unsafe { core::mem::transmute(&interface) };
    let odin_radio = OdinRadio::<'static, RadioInterfaceUartValue>::new(interface);
    let odin_radio: &OdinRadio<_> = unsafe { core::mem::transmute(&odin_radio) };
    let robot_radio = RobotRadioTask::new(&ROBOT, odin_radio);
    let robot_radio = unsafe { core::mem::transmute(&robot_radio) };

    let (tx, rx) = rp.radio_usart.split();
    uart_spawner
        .spawn(RadioQueue::QUEUE_TX.spawn_task(tx))
        .unwrap();
    uart_spawner
        .spawn(RadioQueue::QUEUE_RX.spawn_task(rx))
        .unwrap();
    
    control.load_firmware().await;
    
    spawner.spawn(ODIN_TASK.spawn(odin_radio)).unwrap();
    spawner.spawn(RADIO_TASK.spawn(robot_radio)).unwrap();


    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        let latest_control = robot_radio.take_latest_control();
        let gyro_rads = unsafe {
            let mut buffer: [u8; 4] = [0x0; 4];
            rp.imu_cs_gyro.set_low();
            rp.imu_spi.blocking_transfer_in_place(&mut buffer[0..3]);
            rp.imu_cs_gyro.set_high();
            let rate_z_raw = (buffer[2] as u16 * 256 + buffer[1] as u16) as i16;
            let gyro_conversion = 2000.0 / 32767.0;
            let rate_z_raw = rate_z_raw as f32 * gyro_conversion;
            let rate_z_rads = rate_z_raw * 2.0 * core::f32::consts::PI / 360.0;
            rate_z_rads
        };

        control.tick(latest_control, gyro_rads, 24.).await;

        ticker.next().await;
    }
}
