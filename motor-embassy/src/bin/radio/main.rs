#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]

mod at_protocol;
mod edm_protocol;
mod radio;

use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;

use embassy_stm32::gpio::{Level, OutputOpenDrain, Pin, Pull, Speed};
use embassy_stm32::time::mhz;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::Peripheral;
use embassy_stm32::{
    self as _,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    pac,
    peripherals::{DMA1_CH0, DMA1_CH1, USART2},
};
use embassy_time::{Duration, Timer};
use heapless::String;
use motor_embassy::queue;
use motor_embassy::uart_queue::{UartReadQueue, UartWriteQueue};
use panic_probe as _;
use radio::Radio;
use static_cell::StaticCell;

use crate::radio::WifiAuth;

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [queue::Buffer<256>; 4] = [queue::Buffer::EMPTY; 4];
static QUEUE_TX: UartWriteQueue<USART2, DMA1_CH0, 256, 4> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [queue::Buffer<256>; 4] = [queue::Buffer::EMPTY; 4];
static QUEUE_RX: UartReadQueue<USART2, DMA1_CH1, 256, 4> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

fn get_uuid() -> u16 {
    unsafe { *(0x1FF1_E800 as *const u16) }
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
    let config = usart::Config::default();
    let usart = Uart::new(p.USART2, p.PD6, p.PD5, p.DMA1_CH0, p.DMA1_CH1, config);

    let (tx, rx) = usart.split();

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();
    info!("start");

    let int = interrupt::take!(USART2);
    spawner.spawn(QUEUE_RX.spawn_task(rx, int)).unwrap();
    spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap();

    let radio = OdinW2Radio::new(&QUEUE_RX, &QUEUE_TX, p.PA3).await.unwrap();
    info!("radio created");
    radio.connect_to_network().await.unwrap();
    info!("radio connected");

    loop {
        radio.radio.read().await;
    }

    // radio
    //     .edm_command_send("AT+UDCP=\"udp://224.4.20.69:42069/?flags=1&local_port=42069\"")
    //     .unwrap();

    // radio
    //     .edm_command_send("AT+UDSC=1,udp://0.0.0.0:42069/")
    //     .unwrap();

    // this one
    // radio.edm_command_send("AT+UDSC=1,2,42069,1,0").unwrap();

    // radio.edm_basic_command("AT+UMRS?").await.unwrap();

    // radio.edm_basic_command("AT+GMI").await.unwrap();
    // radio.edm_basic_command("AT+GMM").await.unwrap();
    // radio.edm_basic_command("ATI9").await.unwrap();
    // radio.edm_basic_command("AT+GMR").await.unwrap();
    // radio.edm_basic_command("AT+UMLA=1").await.unwrap();
    // radio.edm_basic_command("AT+UMLA=2").await.unwrap();
    // radio.edm_basic_command("AT+UMLA=3").await.unwrap();

    loop {}
}

struct OdinW2Radio<
    'a,
    UART: usart::BasicInstance,
    DmaRx: usart::RxDma<UART>,
    DmaTx: usart::TxDma<UART>,
    const LEN_RX: usize,
    const LEN_TX: usize,
    const DEPTH_TX: usize,
    const DEPTH_RX: usize,
    PIN: Pin,
> {
    radio: Radio<
        'a,
        UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
    >,
    reset_pin: OutputOpenDrain<'a, PIN>,
}

impl<
        'a,
        UART: usart::BasicInstance,
        DmaRx: usart::RxDma<UART>,
        DmaTx: usart::TxDma<UART>,
        const LEN_RX: usize,
        const LEN_TX: usize,
        const DEPTH_TX: usize,
        const DEPTH_RX: usize,
        PIN: Pin,
    > OdinW2Radio<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>
{
    pub async fn new(
        read_queue: &'a UartReadQueue<'a, UART, DmaRx, LEN_RX, DEPTH_TX>,
        write_queue: &'a UartWriteQueue<'a, UART, DmaTx, LEN_TX, DEPTH_RX>,
        reset_pin: impl Peripheral<P = PIN> + 'a,
    ) -> Result<OdinW2Radio<'a, UART, DmaRx, DmaTx, LEN_RX, LEN_TX, DEPTH_TX, DEPTH_RX, PIN>, ()>
    {
        let mut reset_pin = OutputOpenDrain::new(reset_pin, Level::Low, Speed::Medium, Pull::None);
        Timer::after(Duration::from_micros(100)).await;
        reset_pin.set_high();

        let mut radio = radio::Radio::new(read_queue, write_queue);
        radio.wait_startup().await?;

        let baudrate = 5250000;
        radio.set_echo(false).await?;
        radio.config_uart(baudrate, false, 8, true).await?;

        let div = (UART::frequency().0 + (baudrate / 2)) / baudrate * UART::MULTIPLIER;
        unsafe {
            let r = UART::regs();
            r.cr1().modify(|w| {
                w.set_ue(false);
            });
            r.brr().modify(|w| {
                w.set_brr(div);
            });
            r.cr1().modify(|w| {
                w.set_ue(true);
                w.set_m0(pac::lpuart::vals::M0::BIT9);
                w.set_pce(true);
                w.set_ps(pac::lpuart::vals::Ps::EVEN);
            });
        };

        // Datasheet says wait at least 40ms after UART config change
        Timer::after(Duration::from_millis(50)).await;

        // Datasheet says wait at least 50ms after entering data mode
        radio.enter_edm().await?;
        radio.wait_edm_startup().await?;
        Timer::after(Duration::from_millis(50)).await;

        Ok(Self { radio, reset_pin })
    }

    pub async fn connect_to_network(&self) -> Result<(), ()> {
        let mut s = String::<17>::new();
        core::write!(&mut s, "A-Team Robot {:04X}", get_uuid()).unwrap();
        self.radio.set_host_name(s.as_str()).await?;
        self.radio
            .config_wifi(
                0,
                "PROMISED_LAN_DC_DEVEL",
                WifiAuth::WPA {
                    passphrase: "plddevel",
                },
            )
            .await?;
        self.radio.connect_wifi(0).await?;
        Ok(())
    }
}
