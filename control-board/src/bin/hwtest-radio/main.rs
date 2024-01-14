#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(const_mut_refs)]
#![feature(ptr_metadata)]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use static_cell::StaticCell;

use embassy_stm32::{
    self as _,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    peripherals::{DMA1_CH0, DMA1_CH1, USART2},
    usart::{self, Uart},
    time::mhz, gpio::{Input, Pull}
};
use embassy_time::{Duration, Timer};

use ateam_control_board::drivers::radio::{RobotRadio, TeamColor, WifiNetwork};
use ateam_control_board::queue;
use ateam_control_board::uart_queue::{UartReadQueue, UartWriteQueue};

use ateam_common_packets::{bindings_radio::BasicTelemetry, radio::DataPacket};


static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[link_section = ".axisram.buffers"]
static mut BUFFERS_TX: [queue::Buffer<256>; 10] = [queue::Buffer::EMPTY; 10];
static QUEUE_TX: UartWriteQueue<USART2, DMA1_CH0, 256, 10> =
    UartWriteQueue::new(unsafe { &mut BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut BUFFERS_RX: [queue::Buffer<256>; 20] = [queue::Buffer::EMPTY; 20];
static QUEUE_RX: UartReadQueue<USART2, DMA1_CH1, 256, 20> =
    UartReadQueue::new(unsafe { &mut BUFFERS_RX });

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

    let config = usart::Config::default();
    let int = interrupt::take!(USART2);
    let usart = Uart::new(p.USART2, p.PD6, p.PD5, int, p.DMA1_CH0, p.DMA1_CH1, config);

    let (tx, rx) = usart.split();

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();
    info!("start");

    spawner.spawn(QUEUE_RX.spawn_task(rx)).unwrap();
    spawner.spawn(QUEUE_TX.spawn_task(tx)).unwrap();

    // let reset = p.PC0;
    let reset = p.PA3;
    let mut radio = RobotRadio::new(&QUEUE_RX, &QUEUE_TX, reset).await.unwrap();


    /////////////////////
    // Dip Switch Inputs
    /////////////////////
    let dip5 = Input::new(p.PG3, Pull::Down);
    let dip6 = Input::new(p.PG4, Pull::Down);

    let wifi_network = if dip5.is_high() & dip6.is_high() {
        WifiNetwork::Team
    } else if dip5.is_low() & dip6.is_high() {
        WifiNetwork::CompMain
    } else if dip5.is_high() & dip6.is_low() {
        WifiNetwork::CompPractice
    } else {
        WifiNetwork::Team
    };
    
    info!("radio created");
    radio.connect_to_network(wifi_network).await.unwrap();
    info!("radio connected");

    radio.open_multicast().await.unwrap();
    info!("multicast open");

    loop {
        info!("sending hello");
        radio.send_hello(0, TeamColor::Yellow).await.unwrap();
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

    let mut seq: u16 = 0;
    loop {
        let _ = radio
            .send_telemetry(BasicTelemetry {
                sequence_number: seq,
                robot_revision_major: 0,
                robot_revision_minor: 0,
                battery_level: 0.,
                battery_temperature: 0.,
                _bitfield_align_1: [],
                _bitfield_1: BasicTelemetry::new_bitfield_1(
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ),
                motor_0_temperature: 0.,
                motor_1_temperature: 0.,
                motor_2_temperature: 0.,
                motor_3_temperature: 0.,
                motor_4_temperature: 0.,
                kicker_charge_level: 0.,
            })
            .await;
        info!("send");

        let data_packet = radio.read_packet().await;
        if let Ok(data_packet) = data_packet {
            if let DataPacket::BasicControl(control) = data_packet {
                info!("received control packet.");
                info!("{:?}", defmt::Debug2Format(&control));

                info!("{}", seq);
                seq = (seq + 1) % 10000;
            }
        }
    }
}
