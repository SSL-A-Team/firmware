#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(async_closure)]

use defmt_rtt as _;
use defmt::*;
use embassy_stm32::{
    self,
    executor::InterruptExecutor,
    interrupt::{self, InterruptExt},
    gpio::{Level, OutputOpenDrain, Pull, Speed, Output},
    peripherals::{DMA1_CH0, DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7},
    peripherals::{USART3, UART4, UART5, UART7},
    usart::{Uart, Parity}, time::mhz,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

use motor_embassy::{
    stm32_interface::{Stm32Interface, self},
    queue::Buffer,
    uart_queue::{UartReadQueue, UartWriteQueue},
    include_external_cpp_bin, stspin_motor::WheelMotor,
};

include_external_cpp_bin!{STEVAL3204_DRIB_POTCTRL_FW_IMG, "dev3204-drib-potctrl.bin"}
include_external_cpp_bin!{STEVAL3204_DRIB_FW_IMG, "dev3204-drib.bin"}


// motor pinout
// FrontLeft Wheel  - UART7  - tx pf7,  rx pf6,  boot pg2,  rst pg3
// BackLeft Wheel   - UART4  - tx pd1,  rx pd0,  boot pg0,  rst pg1
// FrontRight Wheel - UART5  - tx pb6,  pb12,    boot pb1,  rst pb2
// BackRight Wheel  - USART3 - tx pb10, rx pb11, boot pa8,  rst pa9

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_TX: [Buffer<MAX_TX_PACKET_SIZE>; TX_BUF_DEPTH] = [Buffer::EMPTY; TX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_TX: UartWriteQueue<UART7, DMA1_CH0, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> =
    UartWriteQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_TX });

#[link_section = ".axisram.buffers"]
static mut FRONT_LEFT_BUFFERS_RX: [Buffer<MAX_RX_PACKET_SIZE>; RX_BUF_DEPTH] = [Buffer::EMPTY; RX_BUF_DEPTH];
static FRONT_LEFT_QUEUE_RX: UartReadQueue<UART7, DMA1_CH1, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> =
    UartReadQueue::new(unsafe { &mut FRONT_LEFT_BUFFERS_RX });

static EXECUTOR_UART_QUEUE: StaticCell<InterruptExecutor<interrupt::CEC>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Startup");

    let mut stm32_config: embassy_stm32::Config = Default::default();
    stm32_config.rcc.hse = Some(mhz(8));
    stm32_config.rcc.sys_ck = Some(mhz(400));
    stm32_config.rcc.hclk = Some(mhz(200));
    stm32_config.rcc.pclk1 = Some(mhz(100));
    let p = embassy_stm32::init(stm32_config);

    let irq = interrupt::take!(CEC);
    irq.set_priority(interrupt::Priority::P6);
    let executor = EXECUTOR_UART_QUEUE.init(InterruptExecutor::new(irq));
    let spawner = executor.start();

    let mut grn_led = Output::new(p.PB0, Level::High, Speed::Medium);
    let mut ylw_led = Output::new(p.PE1, Level::High, Speed::Medium);
    let mut red_led = Output::new(p.PB14, Level::High, Speed::Medium);

    Timer::after(Duration::from_millis(1000)).await;
    ylw_led.set_low();
    red_led.set_low();


    let front_left_uart_config = stm32_interface::get_bootloader_uart_config();
    let usart = Uart::new(p.UART7, p.PF6, p.PF7, p.DMA1_CH0, p.DMA1_CH1, front_left_uart_config);
    let (tx, rx) = usart.split();

    let int = interrupt::take!(UART7);
    spawner.spawn(FRONT_LEFT_QUEUE_RX.spawn_task(rx, int)).unwrap();
    spawner.spawn(FRONT_LEFT_QUEUE_TX.spawn_task(tx)).unwrap();

    let front_left_boot0_pin = Output::new(p.PG2, Level::Low, Speed::Medium);
    let front_left_reset_pin = OutputOpenDrain::new(p.PG3, Level::Low, Speed::Medium, Pull::None);
    let front_left_stm32_interface = Stm32Interface::new(&FRONT_LEFT_QUEUE_RX, &FRONT_LEFT_QUEUE_TX, Some(front_left_boot0_pin), Some(front_left_reset_pin));
    // front_left_stm32_interface.load_firmware_image(STEVAL3204_DRIB_FW_IMG).await;
    let mut front_left_motor = WheelMotor::new(front_left_stm32_interface, STEVAL3204_DRIB_FW_IMG);
    front_left_motor.load_default_firmware_image().await;
    front_left_motor.leave_reset().await;

    let mut ct = 0;

    let mut angle: f32 = 0.0;

    loop {
        front_left_motor.process_packets();
        // defmt::info!("processed packets");

        if ct > 100 {
            ct = 0;
            front_left_motor.set_setpoint(libm::sinf(angle));
            front_left_motor.send_motion_command();
            // defmt::info!("sent motion update");

            angle += core::f32::consts::FRAC_2_PI / 20.0;
        }

        ct += 1;
        Timer::after(Duration::from_millis(1)).await;
    }

    // load firmware image
    // let res = stm32_interface.load_firmware_image(STEVAL3204_DRIB_POTCTRL_FW_IMG).await;
    // let res = stm32_interface.load_firmware_image(STEVAL3204_DRIB_FW_IMG).await;

    // switch to frequency that the firmware images use
    // unsafe { stm32_interface.update_uart_config(2_000_000, Parity::ParityEven) };
    // Timer::after(Duration::from_millis(10)).await;

    ylw_led.set_high();

    // bring part out of reset
    // stm32_interface.leave_reset().await;

    // read some packets
    // defmt::info!("wait for drib packets");




    let mut ctr = 0;
    loop {
        front_left_stm32_interface.read_latest_packet().await;
        // red_led.toggle();
        ctr += 1;
        if ctr > 1000 {
            ctr = 0;
            red_led.toggle();
        }
    }

    defmt::info!("end of program");
    loop {}

    loop {
        defmt::info!("end of program");
        Timer::after(Duration::from_millis(1000)).await;
    }
}