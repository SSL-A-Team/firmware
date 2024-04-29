#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

use core::{
    cell::SyncUnsafeCell,
    sync::atomic::AtomicU16
};

use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, gpio::{Level, Output, Pull, Speed}, peripherals::{self, *}, usart::{self, *}
};
use embassy_executor::Executor;
use embassy_time::Timer;

use defmt::*;
use defmt_rtt as _; 
use panic_probe as _;

use static_cell::StaticCell;

use ateam_lib_stm32::{queue::Buffer, uart::queue::{UartReadQueue, UartWriteQueue}};

type ComsUartModule = USART2;
type ComsUartTxDma = DMA1_CH0;
type ComsUartRxDma = DMA1_CH1;

type LedGreenPin = PB0;
type LedYellowPin = PE1;
type LedRedPin = PB14;
type UserBtnPin = PC13;
type UserBtnExti = EXTI13;

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 10;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 10;

// control communications tx buffer
const COMS_BUFFER_VAL_TX: SyncUnsafeCell<Buffer<MAX_TX_PACKET_SIZE>> = 
    SyncUnsafeCell::new(Buffer::EMPTY);
#[link_section = ".axisram.buffers"]
static COMS_BUFFERS_TX: [SyncUnsafeCell<Buffer<MAX_TX_PACKET_SIZE>>; TX_BUF_DEPTH] = 
    [COMS_BUFFER_VAL_TX; TX_BUF_DEPTH];
static COMS_QUEUE_TX: UartWriteQueue<ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH> = 
    UartWriteQueue::new(&COMS_BUFFERS_TX);

// control communications rx buffer
const COMS_BUFFER_VAL_RX: SyncUnsafeCell<Buffer<MAX_RX_PACKET_SIZE>> = 
    SyncUnsafeCell::new(Buffer::EMPTY);
#[link_section = ".axisram.buffers"]
static COMS_BUFFERS_RX: [SyncUnsafeCell<Buffer<MAX_RX_PACKET_SIZE>>; RX_BUF_DEPTH] = 
    [COMS_BUFFER_VAL_RX; RX_BUF_DEPTH];
static COMS_QUEUE_RX: UartReadQueue<ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH> = 
    UartReadQueue::new(&COMS_BUFFERS_RX);

static LOOP_RATE_MS: AtomicU16 = AtomicU16::new(100);

struct StupidPacket {
    fields_of_minimal_intelligence: [usize; 16],
}

#[embassy_executor::task]
async fn rx_task(coms_reader: &'static UartReadQueue<ComsUartModule, ComsUartRxDma, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH>) {
    let mut rx_packet: StupidPacket = StupidPacket {
        fields_of_minimal_intelligence: [0x55AA55AA; 16]
    };
    
    loop {
        while let Ok(res) = coms_reader.try_dequeue() {
            let buf = res.data();

            if buf.len() != core::mem::size_of::<StupidPacket>() {
                defmt::warn!("got invalid packet of len {:?} data: {:?}", buf.len(), buf);
                continue;
            }

            // reinterpreting/initializing packed ffi structs is nearly entirely unsafe
            unsafe {
                // copy receieved uart bytes into packet
                let state = &mut rx_packet as *mut _ as *mut u8;
                for i in 0..core::mem::size_of::<StupidPacket>() {
                    *state.offset(i as isize) = buf[i];
                }                
            }
        }

        let millis = LOOP_RATE_MS.load(core::sync::atomic::Ordering::SeqCst);
        Timer::after_millis(millis as u64).await;
    }
}

#[embassy_executor::task]
async fn tx_task(coms_writer: &'static UartWriteQueue<ComsUartModule, ComsUartTxDma, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH>) {
    let tx_packet: StupidPacket = StupidPacket {
        fields_of_minimal_intelligence: [0x55AA55AA; 16]
    };

    loop {
        // raw interpretaion of a struct for wire transmission is unsafe
        unsafe {
            // get a slice to packet for transmission
            let struct_bytes = core::slice::from_raw_parts(
                (&tx_packet as *const StupidPacket) as *const u8,
                core::mem::size_of::<StupidPacket>(),
            );

            // send the packet
            let _res = coms_writer.enqueue_copy(struct_bytes);
        }

        let millis = LOOP_RATE_MS.load(core::sync::atomic::Ordering::SeqCst);
        Timer::after_millis(millis as u64).await;
    }
}

#[embassy_executor::task]
async fn handle_btn_press(usr_btn_pin: UserBtnPin,
    usr_btn_exti: UserBtnExti,
    led_green_pin: LedGreenPin,
    led_yellow_pin: LedYellowPin,
    led_red_pin: LedRedPin) {

    let mut usr_btn = ExtiInput::new(usr_btn_pin, usr_btn_exti, Pull::Down);

    let mut green_led = Output::new(led_green_pin, Level::Low, Speed::Medium);
    let mut yellow_led = Output::new(led_yellow_pin, Level::Low, Speed::Medium);
    let mut red_led = Output::new(led_red_pin, Level::Low, Speed::Medium);


    loop {
        usr_btn.wait_for_rising_edge().await;

        green_led.set_low();
        yellow_led.set_low();
        red_led.set_low();

        defmt::info!("updating loop rates");

        let cur_loop_rate = LOOP_RATE_MS.load(core::sync::atomic::Ordering::SeqCst);
        let mut new_loop_rate = 100;
        if cur_loop_rate == 100 {
            new_loop_rate = 10;

            red_led.set_high();
        } else if cur_loop_rate == 10 {
            new_loop_rate = 1;

            yellow_led.set_high();
        } else {
            green_led.set_high();
        }
        LOOP_RATE_MS.store(new_loop_rate, core::sync::atomic::Ordering::SeqCst);
    }
}

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> !{
    // this actually gets us 64MHz peripheral bus clock
    let stm32_config: embassy_stm32::Config = Default::default();
    let p = embassy_stm32::init(stm32_config);
    
    //////////////////////////////////
    //  COMMUNICATIONS TASKS SETUP  //
    //////////////////////////////////

    let mut coms_uart_config = Config::default();
    coms_uart_config.baudrate = 2_000_000; // 2 Mbaud
    coms_uart_config.parity = Parity::ParityEven;
    coms_uart_config.stop_bits = StopBits::STOP1;

    let coms_usart = Uart::new(
        p.USART2,
        p.PD6,
        p.PD5,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        coms_uart_config,
    ).unwrap();

    let (coms_uart_tx, coms_uart_rx) = Uart::split(coms_usart);

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(handle_btn_press(p.PC13, p.EXTI13, p.PB0, p.PE1, p.PB14)));
        unwrap!(spawner.spawn(COMS_QUEUE_RX.spawn_task(coms_uart_rx)));
        unwrap!(spawner.spawn(COMS_QUEUE_TX.spawn_task(coms_uart_tx)));
        unwrap!(spawner.spawn(rx_task(&COMS_QUEUE_RX)));
        unwrap!(spawner.spawn(tx_task(&COMS_QUEUE_TX)));
    });
}