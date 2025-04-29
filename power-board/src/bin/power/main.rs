#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(sync_unsafe_cell)]

use ateam_power_board::{create_power_task, pins::AudioPubSub};
use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

use ateam_lib_stm32::{audio::note::Beat, static_idle_buffered_uart_nl};

pub const TEST_SONG: [Beat; 2] = [
    Beat::Note { tone: 440, duration: 250_000 },
    Beat::Note { tone: 587, duration: 250_000 },
];

static AUDIO_PUBSUB: AudioPubSub = PubSubChannel::new();

const MAX_RX_PACKET_SIZE: usize = core::mem::size_of::<ateam_common_packets::bindings::PowerCommandPacket>();
const MAX_TX_PACKET_SIZE: usize = core::mem::size_of::<ateam_common_packets::bindings::PowerStatusPacket>();
const RX_BUF_DEPTH: usize = 3;
const TX_BUF_DEPTH: usize = 2;
static_idle_buffered_uart_nl!(COMS, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH);

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
#[allow(non_snake_case)]
unsafe fn USART2() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}



#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("power board startup.");

    let _pwr_btn = Input::new(p.PB15, Pull::None);
    let mut _shutdown_ind = Output::new(p.PA15, Level::High, Speed::Low);
    let mut _kill_sig = OutputOpenDrain::new(p.PA8, Level::High, Speed::Low);

    let en_12v0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let en_3v3 = Output::new(p.PB7, Level::Low, Speed::Low);
    let en_5v0 = Output::new(p.PB8, Level::Low, Speed::Low);

    sequence_power_on(en_3v3, en_5v0, en_12v0).await;

    interrupt::USART2.set_priority(Priority::P6);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(interrupt::USART2);
    
    create_power_task!(spawner, p);

    // TODO: start audio task

    // TODO: start LED animation task

    // TODO: start communications queues

    // let coms_uart = Uart::new(motor_fl_uart, motor_fl_rx_pin, motor_fl_tx_pin, SystemIrqs, motor_fl_tx_dma, motor_fl_rx_dma, initial_motor_controller_uart_conifg).unwrap();

    COMS_IDLE_BUFFERED_UART.init();
    // idle_buffered_uart_spawn_tasks!(uart_queue_spawner, COMS, )
    

    let mut main_loop_ticker = Ticker::every(Duration::from_secs(1));
    loop {
        // read packets
        // read channels

        // read pwr button

        // send packets

        main_loop_ticker.next().await;
    }
}

async fn sequence_power_on(mut en_3v3: Output<'static>, mut en_5v0: Output<'static>, mut en_12v0: Output<'static>) {
    Timer::after_millis(20).await;
    en_3v3.set_high();

    Timer::after_millis(10).await;
    en_5v0.set_high();

    Timer::after_millis(10).await;
    en_12v0.set_high();
}

async fn sequence_power_off(mut en_3v3: Output<'static>, mut en_5v0: Output<'static>, mut en_12v0: Output<'static>) {
    en_12v0.set_low();
    Timer::after_millis(100).await;

    en_5v0.set_low();
    Timer::after_millis(10).await;

    en_3v3.set_low();
}
