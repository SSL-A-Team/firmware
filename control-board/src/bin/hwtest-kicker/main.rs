#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(const_mut_refs)]
#![feature(sync_unsafe_cell)]


use ateam_control_board::{
    drivers::kicker::Kicker, get_system_config, include_kicker_bin, pins::{KickerRxDma, KickerTxDma, KickerUart}, stm32_interface::{self, Stm32Interface},
};
use ateam_lib_stm32::{make_uart_queue_pair, queue_pair_register_and_spawn};
use defmt::info;
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::{Level, Output, Speed, Pull}, interrupt, pac::Interrupt, usart::Uart
};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;

include_kicker_bin! {KICKER_FW_IMG, "hwtest-kick.bin"}

const MAX_TX_PACKET_SIZE: usize = 16;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 20;

make_uart_queue_pair!(KICKER,
    KickerUart, KickerRxDma, KickerTxDma,
    MAX_RX_PACKET_SIZE, RX_BUF_DEPTH,
    MAX_TX_PACKET_SIZE, TX_BUF_DEPTH,
    #[link_section = ".axisram.buffers"]);


static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let stm32_config = get_system_config();
    let p = embassy_stm32::init(stm32_config);

    defmt::info!("Kicker system init");

    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CEC, embassy_stm32::interrupt::Priority::P5);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CEC);

    // let kicker_det = Input::new(p.PG8, Pull::Up);
    // if kicker_det.is_high() {
    //     defmt::warn!("kicker appears unplugged!");
    // }

    let mut kicker_pwr_pin = Output::new(p.PG8, Level::Low, Speed::Medium);

    kicker_pwr_pin.set_high();
    defmt::info!("force power off kicker");
    Timer::after_millis(2000).await;
    kicker_pwr_pin.set_low();
    defmt::info!("kicker force power off done");

    defmt::info!("attempting to power on kicker.");
    Timer::after_millis(1000).await;
    kicker_pwr_pin.set_high();
    Timer::after_millis(200).await;
    kicker_pwr_pin.set_low();
    defmt::info!("power on attempt done");

    // loop {
    //     Timer::after_millis(1000).await;
    // }

    let kicker_usart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        ateam_control_board::SystemIrqs,
        p.DMA2_CH4,
        p.DMA2_CH5,
        stm32_interface::get_bootloader_uart_config(),
    ).unwrap();

    defmt::info!("init uart");

    let (kicker_tx, kicker_rx) = Uart::split(kicker_usart);
    queue_pair_register_and_spawn!(uart_queue_spawner, KICKER, kicker_rx, kicker_tx);

    defmt::info!("start qs");

    let kicker_stm32_interface = Stm32Interface::new_from_pins(
        &KICKER_RX_UART_QUEUE,
        &KICKER_TX_UART_QUEUE,
        p.PA8,
        p.PA9,
        Pull::Up,
        true
    );

    info!("flashing kicker...");

    let kicker_firmware_image = KICKER_FW_IMG;
    let mut kicker = Kicker::new(kicker_stm32_interface, kicker_firmware_image);
    let res = kicker.init_default_firmware_image(true).await;

    if res.is_err() {
        defmt::warn!("kicker flashing failed.");
        loop {}
    } else {
        info!("kicker flash complete");
    }

    kicker.set_telemetry_enabled(true);

    let mut main_loop_rate_ticker = Ticker::every(Duration::from_millis(10));
    loop {
        kicker.process_telemetry();

        // TODO print some telemetry or something
        defmt::info!("high voltage: {}, battery voltage: {}", kicker.hv_rail_voltage(), kicker.battery_voltage());

        kicker.send_command();

        main_loop_rate_ticker.next().await;
    }
}
