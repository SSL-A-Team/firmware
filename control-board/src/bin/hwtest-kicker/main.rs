#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]


use ateam_control_board::{
    drivers::kicker::Kicker, get_system_config, include_kicker_bin,
};
use ateam_lib_stm32::{
    drivers::boot::stm32_interface::{self, Stm32Interface},
    idle_buffered_uart_spawn_tasks, static_idle_buffered_uart};
use defmt::info;
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::Pull, interrupt, pac::Interrupt, usart::Uart
};
use embassy_time::{Duration, Ticker};
use panic_probe as _;

include_kicker_bin! {KICKER_FW_IMG, "hwtest-coms.bin"}

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 16;
const RX_BUF_DEPTH: usize = 20;

static_idle_buffered_uart!(KICKER, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);


static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[allow(non_snake_case)]
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

    let kicker_usart = Uart::new(
        p.UART8,
        p.PE0,
        p.PE1,
        ateam_control_board::SystemIrqs,
        p.DMA2_CH2,
        p.DMA2_CH3,
        stm32_interface::get_bootloader_uart_config(),
    ).unwrap();

    defmt::info!("init uart");

    KICKER_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, KICKER, kicker_usart);

    defmt::info!("start qs");

    let kicker_stm32_interface = Stm32Interface::new_from_pins(
        &KICKER_IDLE_BUFFERED_UART,
        KICKER_IDLE_BUFFERED_UART.get_uart_read_queue(),
        KICKER_IDLE_BUFFERED_UART.get_uart_write_queue(),
        p.PG2,
        p.PG3,
        Pull::Up,
        true
    );

    info!("flashing kicker...");

    let kicker_firmware_image = KICKER_FW_IMG;
    let mut kicker = Kicker::new(kicker_stm32_interface, kicker_firmware_image);
    let force_flash = true;
    let res = kicker.init_default_firmware_image(force_flash).await;

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
