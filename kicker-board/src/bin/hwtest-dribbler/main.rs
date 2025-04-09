#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]
#![feature(type_alias_impl_trait)]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::{
    bind_interrupts, gpio::Pull, peripherals, usart::{self, Uart}
};
use embassy_stm32::{
    interrupt, pac::Interrupt
};

use ateam_lib_stm32::{drivers::boot::stm32_interface::{self, Stm32Interface}, idle_buffered_uart_spawn_tasks, static_idle_buffered_uart, uart};

use ateam_kicker_board::{tasks::get_system_config, *};

use panic_probe as _;

include_external_cpp_bin! {WHEEL_FW_IMG, "wheel.bin"}

const MAX_TX_PACKET_SIZE: usize = 64;
const TX_BUF_DEPTH: usize = 3;
const MAX_RX_PACKET_SIZE: usize = 64;
const RX_BUF_DEPTH: usize = 20;

static_idle_buffered_uart!(DRIB, MAX_RX_PACKET_SIZE, RX_BUF_DEPTH, MAX_TX_PACKET_SIZE, TX_BUF_DEPTH, #[link_section = ".axisram.buffers"]);

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CORDIC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let sys_config = get_system_config(tasks::ClkSource::InternalOscillator);
    let p = embassy_stm32::init(sys_config);

    info!("kicker startup!");

    interrupt::InterruptExt::set_priority(embassy_stm32::interrupt::CORDIC, embassy_stm32::interrupt::Priority::P7);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(Interrupt::CORDIC);


    let initial_motor_controller_uart_conifg = stm32_interface::get_bootloader_uart_config();

    let drib_uart = Uart::new(p.USART3, p.PE15, p.PB10, Irqs, p.DMA1_CH1, p.DMA1_CH2, initial_motor_controller_uart_conifg).unwrap();

    DRIB_IDLE_BUFFERED_UART.init();
    idle_buffered_uart_spawn_tasks!(uart_queue_spawner, DRIB, drib_uart);

    let mut drib_motor = Stm32Interface::new_from_pins(
        &DRIB_IDLE_BUFFERED_UART,
        DRIB_IDLE_BUFFERED_UART.get_uart_read_queue(), DRIB_IDLE_BUFFERED_UART.get_uart_write_queue(), 
        p.PE13, p.PE14,
        Pull::None, true);

    defmt::info!("programming firmware image");

    let res = drib_motor.load_firmware_image(WHEEL_FW_IMG).await;
    if res.is_err() {
        defmt::error!("failed to load dribbler firmware");
    } else {
        defmt::info!("loaded dribbler firmware");
    }

    loop {}
}