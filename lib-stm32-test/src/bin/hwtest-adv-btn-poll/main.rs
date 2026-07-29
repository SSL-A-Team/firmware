#![no_std]
#![no_main]

use ateam_lib_stm32::drivers::switches::button::AdvExtiButton;

use defmt_rtt as _;
use embassy_stm32::{bind_interrupts, exti};
use embassy_time::Timer;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    EXTI15_10 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    // this actually gets us 64MHz peripheral bus clock
    let stm32_config: embassy_stm32::Config = Default::default();
    let p = embassy_stm32::init(stm32_config);

    let mut adv_btn: AdvExtiButton = AdvExtiButton::new_from_pins(p.PC13, p.EXTI13, Irqs, false);

    loop {
        if let Some(ev) = adv_btn.poll_btn_event() {
            defmt::info!("btn event: {}", ev);
        }

        Timer::after_millis(10).await;
    }
}
