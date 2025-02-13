use ateam_lib_stm32::drivers::switches::button::{AdvExtiButton, ADV_BTN_EVENT_DOUBLE_TAP};
use embassy_executor::Spawner;
use embassy_futures::select;
use embassy_stm32::gpio::{Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_time::Timer;

use crate::{pins::{PowerBtnPressedIntExti, PowerBtnPressedIntPin, PowerKillPin, ShutdownInitiatedLedPin}, robot_state::{self, SharedRobotState}};

pub const HARD_SHUTDOWN_TIME_MS: u64 = 15000;

#[macro_export]
macro_rules! create_shutdown_task {
    ($spawner:ident, $robot_state:ident, $p:ident) => {
        ateam_control_board::tasks::shutdown_task::start_shutdown_task($spawner,
            $robot_state,
            $p.PE11, $p.EXTI11, $p.PE10, $p.PF4);
    };
}

#[embassy_executor::task]
async fn shutdown_task_entry(robot_state: &'static SharedRobotState,
    mut power_btn: AdvExtiButton,
    mut system_kill_pin: OutputOpenDrain<'static>,
    mut shutdown_initiated_led: Output<'static>) {

    defmt::info!("shutdown task initialized");

    power_btn.wait_for_btn_event(ADV_BTN_EVENT_DOUBLE_TAP).await;
    
    robot_state.flag_shutdown_requested();
    shutdown_initiated_led.set_high();

    defmt::warn!("shutdown initiated via user btn! syncing...");

    // wait for tasks to flag shutdown complete, or hard temrinate after hard shutdown time
    select::select(async move {
        loop {
            Timer::after_millis(100).await;

            if robot_state.kicker_shutdown_complete() || robot_state.get_kicker_inop() {
                break;
            }
        }
    }, Timer::after_millis(HARD_SHUTDOWN_TIME_MS)).await;

    defmt::info!("initiating power off");
    Timer::after_millis(100).await;

    loop {
        system_kill_pin.set_low();
    }
}

pub fn start_shutdown_task(spawner: Spawner,
    robot_state: &'static SharedRobotState,
    power_btn_pin: PowerBtnPressedIntPin,
    power_btn_pin_exti: PowerBtnPressedIntExti,
    system_kill_pin: PowerKillPin,
    shutdown_initiated_led_pin: ShutdownInitiatedLedPin) {

    let power_btn: AdvExtiButton = AdvExtiButton::new_from_pins(power_btn_pin, power_btn_pin_exti, true);
    let system_kill_output = OutputOpenDrain::new(system_kill_pin, Level::High, Speed::Medium, Pull::None);
    let shutdown_initiated_led = Output::new(shutdown_initiated_led_pin, Level::Low, Speed::Medium);

    spawner.spawn(shutdown_task_entry(robot_state, power_btn, system_kill_output, shutdown_initiated_led)).unwrap();
}