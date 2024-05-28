#![no_std]
#![no_main]

use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt;

use defmt_rtt as _; 

use ateam_control_board::{get_system_config, robot_state::SharedRobotState, tasks::user_io_task::start_io_task};


use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static UART_QUEUE_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    UART_QUEUE_EXECUTOR.on_interrupt();
}

#[embassy_executor::main]
async fn main(main_spawner: embassy_executor::Spawner) {
    // init system
    let sys_config = get_system_config();
    let p = embassy_stm32::init(sys_config);

    defmt::info!("embassy HAL configured.");

    let robot_state = ROBOT_STATE.take();

    ////////////////////////
    //  setup task pools  //
    ////////////////////////

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    ///////////////////
    //  start tasks  //
    ///////////////////

    start_io_task(main_spawner,
        robot_state,
        p.PD6, p.PD5, p.EXTI6, p.EXTI5,
        p.PG7, p.PG6, p.PG5, p.PG4, p.PG3, p.PG2, p.PD15,
        p.PG12, p.PG11, p.PG10, p.PG9,
        p.PF3, p.PF2, p.PF1, p.PF0,
        p.PD0, p.PD1, p.PD3, p.PD4, p.PD14);

    loop {
        Timer::after_millis(1000).await;
    }
}