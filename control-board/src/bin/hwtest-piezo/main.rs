#![no_std]
#![no_main]

use ateam_lib_stm32::{audio::tone_player::TonePlayer, drivers::audio::buzzer::Buzzer};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::OutputType,
    interrupt,
    time::khz,
    timer::{
        simple_pwm::{PwmPin, SimplePwm},
        Channel,
    },
};

use defmt_rtt as _;

use ateam_control_board::{
    create_io_task, get_system_config, robot_state::SharedRobotState, songs::TEST_SONG,
};

use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> =
    ConstStaticCell::new(SharedRobotState::new());

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

    create_io_task!(main_spawner, robot_state, p);

    let ch2 = PwmPin::new(p.PE6, OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM15,
        None,
        Some(ch2),
        None,
        None,
        khz(2),
        Default::default(),
    );

    let audio_driver = Buzzer::new(pwm, Channel::Ch2);
    let mut tone_player = TonePlayer::new(audio_driver);

    if tone_player.load_song(&TEST_SONG).is_err() {
        defmt::warn!("song uses pitch or duration outside of operating range");
    }

    tone_player.play_song().await;

    loop {
        Timer::after_millis(10).await;
    }
}
