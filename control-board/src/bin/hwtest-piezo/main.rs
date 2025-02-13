#![no_std]
#![no_main]

use ateam_lib_stm32::{audio::tone_player::TonePlayer, drivers::audio::buzzer::Buzzer};
use embassy_executor::InterruptExecutor;
use embassy_stm32::{
    gpio::OutputType, interrupt, time::khz, timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}
};

use defmt_rtt as _; 

use ateam_control_board::{create_io_task, get_system_config, pins::BatteryVoltPubSub, robot_state::SharedRobotState, songs::TEST_SONG};


use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Timer;
// provide embedded panic probe
use panic_probe as _;
use static_cell::ConstStaticCell;

static ROBOT_STATE: ConstStaticCell<SharedRobotState> = ConstStaticCell::new(SharedRobotState::new());

static BATTERY_VOLT_CHANNEL: BatteryVoltPubSub = PubSubChannel::new();

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

    let battery_volt_publisher = BATTERY_VOLT_CHANNEL.publisher().unwrap();


    ///////////////////
    //  start tasks  //
    ///////////////////


            
    create_io_task!(main_spawner,
        robot_state,
        battery_volt_publisher,
        p);

    let ch2 = PwmPin::new_ch2(p.PE6, OutputType::PushPull);
    let pwm = SimplePwm::new(p.TIM15, None, Some(ch2), None, None, khz(2), Default::default());
    
    let audio_driver = Buzzer::new(pwm, Channel::Ch2);
    let mut tone_player = TonePlayer::new(audio_driver);

    if tone_player.load_song(&TEST_SONG).is_err() {
        defmt::warn!("song uses pitch or duration outside of operating range");
    }

    tone_player.play_song().await;



    // let max = pwm.get_max_duty();
    // pwm.enable(Channel::Ch2);


    // pwm.set_duty(Channel::Ch2, max / 2);

    // loop {
    //     pwm.set_frequency(hz(35));
    //     Timer::after_millis(1000).await;
    //     pwm.set_frequency(hz(7000));
    //     Timer::after_millis(1000).await;
    //     pwm.disable(Channel::Ch2);

    //     break;
    // }

    loop {
        Timer::after_millis(10).await;
    }
}