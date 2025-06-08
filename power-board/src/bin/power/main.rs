#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(sync_unsafe_cell)]

use ateam_lib_stm32::audio::songs::SongId;
use ateam_lib_stm32::audio::AudioCommand;
use ateam_power_board::pins::{AudioPubSub, TelemetryPubSub};
use ateam_power_board::power_state::SharedPowerState;
use ateam_power_board::{create_power_task, create_coms_task, create_audio_task};
use defmt::*;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

static SHARED_POWER_STATE: SharedPowerState = SharedPowerState::new();

static AUDIO_PUBSUB: AudioPubSub = PubSubChannel::new();
static TELEMETRY_CHANNEL: TelemetryPubSub = PubSubChannel::new();

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

    let pwr_btn = Input::new(p.PB15, Pull::None);
    let mut shutdown_ind = Output::new(p.PA15, Level::High, Speed::Low);
    let mut kill_sig = OutputOpenDrain::new(p.PA8, Level::High, Speed::Low);

    let mut en_12v0 = Output::new(p.PB6, Level::Low, Speed::Low);
    let mut en_3v3 = Output::new(p.PB7, Level::Low, Speed::Low);
    let mut en_5v0 = Output::new(p.PB8, Level::Low, Speed::Low);

    let shared_power_state = &SHARED_POWER_STATE;
    let main_audio_publisher = AUDIO_PUBSUB.publisher().unwrap();

    sequence_power_on(&mut en_3v3, &mut en_5v0, &mut en_12v0).await;
    main_audio_publisher.publish(AudioCommand::PlaySong(SongId::PowerOn)).await;

    interrupt::USART2.set_priority(Priority::P6);
    let uart_queue_spawner = UART_QUEUE_EXECUTOR.start(interrupt::USART2);

    //////////////////////////////////////
    //  setup inter-task coms channels  //
    //////////////////////////////////////

    // commands channel
    let power_telemetry_publisher = TELEMETRY_CHANNEL.publisher().unwrap();
    let coms_telemetry_subscriber = TELEMETRY_CHANNEL.subscriber().unwrap();

    // audio channel
    let power_audio_publisher = AUDIO_PUBSUB.publisher().unwrap();
    let coms_audio_publisher = AUDIO_PUBSUB.publisher().unwrap();
    let power_audio_subscriber = AUDIO_PUBSUB.subscriber().unwrap();

    // start power task
    create_power_task!(spawner, shared_power_state, power_telemetry_publisher, power_audio_publisher, p);

    // start coms task
    create_coms_task!(spawner, uart_queue_spawner, shared_power_state, coms_telemetry_subscriber, coms_audio_publisher, p);

    // TODO: start audio task
    create_audio_task!(spawner, power_audio_subscriber, p);

    // TODO: start LED animation task


    let mut main_loop_ticker = Ticker::every(Duration::from_millis(10));
    loop {
        // read pwr button
        // shortest possible interrupt from pwr btn controller is 32ms
        if pwr_btn.get_level() == Level::Low || SHARED_POWER_STATE.get_shutdown_requested().await {
            shutdown_ind.set_low();  // indicate shutdown request
            let shutdown_requested_time = Instant::now();
            SHARED_POWER_STATE.set_shutdown_requested(true).await;  // update power board state
            loop {
                let cur_robot_state = shared_power_state.get_state().await;

                if !SHARED_POWER_STATE.get_shutdown_requested().await {
                    defmt::info!("MAIN TASK: Shutdown cancelled");
                    shutdown_ind.set_high();
                    break
                }

                if SHARED_POWER_STATE.get_shutdown_ready().await 
                    || Instant::now() > shutdown_requested_time + Duration::from_secs(30) 
                    || !cur_robot_state.coms_established {
                    defmt::info!("MAIN TASK: Shutdown acknowledged, turning off power");
                    main_audio_publisher.publish(AudioCommand::PlaySong(SongId::PowerOff)).await;
                    Timer::after_millis(500).await;
                    sequence_power_off(&mut en_3v3, &mut en_5v0, &mut en_12v0).await;
                    kill_sig.set_low();
                    break
                }

                Timer::after_millis(10).await;
            }
        }

        main_loop_ticker.next().await;
    }
}

async fn sequence_power_on(en_3v3: &mut Output<'static>, en_5v0: &mut Output<'static>, en_12v0: &mut Output<'static>) {
    Timer::after_millis(20).await;
    en_3v3.set_high();

    Timer::after_millis(10).await;
    en_5v0.set_high();

    Timer::after_millis(10).await;
    en_12v0.set_high();
}

async fn sequence_power_off(en_3v3: &mut Output<'static>, en_5v0: &mut Output<'static>, en_12v0: &mut Output<'static>) {
    en_12v0.set_low();
    Timer::after_millis(100).await;

    en_5v0.set_low();
    Timer::after_millis(10).await;

    en_3v3.set_low();
}
