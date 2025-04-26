use embassy_executor::Spawner;
use embassy_time::Timer;

#[macro_export]
macro_rules! create_audio_task {
    ($spawner:ident, $p:ident) => {
        ateam_power_board::tasks::audio_task::audio_task_entry(&$spawner);
    };
}

#[embassy_executor::task]
async fn audio_task_entry() {
    defmt::warn!("audio task unimplemented");

    loop {
        // async await AudioCommand from a Subscriber

        // play song

        Timer::after_millis(1000).await;
    }
}

pub async fn start_power_task(spawner: &Spawner,
    ) {

    // init tone player

    spawner.spawn(audio_task_entry()).expect("failed to spawn audio task");
}