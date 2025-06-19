use embassy_executor::Spawner;
use embassy_time::Timer;

#[macro_export]
macro_rules! create_led_task {
    ($spawner:ident, $p:ident) => {
        ateam_power_board::tasks::led_task::led_task_entry(&$spawner);
    };
}

#[embassy_executor::task]
async fn led_task_entry() {
    defmt::warn!("led task unimplemented");

    loop {
        // latch latest robot state

        // update animations

        Timer::after_millis(1000).await;
    }
}

pub async fn start_power_task(spawner: &Spawner,
    ) {

    // init Apa102 animation stuff

    spawner.spawn(led_task_entry()).expect("failed to spawn led task");
}