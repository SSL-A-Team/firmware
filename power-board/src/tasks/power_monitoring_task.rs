use embassy_time::{Duration, Ticker};


#[embassy_executor::task]
async fn power_mon_task_entry() {

    // ADC needs to polled no faster than every 0.793Hz = 1261ms due to the very high impedance inputs
    // and no active amplification. This is to keep powered off current draw on the battery very low.
    let mut loop_ticker = Ticker::every(Duration::from_millis(1300));

    // TOOD setup battery model here

    loop {
        // read raw ADC values

        // covert power rails

        // input to battery model

        // apply data filters

        // analyze for error conditions
            // Vbatt voltage too low
            // Vbatt voltage too high
            // Vbatt PMIC differential
            // 5v0 voltage too low
            // 5v0 voltage too high
            // 3v3 voltage too low
            // 3v3 voltage too high

            // battery balance - any cell too high
            // battery balance - any cell too low
            // battery balance - any cell difference too high

        // sent pubsub message to coms task

        loop_ticker.next().await;
    }
}

pub async fn start_power_mon_task() {
    
}