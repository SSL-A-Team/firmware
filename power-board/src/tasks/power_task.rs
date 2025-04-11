use embassy_stm32::{adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime}, peripherals::ADC1};
use embassy_time::{Duration, Ticker};

use crate::pins::*;


#[embassy_executor::task]
async fn power_task_entry(
    adc: PowerAdc,
    mut adc_dma: PowerAdcDma,
    cell1_adc_pin: BatteryCell1VoltageMonitorPin,
    cell2_adc_pin: BatteryCell2VoltageMonitorPin,
    cell3_adc_pin: BatteryCell3VoltageMonitorPin,
    cell4_adc_pin: BatteryCell4VoltageMonitorPin,
    cell5_adc_pin: BatteryCell5VoltageMonitorPin,
    cell6_adc_pin: BatteryCell6VoltageMonitorPin,
    power_rail_12v0_adc_pin: Power12v0VoltageMonitorPin,
    power_rail_5v0_adc_pin: Power5v0VoltageMonitorPin,
    power_rail_3v3_adc_pin: Power3v3VoltageMonitorPin,
    power_rail_vbatt_before_lsw_adc_pin: BatteryPreLoadSwitchVoltageMonitorPin,
    power_rail_vbatt_adc_pin: BatteryVoltageMonitorPin,

) {

    let mut adc = Adc::new(adc);
    let mut cell1_adc_pin = cell1_adc_pin.degrade_adc();
    let mut cell2_adc_pin = cell2_adc_pin.degrade_adc();
    let mut cell3_adc_pin = cell3_adc_pin.degrade_adc();
    let mut cell4_adc_pin = cell4_adc_pin.degrade_adc();
    let mut cell5_adc_pin = cell5_adc_pin.degrade_adc();
    let mut cell6_adc_pin = cell6_adc_pin.degrade_adc();
    let mut power_rail_12v0_adc_pin  = power_rail_12v0_adc_pin.degrade_adc();
    let mut power_rail_5v0_adc_pin = power_rail_5v0_adc_pin.degrade_adc();
    let mut power_rail_3v3_adc_pin = power_rail_3v3_adc_pin.degrade_adc();
    let mut power_rail_vbatt_before_lsw_adc_pin = power_rail_vbatt_before_lsw_adc_pin.degrade_adc();
    let mut power_rail_vbatt_adc_pin = power_rail_vbatt_adc_pin.degrade_adc();
    let mut vrefint_channel: AnyAdcChannel<ADC1> = adc.enable_vrefint().degrade_adc();

    let mut adc_samples: [u16; 12] = [0; 12];

    // ADC needs to polled no faster than every 0.793Hz = 1261ms due to the very high impedance inputs
    // and no active amplification. This is to keep powered off current draw on the battery very low.
    let mut loop_ticker = Ticker::every(Duration::from_millis(1300));

    // TOOD setup battery model here

    loop {
        // read raw ADC values

        // could eventually split this into two sequence
        // very high z and high z if the rails need to be samples more often
        let adc_read_seq = [
            (&mut cell1_adc_pin, SampleTime::CYCLES160_5),
            (&mut cell2_adc_pin, SampleTime::CYCLES160_5),
            (&mut cell3_adc_pin, SampleTime::CYCLES160_5),
            (&mut cell4_adc_pin, SampleTime::CYCLES160_5),
            (&mut cell5_adc_pin, SampleTime::CYCLES160_5),
            (&mut cell6_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_12v0_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_5v0_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_3v3_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_vbatt_before_lsw_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_vbatt_adc_pin, SampleTime::CYCLES160_5),
            (&mut vrefint_channel, SampleTime::CYCLES160_5),
    
        ].into_iter();
        adc.read(&mut adc_dma, adc_read_seq, &mut adc_samples).await;


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

pub async fn start_power_task() {
    
}