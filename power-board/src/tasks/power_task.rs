use core::fmt;

use ateam_lib_stm32::{filter::WindowAvergingFilter, math::range::Range, power::PowerRail};
use embassy_executor::Spawner;
use embassy_stm32::{adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime}, peripherals::ADC1};
use embassy_time::{Duration, Instant, Ticker};

use crate::{adc_raw_to_mv, adc_raw_vrefint_to_mv, config::{POWER_RAIL_12V0_PARAMETERS, POWER_RAIL_3V3_PARAMETERS, POWER_RAIL_5V0_PARAMETERS, POWER_RAIL_BATTERY_PARAMETERSL}, pins::*};

const BATTERY_CELL_READ_INTERVAL: Duration = Duration::from_millis(1300);
const POWER_RAIL_FILTER_WINDOW_SIZE: usize = 10;

#[derive(Debug)]
pub struct PowerRailAdcSamples {
    pub rail_3v3: f32,
    pub rail_5v0: f32,
    pub rail_12v0: f32,
    pub battery: f32,
}

impl Default for PowerRailAdcSamples {
    fn default() -> Self {
        Self { battery: Default::default(), rail_12v0: Default::default(), rail_5v0: Default::default(), rail_3v3: Default::default() }
    }
}

impl PowerRailAdcSamples {
    pub fn new_from_samples(samples: &[u16]) -> Result<Self, Self> {
        if samples.len() == 4 {
            Ok(Self {
                rail_3v3:  adc_raw_to_mv(samples[0]),    
                rail_5v0:  adc_raw_to_mv(samples[1]),
                rail_12v0: adc_raw_to_mv(samples[2]),
                battery:   adc_raw_to_mv(samples[3]),
            })    
        } else if samples.len() == 5 {
            Ok(Self {
                rail_3v3:  adc_raw_vrefint_to_mv(samples[0], samples[4]),
                rail_5v0:  adc_raw_vrefint_to_mv(samples[1], samples[4]),
                rail_12v0: adc_raw_vrefint_to_mv(samples[2], samples[4]),
                battery:   adc_raw_vrefint_to_mv(samples[3], samples[4]),
            })
        } else {
            Err(Self::default())
        }
    }
}



pub struct BatteryAdcSamples {
    pub cell1: f32,
    pub cell2: f32,
    pub cell3: f32,
    pub cell4: f32,
    pub cell5: f32,
    pub cell6: f32,
    pub vbatt: f32,
}

impl fmt::Debug for BatteryAdcSamples {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BatteryAdcSamples").field("cell1", &self.cell1).field("cell2", &self.cell2).field("cell3", &self.cell3).field("cell4", &self.cell4).field("cell5", &self.cell5).field("cell6", &self.cell6).field("vbatt", &self.vbatt).finish()
    }
}

impl Default for BatteryAdcSamples {
    fn default() -> Self {
        Self { cell1: Default::default(), cell2: Default::default(), cell3: Default::default(), cell4: Default::default(), cell5: Default::default(), cell6: Default::default(), vbatt: Default::default() }
    }
}

impl BatteryAdcSamples {
    pub fn new_from_samples(samples: &[u16]) -> Result<Self, Self> {
        if samples.len() == 7 {
            Ok(Self {
                cell1: adc_raw_to_mv(samples[0]),
                cell2: adc_raw_to_mv(samples[1]),
                cell3: adc_raw_to_mv(samples[2]),
                cell4: adc_raw_to_mv(samples[3]),
                cell5: adc_raw_to_mv(samples[4]),
                cell6: adc_raw_to_mv(samples[5]),
                vbatt: adc_raw_to_mv(samples[6]),
            })    
        } else if samples.len() == 8 {
            Ok(Self {
                cell1: adc_raw_vrefint_to_mv(samples[0], samples[7]),
                cell2: adc_raw_vrefint_to_mv(samples[1], samples[7]),
                cell3: adc_raw_vrefint_to_mv(samples[2], samples[7]),
                cell4: adc_raw_vrefint_to_mv(samples[3], samples[7]),
                cell5: adc_raw_vrefint_to_mv(samples[4], samples[7]),
                cell6: adc_raw_vrefint_to_mv(samples[5], samples[7]),
                vbatt: adc_raw_vrefint_to_mv(samples[6], samples[7]),
            })
        } else {
            Err(Self::default())
        }
    }
}

#[macro_export]
macro_rules! create_power_task {
    ($spawner:ident, $p:ident) => {
        ateam_power_board::tasks::power_task::start_power_task(&$spawner,
            $p.ADC1, $p.DMA1_CH1,
            $p.PA0, $p.PA1, $p.PA2, $p.PA3, $p.PA4, $p.PA5,
            $p.PB1, $p.PA7, $p.PA6, $p.PB10, $p.PB2).await;
    };
}

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

    /////////////////
    //  ADC Setup  //
    /////////////////

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

    let mut power_rail_adc_raw_samples: [u16; 5] = [0; 5];
    let mut battery_cell_adc_raw_samples: [u16; 8] = [0; 8];

    // ADC needs to polled no faster than every 0.793Hz = 1261ms due to the very high impedance inputs
    // and no active amplification. This is to keep powered off current draw on the battery very low.
    let mut loop_ticker = Ticker::every(Duration::from_millis(100));
    let mut last_battery_cell_read_time = Instant::now();

    ////////////////////////
    //  Power Rail Setup  //
    ////////////////////////
    
    let mut power_rail_battery = PowerRail::new(POWER_RAIL_BATTERY_PARAMETERSL,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, _>::new(),
        Range::new(0.0, 2062.0),
        Range::new(0.0, 25.2));

    let mut power_rail_12v0 = PowerRail::new(POWER_RAIL_12V0_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, _>::new(),
        Range::new(0.0, 1741.0),
        Range::new(0.0, 12.0));

    let mut power_rail_5v0 = PowerRail::new(POWER_RAIL_5V0_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, _>::new(),
        Range::new(0.0, 1745.0),
        Range::new(0.0, 5.0));

    let mut power_rail_3v3 = PowerRail::new(POWER_RAIL_3V3_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, _>::new(),
        Range::new(0.0, 1750.0),
        Range::new(0.0, 3.3));

    ///////////////////////////
    //  Battery Model Setup  //
    ///////////////////////////

    // TOOD setup battery model here

    loop {
        ////////////////////////////////////////////
        //  Read and Convert Power Rail Voltages  //
        ////////////////////////////////////////////

        // could eventually split this into two sequence
        // very high z and high z if the rails need to be samples more often
        let power_rail_read_seq = [
            (&mut power_rail_vbatt_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_12v0_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_5v0_adc_pin, SampleTime::CYCLES160_5),
            (&mut power_rail_3v3_adc_pin, SampleTime::CYCLES160_5),
            (&mut vrefint_channel, SampleTime::CYCLES160_5),
        ].into_iter();
        adc.read(&mut adc_dma, power_rail_read_seq, &mut power_rail_adc_raw_samples).await;

        // covert power rails
        let power_rail_adc_voltages = PowerRailAdcSamples::new_from_samples(&power_rail_adc_raw_samples).expect("invalid slice length on power rail adc sample conversion");
        power_rail_battery.add_rail_voltage_sample(power_rail_adc_voltages.battery);
        power_rail_12v0.add_rail_voltage_sample(power_rail_adc_voltages.rail_12v0);
        power_rail_5v0.add_rail_voltage_sample(power_rail_adc_voltages.rail_5v0);
        power_rail_3v3.add_rail_voltage_sample(power_rail_adc_voltages.rail_3v3);

        defmt::info!("power rail voltages: Battery {}, 12v0 {}, 5v0 {}, 3v3 {}", power_rail_battery.get_rail_voltage(), power_rail_12v0.get_rail_voltage(), power_rail_5v0.get_rail_voltage(), power_rail_3v3.get_rail_voltage());

        ///////////////////////////////
        //  Check Power Rail Errors  //
        ///////////////////////////////

        // analyze for error conditions
            // Vbatt voltage too low
            // Vbatt voltage too high
            // 12v0 voltage too low
            // 12v0 voltage too high
            // 5v0 voltage too low
            // 5v0 voltage too high
            // 3v3 voltage too low
            // 3v3 voltage too high



        //////////////////////////
        //  Battery Monitoring  //
        //////////////////////////

        if last_battery_cell_read_time.elapsed() > BATTERY_CELL_READ_INTERVAL {
            let battery_cell_read_seq = [
                (&mut cell1_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell2_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell3_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell4_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell5_adc_pin, SampleTime::CYCLES160_5),
                (&mut cell6_adc_pin, SampleTime::CYCLES160_5),
                (&mut power_rail_vbatt_before_lsw_adc_pin, SampleTime::CYCLES160_5),
                (&mut vrefint_channel, SampleTime::CYCLES160_5),
            ].into_iter();
            adc.read(&mut adc_dma, battery_cell_read_seq, &mut battery_cell_adc_raw_samples).await;
            // defmt::info!("vrefint channel {}", battery_cell_adc_raw_samples[7]);
            // defmt::info!("battery cell raw samples {} {}", battery_cell_adc_raw_samples[0], adc_raw_vrefint_to_mv(battery_cell_adc_raw_samples[0], battery_cell_adc_raw_samples[7]));

            last_battery_cell_read_time = Instant::now();

            let battery_cell_adc_voltages: BatteryAdcSamples = BatteryAdcSamples::new_from_samples(&battery_cell_adc_raw_samples).expect("invalid slice length on battery cell adc conversion");
            defmt::info!("battery cell adc voltages {}", battery_cell_adc_voltages.vbatt);
            
            // input to battery model

            // check for battery errors
            // Vbatt PMIC differential
            // battery balance - any cell too high
            // battery balance - any cell too low
            // battery balance - any cell difference too high
        }

        // sent pubsub message to coms task

        loop_ticker.next().await;
    }
}

pub async fn start_power_task(spawner: &Spawner,
    adc: PowerAdc,
    adc_dma: PowerAdcDma,
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
    spawner.spawn(power_task_entry(
        adc, adc_dma,
        cell1_adc_pin, cell2_adc_pin, cell3_adc_pin, cell4_adc_pin, cell5_adc_pin, cell6_adc_pin,
        power_rail_12v0_adc_pin, power_rail_5v0_adc_pin, power_rail_3v3_adc_pin, power_rail_vbatt_before_lsw_adc_pin, power_rail_vbatt_adc_pin
    )).expect("failed to spawn power task");
}