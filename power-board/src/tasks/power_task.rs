use core::mem::MaybeUninit;

use ateam_common_packets::bindings::BatteryInfoPacket;
use ateam_lib_stm32::{drivers::adc::AdcConverter, filter::WindowAvergingFilter, math::range::Range, power::{battery::LipoModel, PowerRail}};
use embassy_executor::Spawner;
use embassy_stm32::{adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime}, peripherals::ADC1};
use embassy_time::{Duration, Instant, Ticker};

use crate::{config::{LIPO6S_BALANCE_RAW_SAMPLES_TO_VOLTAGES, LIPO_BATTERY_CONFIG_6S, POWER_RAIL_12V0_PARAMETERS, POWER_RAIL_3V3_PARAMETERS, POWER_RAIL_5V0_PARAMETERS, POWER_RAIL_BATTERY_PARAMETERS}, pins::*, power_state::SharedPowerState};

const BATTERY_CELL_READ_INTERVAL: Duration = Duration::from_millis(1300);
const POWER_RAIL_FILTER_WINDOW_SIZE: usize = 10;

#[macro_export]
macro_rules! create_power_task {
    ($spawner:ident, $shared_power_state:ident, $telemetry_publisher:ident, $p:ident) => {
        ateam_power_board::tasks::power_task::start_power_task(&$spawner, $shared_power_state, $telemetry_publisher,
            $p.ADC1, $p.DMA1_CH1,
            $p.PA0, $p.PA1, $p.PA2, $p.PA3, $p.PA4, $p.PA5,
            $p.PB1, $p.PA7, $p.PA6, $p.PB10, $p.PB2).await;
    };
}

#[embassy_executor::task]
async fn power_task_entry(
    shared_power_state: &'static SharedPowerState,
    telemetry_publisher: TelemetryPublisher,
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
    let mut power_rail_voltage_samples: [u16; 4] = [0; 4];

    let mut battery_cell_adc_raw_samples: [u16; 8] = [0; 8];
    let mut battery_cell_voltage_samples: [f32; 7] = [0.0; 7];

    // From https://www.st.com/resource/en/datasheet/stm32g031g8.pdf
    // 6.3.3 Embedded internal reference voltage
    let mut adc_converter = AdcConverter::new(3000, 1212, true, false);

    // ADC needs to polled no faster than every 0.793Hz = 1261ms due to the very high impedance inputs
    // and no active amplification. This is to keep powered off current draw on the battery very low.
    let mut loop_ticker = Ticker::every(Duration::from_millis(100));
    let mut last_battery_cell_read_time = Instant::now();

    ////////////////////////
    //  Power Rail Setup  //
    ////////////////////////
    
    let mut power_rail_battery = PowerRail::new(POWER_RAIL_BATTERY_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, true, _>::new(),
        Range::new(0.0, 2062.0),
        Range::new(0.0, 25.2));

    let mut power_rail_12v0 = PowerRail::new(POWER_RAIL_12V0_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, true, _>::new(),
        Range::new(0.0, 1741.0),
        Range::new(0.0, 12.0));

    let mut power_rail_5v0 = PowerRail::new(POWER_RAIL_5V0_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, true, _>::new(),
        Range::new(0.0, 1745.0),
        Range::new(0.0, 5.0));

    let mut power_rail_3v3 = PowerRail::new(POWER_RAIL_3V3_PARAMETERS,
        WindowAvergingFilter::<POWER_RAIL_FILTER_WINDOW_SIZE, true, _>::new(),
        Range::new(0.0, 1750.0),
        Range::new(0.0, 3.3));

    ///////////////////////////
    //  Battery Model Setup  //
    ///////////////////////////

    let mut lipo6s_battery_model: LipoModel<'_, 6, WindowAvergingFilter<15, true, f32>> = 
            LipoModel::new(
                LIPO_BATTERY_CONFIG_6S,
                &LIPO6S_BALANCE_RAW_SAMPLES_TO_VOLTAGES,
                ateam_lib_stm32::power::battery::CellVoltageComputeMode::Chained);

    // Create empty telemetry packet
    let battery_telem_packet: BatteryInfoPacket = unsafe { MaybeUninit::zeroed().assume_init() };

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

        // let power_rail_adc_voltages = PowerRailAdcSamples::new_from_samples(&power_rail_adc_raw_samples).expect("invalid slice length on power rail adc sample conversion");
        adc_converter.update_vrefint(*power_rail_adc_raw_samples.last().unwrap_or(&0));
        adc_converter.raw_samples_to_mv(&power_rail_adc_raw_samples, &mut power_rail_voltage_samples);
        power_rail_3v3.add_rail_voltage_sample(power_rail_voltage_samples[0] as f32);
        power_rail_5v0.add_rail_voltage_sample(power_rail_voltage_samples[1] as f32);
        power_rail_12v0.add_rail_voltage_sample(power_rail_voltage_samples[2] as f32);
        power_rail_battery.add_rail_voltage_sample(power_rail_voltage_samples[3] as f32);

        // defmt::info!("power rail voltages: Battery {}, 12v0 {}, 5v0 {}, 3v3 {}", power_rail_battery.get_rail_voltage(), power_rail_12v0.get_rail_voltage(), power_rail_5v0.get_rail_voltage(), power_rail_3v3.get_rail_voltage());

        ///////////////////////////////
        //  Check Power Rail Errors  //
        ///////////////////////////////


        shared_power_state.set_power_rail_3v3_ok(power_rail_3v3.power_ok()).await;
        shared_power_state.set_power_rail_5v0_ok(power_rail_5v0.power_ok()).await;
        shared_power_state.set_power_rail_12v0_ok(power_rail_12v0.power_ok()).await;
        shared_power_state.set_power_ok(
            power_rail_3v3.power_ok() &&
            power_rail_5v0.power_ok() &&
            power_rail_12v0.power_ok() &&
            power_rail_battery.power_ok()
        ).await;


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

            last_battery_cell_read_time = Instant::now();

            adc_converter.update_vrefint(*battery_cell_adc_raw_samples.last().unwrap_or(&0));
            adc_converter.raw_samples_to_v(&battery_cell_adc_raw_samples, &mut battery_cell_voltage_samples);

            lipo6s_battery_model.add_cell_voltage_samples(&battery_cell_voltage_samples[0..6]);

            defmt::info!("battery cell voltages {}", lipo6s_battery_model.get_cell_voltages());
            defmt::info!("battery cell percentages {}", lipo6s_battery_model.get_cell_percentages());
            defmt::info!("battery worst cell imblanace {}", lipo6s_battery_model.get_worst_cell_imbalance());

            if lipo6s_battery_model.get_cell_percentages().into_iter().all(|v| *v <= 0) {
                defmt::info!("battery balance connector is unplugged");
                shared_power_state.set_balance_connected(false).await;
            } else {
                shared_power_state.set_balance_connected(true).await;
            }
            
            // Vbatt PMIC differential
            if f32::abs(battery_cell_voltage_samples[7] - power_rail_voltage_samples[3] as f32 / 1000.0) > 1.0 {
                defmt::error!("adc measuring substantial voltage drop across the load switch");
            }

            let mut high_current_ops_allowed = true;

            if lipo6s_battery_model.battery_warn() {
                shared_power_state.set_battery_low_warn(true).await;
            }

            if lipo6s_battery_model.battery_crit() {
                shared_power_state.set_battery_low_crit(true).await;
                high_current_ops_allowed = false;
            }

            if lipo6s_battery_model.battery_power_off() {
                shared_power_state.set_shutdown_requested(true).await;
                high_current_ops_allowed = false;
            }

            shared_power_state.set_high_current_operations_allowed(high_current_ops_allowed).await;
        }

        // send pubsub message to coms task
        telemetry_publisher.publish_immediate(battery_telem_packet);

        loop_ticker.next().await;
    }
}

pub async fn start_power_task(spawner: &Spawner, shared_power_state: &'static SharedPowerState, telemetry_publisher: TelemetryPublisher,
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
        shared_power_state,
        telemetry_publisher,
        adc, adc_dma,
        cell1_adc_pin, cell2_adc_pin, cell3_adc_pin, cell4_adc_pin, cell5_adc_pin, cell6_adc_pin,
        power_rail_12v0_adc_pin, power_rail_5v0_adc_pin, power_rail_3v3_adc_pin, power_rail_vbatt_before_lsw_adc_pin, power_rail_vbatt_adc_pin
    )).expect("failed to spawn power task");
}