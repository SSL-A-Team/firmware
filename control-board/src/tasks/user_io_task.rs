use ateam_lib_stm32::filter::{Filter, IirFilter};
use ateam_lib_stm32::time::SyncTicker;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pull, Speed};
use embassy_time::{Duration, Ticker};

use ateam_lib_stm32::drivers::switches::dip::DipSwitch;
use ateam_lib_stm32::drivers::switches::rotary_encoder::RotaryEncoder;
use ateam_lib_stm32::drivers::other::adc_helper::AdcHelper;
use embassy_stm32::adc::{Adc, SampleTime, Resolution};

use crate::drivers::shell_indicator::ShellIndicator;
use crate::robot_state::SharedRobotState;

use crate::{adc_v_to_battery_voltage, pins::*, BATTERY_MIN_CRIT_VOLTAGE, BATTERY_MIN_SAFE_VOLTAGE, BATTERY_MAX_VOLTAGE};

fn get_vref_int_cal() -> u16 {
    unsafe { *(0x1FF1_E860 as *const u16) }
}

const IO_TASK_LOOP_RATE_DURATION: Duration = Duration::from_millis(50);  // 20 Hz
const BATTERY_FILTER_WINDOW: Duration = Duration::from_secs(10);

#[macro_export]
macro_rules! create_io_task {
    ($spawner:ident, $robot_state:ident, $p:ident) => {
        ateam_control_board::tasks::user_io_task::start_io_task(&$spawner,
            $robot_state,
            $p.ADC1, $p.PA0,
            $p.ADC3,
            $p.PE9, $p.PE8, $p.PE7, $p.PF15, $p.PF14, $p.PF13, $p.PF12, $p.PF11,
            $p.PC13,
            $p.PB10, $p.PB11,
            $p.PD0, $p.PD1, $p.PG10, $p.PG11,
            $p.PG6, $p.PG5, $p.PG4, $p.PD13,
            $p.PD10,
            $p.PC3, $p.PC1, $p.PC0, $p.PC2, $p.PF10).await;
    };
}

#[embassy_executor::task]
async fn user_io_task_entry(
    robot_state: &'static SharedRobotState,
    mut battery_volt_adc: AdcHelper<'static, BatteryAdc, BatteryAdcPin>,
    vref_int_adc: Adc<'static, VrefIntAdc>,
    dip_switch: DipSwitch<'static, 8>,
    debug_mode_dip_switch: DipSwitch<'static, 1>,
    robot_color_dip_switch: DipSwitch<'static, 2>,
    robot_id_rotary: RotaryEncoder<'static, 4>,
    mut debug_led0: Output<'static>,
    mut debug_led1: Output<'static>,
    mut debug_led2: Output<'static>,
    mut debug_led3: Output<'static>,
    mut robot_id_src_disagree: Output<'static>,
    mut robot_id_indicator: ShellIndicator<'static>,
) {
    defmt::info!("user io task initialized");

    let mut io_task_loop_rate_ticker = Ticker::every(IO_TASK_LOOP_RATE_DURATION);

    let mut lifelight_high = true;
    let mut lifelight_ticker = SyncTicker::every(Duration::from_millis(100));
    debug_led0.set_high();

    ///////////////////////
    // Battery ADC Setup //
    ///////////////////////
    
    // Get the Vref_int calibration values.
    let vref_int_cal = get_vref_int_cal() as f32;
    let mut _vref_int_ch = vref_int_adc.enable_vrefint();

    // Create the running buffers for averaging
    const BATTERY_VOLTAGE_FILTER_ALPHA: f32 = IO_TASK_LOOP_RATE_DURATION.as_millis() as f32 / BATTERY_FILTER_WINDOW.as_millis() as f32;
    let mut battery_voltage_filter = IirFilter::new(BATTERY_VOLTAGE_FILTER_ALPHA);
    
    loop {
        let cur_robot_state = robot_state.get_state();

        // read switches
        let robot_network_index = dip_switch.read_block(7..4);
        let robot_radio_driver_use_flow_control = dip_switch.read_pin(3);
        let send_extended_telem = dip_switch.read_pin(0);

        let hw_debug_mode = debug_mode_dip_switch.read_pin(0);
        
        let robot_team_isblue = robot_color_dip_switch.read_pin(1);
        
        let robot_id = robot_id_rotary.read_value();

        if hw_debug_mode != cur_robot_state.hw_debug_mode {
            robot_state.set_hw_in_debug_mode(hw_debug_mode);
            if hw_debug_mode {
                defmt::info!("robot entered debug mode");
            }
        }

        if robot_radio_driver_use_flow_control != cur_robot_state.hw_wifi_driver_use_flow_control {
            robot_state.set_hw_wifi_driver_use_flow_control(robot_radio_driver_use_flow_control);
            defmt::info!("updated radio driver use flow control {} -> {}", cur_robot_state.hw_wifi_driver_use_flow_control, robot_radio_driver_use_flow_control);
        }

        // publish updates to robot_state
        if robot_id != cur_robot_state.hw_robot_id {
            robot_state.set_hw_robot_id(robot_id);
            defmt::info!("updated robot id {} -> {}", cur_robot_state.hw_robot_id, robot_id);
        }

        if robot_team_isblue != cur_robot_state.hw_robot_team_is_blue {
            robot_state.set_hw_robot_team_is_blue(robot_team_isblue);
            defmt::info!("updated robot team is blue {} -> {}", cur_robot_state.hw_robot_team_is_blue, robot_team_isblue);
        }

        if robot_network_index != cur_robot_state.hw_wifi_network_index as u8 {
            robot_state.set_hw_network_index(robot_network_index);
            defmt::info!("updated robot network index {} -> {}", cur_robot_state.hw_wifi_network_index, robot_network_index);
        }

        if send_extended_telem != cur_robot_state.radio_send_extended_telem {
            robot_state.set_radio_send_extended_telem(send_extended_telem);
            defmt::info!("updated robot send extended telem {} -> {}", cur_robot_state.radio_send_extended_telem, send_extended_telem);
        }
    
        ///////////////////////
        //  Battery reading  //
        ///////////////////////

        // FIXME: Vref_int is not returning valid value. Embassy issue. 
        // let vref_int_read_mv = vref_int_adc.read(&mut vref_int_ch);
        let vref_int_read_mv = 1216.0;
        let batt_res_div_v = battery_volt_adc.read_volt_raw_f32(vref_int_read_mv as f32, vref_int_cal) * 0.9090;
        let cur_battery_voltage = adc_v_to_battery_voltage(batt_res_div_v);
        
        // Add new battery read to cyclical buffer.
        battery_voltage_filter.add_sample(cur_battery_voltage);        
        let battery_voltage_ave = battery_voltage_filter.filtered_value().expect("iir never return None here");

        // TODO do something with local voltage?
        // battery_volt_publisher.publish_immediate(battery_voltage_ave);
        robot_state.set_battery_low(battery_voltage_ave < BATTERY_MIN_SAFE_VOLTAGE);
        robot_state.set_battery_crit(battery_voltage_ave < BATTERY_MIN_CRIT_VOLTAGE || battery_voltage_ave > BATTERY_MAX_VOLTAGE);

        // update indicators
        robot_id_indicator.set(robot_id, robot_team_isblue);
        robot_id_src_disagree.set_low();

        if !robot_state.hw_init_state_valid() {
            defmt::info!("loaded robot state: robot id: {}, team: {}", robot_id, robot_team_isblue);
            robot_state.set_hw_init_state_valid(true);
        }

        // if hardware init is done, flash light 0
        if robot_state.hw_init_state_valid() {
            if lifelight_ticker.next() {
                if lifelight_high {
                    debug_led0.set_low();
                    lifelight_high = false;
                } else {
                    debug_led0.set_high();
                    lifelight_high = true;
                }
            }
        } else {
            debug_led0.set_high();
        }

        if send_extended_telem {
            debug_led1.set_high();
        } else {
            debug_led1.set_low();
        }

        if robot_radio_driver_use_flow_control {
            debug_led2.set_high();
        } else {
            debug_led2.set_low();
        }

        if hw_debug_mode {
            debug_led3.set_high();
        } else {
            debug_led3.set_low();
        }

        io_task_loop_rate_ticker.next().await;
    }
}

pub async fn start_io_task(spawner: &Spawner,
    robot_state: &'static SharedRobotState,
    battery_adc_peri: BatteryAdc, battery_adc_pin: BatteryAdcPin,
    vref_int_adc_peri: VrefIntAdc,
    usr_dip7_pin: UsrDip7Pin, usr_dip6_pin: UsrDip6Pin, usr_dip5_pin: UsrDip5Pin, usr_dip4_pin: UsrDip4Pin,
    usr_dip3_pin: UsrDip3Pin, usr_dip2_pin: UsrDip2Pin, usr_dip1_pin: UsrDip1Pin, usr_dip0_pin: UsrDip0Pin,
    debug_mode_pin: UsrDipDebugMode,
    robot_id_team_is_blue_pin: UsrDipTeamIsBluePin, robot_id_src_select_pin: UsrDipBotIdSrcSelect,
    robot_id_selector3_pin: RobotIdSelector3Pin, robot_id_selector2_pin: RobotIdSelector2Pin,
    robot_id_selector1_pin: RobotIdSelector1Pin, robot_id_selector0_pin: RobotIdSelector0Pin,
    usr_led0_pin: UsrLed0Pin, usr_led1_pin: UsrLed1Pin, usr_led2_pin: UsrLed2Pin, usr_led3_pin: UsrLed3Pin,
    robot_id_src_disagree_led_pin: RobotIdSrcDisagree,
    robot_id_indicator_fl: RobotIdIndicator0FlPin, robot_id_indicator_bl: RobotIdIndicator1BlPin,
    robot_id_indicator_br: RobotIdIndicator2BrPin, robot_id_indicator_fr: RobotIdIndicator3FrPin,
    robot_id_indicator_isblue: RobotIdIndicator4TeamIsBluePin,
    ) {

    let dip_sw_pins: [AnyPin; 8] = [usr_dip7_pin.into(), usr_dip6_pin.into(), usr_dip5_pin.into(), usr_dip4_pin.into(), usr_dip3_pin.into(), usr_dip2_pin.into(), usr_dip1_pin.into(), usr_dip0_pin.into()];
    let dip_switch = DipSwitch::new_from_pins(dip_sw_pins, Pull::None, None);

    let debug_mode_pins: [AnyPin; 1] = [debug_mode_pin.into()];
    let debug_mode_dip = DipSwitch::new_from_pins(debug_mode_pins, Pull::None, Some([true]));
    
    let team_color_pins: [AnyPin; 2] = [robot_id_src_select_pin.into(), robot_id_team_is_blue_pin.into()];
    let team_color_dip = DipSwitch::new_from_pins(team_color_pins, Pull::None, Some([false, true]));

    let robot_id_selector_pins: [AnyPin; 4] = [robot_id_selector3_pin.into(), robot_id_selector2_pin.into(), robot_id_selector1_pin.into(), robot_id_selector0_pin.into()];
    let robot_id_rotary = RotaryEncoder::new_from_pins(robot_id_selector_pins, Pull::None, None);

    let debug_led0 = Output::new(usr_led0_pin, Level::Low, Speed::Low);
    let debug_led1 = Output::new(usr_led1_pin, Level::Low, Speed::Low);
    let debug_led2 = Output::new(usr_led2_pin, Level::Low, Speed::Low);
    let debug_led3 = Output::new(usr_led3_pin, Level::Low, Speed::Low);

    let robot_id_src_disagree_led = Output::new(robot_id_src_disagree_led_pin, Level::Low, Speed::Low);
    let robot_id_indicator = ShellIndicator::new(robot_id_indicator_fr, robot_id_indicator_fl, robot_id_indicator_bl, robot_id_indicator_br, Some(robot_id_indicator_isblue));

    let battery_volt_adc = AdcHelper::new(battery_adc_peri, battery_adc_pin, SampleTime::CYCLES32_5, Resolution::BITS12);
    let mut vref_int_adc = Adc::new(vref_int_adc_peri);
    // Set the Vref_int ADC settings to the same as the battery.
    vref_int_adc.set_resolution(Resolution::BITS12);
    vref_int_adc.set_sample_time(SampleTime::CYCLES32_5);

    spawner.spawn(user_io_task_entry(
        robot_state,
        battery_volt_adc, vref_int_adc,
        dip_switch, debug_mode_dip, team_color_dip, robot_id_rotary,
        debug_led0, debug_led1, debug_led2, debug_led3,
        robot_id_src_disagree_led, robot_id_indicator)).unwrap();
}