
use ateam_lib_stm32::anim::{self, AnimInterface, AnimRepeatMode, Blink, CompositeAnimation, Lerp};
use ateam_lib_stm32::drivers::led::apa102::{Apa102, Apa102Anim};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pull, Speed};
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::time::mhz;
use embassy_time::{Duration, Timer};

use smart_leds::colors::{BLACK, WHITE};
use smart_leds::RGB8;
use static_cell::ConstStaticCell;

// use ateam_lib_stm32::drivers::led::apa102::{Apa102, Apa102AnimationRepeat, Apa102AnimationTrait, Apa102Blink};
use ateam_lib_stm32::drivers::switches::button::AdvExtiButton;
use ateam_lib_stm32::drivers::switches::dip::DipSwitch;
use ateam_lib_stm32::drivers::switches::rotary_encoder::RotaryEncoder;
use ateam_lib_stm32::drivers::other::adc_helper::AdcHelper;
use embassy_stm32::adc::{Adc, SampleTime, Resolution};

use crate::drivers::shell_indicator::ShellIndicator;
use crate::robot_state::SharedRobotState;

use crate::{adc_v_to_battery_voltage, pins::*, stm32_interface, BATTERY_MIN_CRIT_VOLTAGE, BATTERY_MIN_SAFE_VOLTAGE, BATTERY_MAX_VOLTAGE, BATTERY_BUFFER_SIZE};
use crate::tasks::shutdown_task::HARD_SHUTDOWN_TIME_MS;

// #[link_section = ".sram4"]
// static DOTSTAR_SPI_BUFFER_CELL: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);

fn get_vref_int_cal() -> u16 {
    unsafe { *(0x1FF1_E860 as *const u16) }
}

#[link_section = ".sram4"]
static mut DOTSTAR_SPI_BUFFER_CELL: [u8; 16] = [0; 16];



#[macro_export]
macro_rules! create_io_task {
    ($spawner:ident, $robot_state:ident, $battery_volt_publisher:ident, $p:ident) => {
        ateam_control_board::tasks::user_io_task::start_io_task(&$spawner,
            $robot_state,
            $battery_volt_publisher,
            $p.ADC2, $p.PF14,
            $p.ADC3,
            $p.PD6, $p.PD5, $p.EXTI6, $p.EXTI5,
            $p.PG7, $p.PG6, $p.PG5, $p.PG4, $p.PG3, $p.PG2, $p.PD15,
            $p.PG12, $p.PG11, $p.PG10, $p.PG9,
            $p.PF3, $p.PF2, $p.PF1, $p.PF0,
            $p.PD0, $p.PD1, $p.PD3, $p.PD4, $p.PD14,
            $p.SPI6, $p.PB3, $p.PB5, $p.BDMA_CH0).await;
    };
}

#[embassy_executor::task]
async fn user_io_task_entry(
    robot_state: &'static SharedRobotState,
    mut _usr_btn0: AdvExtiButton,
    mut _usr_btn1: AdvExtiButton,
    battery_volt_publisher: BatteryVoltPublisher,
    mut battery_volt_adc: AdcHelper<'static, BatteryAdc, BatteryAdcPin>,
    vref_int_adc: Adc<'static, VrefIntAdc>,
    dip_switch: DipSwitch<'static, 7>,
    robot_id_rotary: RotaryEncoder<'static, 4>,
    mut debug_led0: Output<'static>,
    mut robot_id_indicator: ShellIndicator<'static>,
    mut dotstars: Apa102<'static, 'static, 2>,
) {
    defmt::info!("user io task initialized");

    // let shutdown_anim = anim::Animation::Lerp(Lerp::new(WHITE, BLACK, Duration::from_millis(HARD_SHUTDOWN_TIME_MS), AnimRepeatMode::None));
    // let mut sd_anim_seq = [shutdown_anim];
    // let mut shutdown_anim = CompositeAnimation::new(&mut sd_anim_seq, AnimRepeatMode::None);

    const SHUTDOWN_ANIM_ID: usize = 2;
    let shutdown_dimmer = anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 255, g: 255, b: 255}, RGB8 { r: 0, g: 0, b:0 }, Duration::from_millis(10000), AnimRepeatMode::None));
    let mut shutdown_dimmer_arr = [shutdown_dimmer];
    let shutdown_ca = CompositeAnimation::new(SHUTDOWN_ANIM_ID, &mut shutdown_dimmer_arr, AnimRepeatMode::None);

    let mut led0_anim_playbook = [shutdown_ca];

    const RGB_LERP_ID: usize = 1;
    let anim0 = anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 255, g: 0, b: 0 }, RGB8 { r: 0, g: 255, b: 0 }, Duration::from_millis(1000), AnimRepeatMode::None));
    let anim1 = anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 0, g: 255, b: 0 }, RGB8 { r: 0, g: 0, b: 255 }, Duration::from_millis(1000), AnimRepeatMode::None));
    let anim2 = anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 0, g: 0, b: 255 }, RGB8 { r: 255, g: 0, b: 0 }, Duration::from_millis(1000), AnimRepeatMode::None));
    let mut anim_seq = [anim0, anim1, anim2];
    let composite_anim0 = CompositeAnimation::new(RGB_LERP_ID, &mut anim_seq, AnimRepeatMode::Forever);
    // let mut composite_anim0 = CompositeAnimation::new(RGB_LERP_ID, &mut [anim0, anim1, anim2], AnimRepeatMode::Forever);

    let mut led1_anim_playbook = [composite_anim0];

    // composite_anim0.start_animation();

    // let animation_playbooks: [Option<&mut [CompositeAnimation<u8, RGB8>]>; 2] = [
    //     Some(&mut [
    //         CompositeAnimation::new(RGB_LERP_ID, &mut [
    //             anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 255, g: 0, b: 0 }, RGB8 { r: 0, g: 255, b: 0 }, Duration::from_millis(1000), AnimRepeatMode::None)),
    //             anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 0, g: 255, b: 0 }, RGB8 { r: 0, g: 0, b: 255 }, Duration::from_millis(1000), AnimRepeatMode::None)),
    //             anim::Animation::Lerp(Lerp::new(0, RGB8 { r: 0, g: 0, b: 255 }, RGB8 { r: 255, g: 0, b: 0 }, Duration::from_millis(1000), AnimRepeatMode::None)),     
    //         ], AnimRepeatMode::Forever)
    //     ]),
    //     None,
    // ];

    let animation_playbooks: [Option<&mut [CompositeAnimation<u8, RGB8>]>; 2] = [
        Some(&mut led1_anim_playbook),
        Some(&mut led0_anim_playbook),
    ];

    // let anim0_1 = anim::Animation::Lerp(Lerp::new(RGB8 { r: 255, g: 255, b: 0 }, RGB8 { r: 0, g: 255, b: 255 }, Duration::from_millis(1000), AnimRepeatMode::None));
    // let anim1_1 = anim::Animation::Lerp(Lerp::new(RGB8 { r: 0, g: 255, b: 255 }, RGB8 { r: 255, g: 0, b: 255 }, Duration::from_millis(1000), AnimRepeatMode::None));
    // let anim2_1 = anim::Animation::Lerp(Lerp::new(RGB8 { r: 255, g: 0, b: 255 }, RGB8 { r: 255, g: 255, b: 0 }, Duration::from_millis(1000), AnimRepeatMode::None));
    // let mut anim_seq = [anim0_1, anim1_1, anim2_1];
    // let mut composite_anim1 = CompositeAnimation::new(&mut anim_seq, AnimRepeatMode::Forever);
    // composite_anim1.start_animation();

    dotstars.set_drv_str_all(32);
    let mut dotstars_anim = Apa102Anim::new(dotstars, animation_playbooks);

    dotstars_anim.enable_animation(0, RGB_LERP_ID);
    // dotstars_anim.enable_animation(1, RGB_LERP_ID);

    // let mut color_lerp = TimeLerp::new(RGB8 { r: 255, g: 0, b: 0 }, RGB8 { r: 0, g: 0, b: 255 }, Duration::from_millis(10000));
    // color_lerp.start();

    // Battery ADC Setup
    // Get the Vref_int calibration values.
    let vref_int_cal = get_vref_int_cal() as f32;
    let mut vref_int_ch = vref_int_adc.enable_vrefint();

    // let mut anim1 = Lerp::new(RGB8 { r: 0, g: 0, b: 0}, RGB8 { r: 255, g: 255, b: 255 }, Duration::from_millis(1000), AnimRepeatMode::Forever);
    // anim1.start_animation();
    // dotstars_anim.set_animation(anim1, 1);

    // let mut blink_anim = Blink::new(RGB8 { r: 255, g: 0, b: 0 }, RGB8 { r: 0, g: 0, b: 255 }, Duration::from_millis(800), Duration::from_millis(200), AnimRepeatMode::Forever);
    // blink_anim.start_animation();
    // dotstars.set_animation(blink_anim, 1);

    let mut battery_voltage_buffer: [f32; BATTERY_BUFFER_SIZE] =
        [BATTERY_MAX_VOLTAGE; BATTERY_BUFFER_SIZE];
    
    let mut battery_voltage_filt_indx = 0;

    let mut prev_robot_state = robot_state.get_state();
    loop {
        let cur_robot_state = robot_state.get_state();

        // read switches
        let robot_id = robot_id_rotary.read_value();

        let robot_team_isblue = dip_switch.read_pin(0);
        let hw_debug_mode = dip_switch.read_pin(1);
        let robot_network_index = dip_switch.read_block(6..4);

        if hw_debug_mode != cur_robot_state.hw_debug_mode {
            robot_state.set_hw_in_debug_mode(hw_debug_mode);
            if hw_debug_mode {
                defmt::info!("robot entered debug mode");
            }
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
    
        ///////////////////////
        //  Battery reading  //
        ///////////////////////

        // FIXME: Vref_int is not returning valid value. Embassy issue. 
        // let vref_int_read_mv = vref_int_adc.read(&mut vref_int_ch);
        let vref_int_read_mv = 1216.0;
        let batt_res_div_v = battery_volt_adc.read_volt_raw_f32(vref_int_read_mv as f32, vref_int_cal);
        let battery_voltage_cur = adc_v_to_battery_voltage(batt_res_div_v);
        
        // Add new battery read to cyclical buffer.
        battery_voltage_buffer[battery_voltage_filt_indx] = battery_voltage_cur;

        // Shift index for next run.
        if battery_voltage_filt_indx == (BATTERY_BUFFER_SIZE - 1) {
            battery_voltage_filt_indx = 0;
        } else {
            battery_voltage_filt_indx += 1;
        }
        
        let battery_voltage_sum: f32 = battery_voltage_buffer.iter().sum();
        // Calculate battery average
        let battery_voltage_ave = battery_voltage_sum / (BATTERY_BUFFER_SIZE as f32);
                
        battery_volt_publisher.publish_immediate(battery_voltage_ave);
        robot_state.set_battery_low(battery_voltage_ave < BATTERY_MIN_SAFE_VOLTAGE);
        robot_state.set_battery_crit(battery_voltage_ave < BATTERY_MIN_CRIT_VOLTAGE || battery_voltage_ave > BATTERY_MAX_VOLTAGE);

        // TODO read messages

        // update indicators
        robot_id_indicator.set(robot_id, robot_team_isblue);

        if hw_debug_mode {
            debug_led0.set_high();
        } else {
            debug_led0.set_low();
        }

        if !prev_robot_state.shutdown_requested && cur_robot_state.shutdown_requested {
            dotstars_anim.disable_animation(0, RGB_LERP_ID);
            dotstars_anim.enable_animation(1, SHUTDOWN_ANIM_ID);
        } 

        // let color = color_lerp.update();
        // dotstars.set_color(color, 0);
        // dotstars.set_color(RGB8 { r: 0, g: 0, b: 0 }, 1);
        dotstars_anim.update().await;


        if !robot_state.hw_init_state_valid() {
            defmt::info!("loaded robot state: robot id: {}, team: {}", robot_id, robot_team_isblue);
            robot_state.set_hw_init_state_valid(true);
        }

        prev_robot_state = cur_robot_state;

        Timer::after_millis(50).await;
    }
}

pub async fn start_io_task(spawner: &Spawner,
    robot_state: &'static SharedRobotState,
    battery_volt_publisher: BatteryVoltPublisher,
    battery_adc_peri: BatteryAdc, battery_adc_pin: BatteryAdcPin,
    vref_int_adc_peri: VrefIntAdc,
    usr_btn0_pin: UsrBtn0Pin, usr_btn1_pin: UsrBtn1Pin, usr_btn0_exti: UsrBtn0Exti, usr_btn1_exti: UsrBtn1Exti,
    usr_dip7_pin: UsrDip7IsBluePin, usr_dip6_pin: UsrDip6Pin, usr_dip5_pin: UsrDip5Pin, usr_dip4_pin: UsrDip4Pin,
    usr_dip3_pin: UsrDip3Pin, usr_dip2_pin: UsrDip2Pin, usr_dip1_pin: UsrDip1Pin,
    robot_id_selector3_pin: RobotIdSelector3Pin, robot_id_selector2_pin: RobotIdSelector2Pin,
    robot_id_selector1_pin: RobotIdSelector1Pin, robot_id_selector0_pin: RobotIdSelector0Pin,
    usr_led0_pin: UsrLed0Pin, _usr_led1_pin: UsrLed1Pin, _usr_led2_pin: UsrLed2Pin, _usr_led3_pin: UsrLed3Pin,
    robot_id_indicator_fl: RobotIdIndicator0FlPin, robot_id_indicator_bl: RobotIdIndicator1BlPin,
    robot_id_indicator_br: RobotIdIndicator2BrPin, robot_id_indicator_fr: RobotIdIndicator3FrPin,
    robot_id_indicator_isblue: RobotIdIndicator4TeamIsBluePin,
    dotstar_peri: DotstarSpi,
    dotstar_sck_pin: DotstarSpiSck,
    dotstar_mosi_pin: DotstarSpiMosi,
    dotstar_tx_dma: DotstarTxDma,
    ) {

    // defmt::info!("taking buf");
    // let dotstar_spi_buf: &'static mut [u8; 16] = DOTSTAR_SPI_BUFFER_CELL.take();
    // defmt::info!("took buf");

    let dotstar_spi_buf: &'static mut [u8; 16] = unsafe { &mut DOTSTAR_SPI_BUFFER_CELL };
    let dotstars = Apa102::<2>::new_from_pins(dotstar_peri, dotstar_sck_pin, dotstar_mosi_pin, dotstar_tx_dma, dotstar_spi_buf.into());

    let adv_usr_btn0: AdvExtiButton = AdvExtiButton::new_from_pins(usr_btn0_pin, usr_btn0_exti, false);
    let adv_usr_btn1: AdvExtiButton = AdvExtiButton::new_from_pins(usr_btn1_pin, usr_btn1_exti, false);

    let dip_sw_pins: [AnyPin; 7] = [usr_dip7_pin.into(), usr_dip6_pin.into(), usr_dip5_pin.into(), usr_dip4_pin.into(), usr_dip3_pin.into(), usr_dip2_pin.into(), usr_dip1_pin.into()];
    let dip_switch = DipSwitch::new_from_pins(dip_sw_pins, Pull::None, None);

    let robot_id_selector_pins: [AnyPin; 4] = [robot_id_selector3_pin.into(), robot_id_selector2_pin.into(), robot_id_selector1_pin.into(), robot_id_selector0_pin.into()];
    let robot_id_rotary = RotaryEncoder::new_from_pins(robot_id_selector_pins, Pull::None, None);

    let debug_led0 = Output::new(usr_led0_pin, Level::Low, Speed::Low);

    let robot_id_indicator = ShellIndicator::new(robot_id_indicator_fr, robot_id_indicator_fl, robot_id_indicator_bl, robot_id_indicator_br, Some(robot_id_indicator_isblue));

    let battery_volt_adc = AdcHelper::new(battery_adc_peri, battery_adc_pin, SampleTime::CYCLES32_5, Resolution::BITS12);
    let mut vref_int_adc = Adc::new(vref_int_adc_peri);
    // Set the Vref_int ADC settings to the same as the battery.
    vref_int_adc.set_resolution(Resolution::BITS12);
    vref_int_adc.set_sample_time(SampleTime::CYCLES32_5);

    spawner.spawn(user_io_task_entry(robot_state, adv_usr_btn0, adv_usr_btn1, battery_volt_publisher, battery_volt_adc, vref_int_adc, dip_switch, robot_id_rotary, debug_led0, robot_id_indicator, dotstars)).unwrap();
}