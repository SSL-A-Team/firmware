use ateam_lib_stm32::drivers::switches::dip::DipSwitch;
use ateam_lib_stm32::drivers::switches::rotary_encoder::RotaryEncoder;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pull, Speed};
use embassy_time::Timer;

use crate::drivers::shell_indicator::ShellIndicator;
use crate::robot_state::SharedRobotState;

use crate::pins::*;

#[macro_export]
macro_rules! create_io_task {
    ($spawner:ident, $robot_state:ident, $p:ident) => {
        ateam_control_board::tasks::user_io_task::start_io_task($spawner,
            $robot_state,
            $p.PD6, $p.PD5, $p.EXTI6, $p.EXTI5,
            $p.PG7, $p.PG6, $p.PG5, $p.PG4, $p.PG3, $p.PG2, $p.PD15,
            $p.PG12, $p.PG11, $p.PG10, $p.PG9,
            $p.PF3, $p.PF2, $p.PF1, $p.PF0,
            $p.PD0, $p.PD1, $p.PD3, $p.PD4, $p.PD14);
    };
}

#[embassy_executor::task]
async fn user_io_task_entry(robot_state: &'static SharedRobotState,
    dip_switch: DipSwitch<'static, 7>,
    robot_id_rotary: RotaryEncoder<'static, 4>,
    mut debug_led0: Output<'static>,
    mut robot_id_indicator: ShellIndicator<'static>,
) {
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

        // TODO read messages

        // update indicators
        robot_id_indicator.set(robot_id, robot_team_isblue);

        if hw_debug_mode {
            debug_led0.set_high();
        } else {
            debug_led0.set_low();
        }

        if !robot_state.hw_init_state_valid() {
            defmt::info!("loaded robot state: robot id: {}, team: {}", robot_id, robot_team_isblue);
            robot_state.set_hw_init_state_valid(true);
        }

        Timer::after_millis(100).await;
    }
}

pub fn start_io_task(spawner: Spawner,
    robot_state: &'static SharedRobotState,
    _usr_btn0_pin: UsrBtn0Pin, _usr_btn1_pin: UsrBtn1Pin, _usr_btn0_exti: UsrBtn0Exti, _usr_btn1_exti: UsrBtn1Exti,
    usr_dip7_pin: UsrDip7IsBluePin, usr_dip6_pin: UsrDip6Pin, usr_dip5_pin: UsrDip5Pin, usr_dip4_pin: UsrDip4Pin,
    usr_dip3_pin: UsrDip3Pin, usr_dip2_pin: UsrDip2Pin, usr_dip1_pin: UsrDip1Pin,
    robot_id_selector3_pin: RobotIdSelector3Pin, robot_id_selector2_pin: RobotIdSelector2Pin,
    robot_id_selector1_pin: RobotIdSelector1Pin, robot_id_selector0_pin: RobotIdSelector0Pin,
    usr_led0_pin: UsrLed0Pin, _usr_led1_pin: UsrLed1Pin, _usr_led2_pin: UsrLed2Pin, _usr_led3_pin: UsrLed3Pin,
    robot_id_indicator_fl: RobotIdIndicator0FlPin, robot_id_indicator_bl: RobotIdIndicator1BlPin,
    robot_id_indicator_br: RobotIdIndicator2BrPin, robot_id_indicator_fr: RobotIdIndicator3FrPin,
    robot_id_indicator_isblue: RobotIdIndicator4TeamIsBluePin,
    ) {

    let dip_sw_pins: [AnyPin; 7] = [usr_dip7_pin.into(), usr_dip6_pin.into(), usr_dip5_pin.into(), usr_dip4_pin.into(), usr_dip3_pin.into(), usr_dip2_pin.into(), usr_dip1_pin.into()];
    let dip_switch = DipSwitch::new_from_pins(dip_sw_pins, Pull::None, None);

    let robot_id_selector_pins: [AnyPin; 4] = [robot_id_selector3_pin.into(), robot_id_selector2_pin.into(), robot_id_selector1_pin.into(), robot_id_selector0_pin.into()];
    let robot_id_rotary = RotaryEncoder::new_from_pins(robot_id_selector_pins, Pull::None, None);

    let debug_led0 = Output::new(usr_led0_pin, Level::Low, Speed::Low);

    let robot_id_indicator = ShellIndicator::new(robot_id_indicator_fr, robot_id_indicator_fl, robot_id_indicator_br, robot_id_indicator_bl, Some(robot_id_indicator_isblue));

    spawner.spawn(user_io_task_entry(robot_state, dip_switch, robot_id_rotary, debug_led0, robot_id_indicator)).unwrap();
}