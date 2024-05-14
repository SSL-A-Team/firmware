use ateam_lib_stm32::drivers::switches::dip::DipSwitch;
use ateam_lib_stm32::drivers::switches::rotary_encoder::RotaryEncoder;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Pull};

use crate::drivers::shell_indicator::ShellIndicator;
use crate::robot_state::RobotState;

use crate::pins::*;


#[embassy_executor::task]
async fn user_io_task_entry(robot_state: &'static RobotState,
    dip_switch: DipSwitch<'static, 7>,
    robot_id_rotary: RotaryEncoder<'static, 4>,
    robot_id_indicator: ShellIndicator<'static>,
) {
    loop {
        // TODO read switches

        // TODO publish updates to robot_state

        // TODO read messages

        // TODO update indicators
    }
}

pub fn start_io_task(spawner: Spawner,
    robot_state: &'static RobotState,
    usr_btn0_pin: UsrBtn0Pin, usr_btn1_pin: UsrBtn1Pin, usr_btn0_exti: UsrBtn0Exti, usr_btn1_exti: UsrBtn1Exti,
    usr_dip7_pin: UsrDip7IsBluePin, usr_dip6_pin: UsrDip6Pin, usr_dip5_pin: UsrDip5Pin, usr_dip4_pin: UsrDip4Pin,
    usr_dip3_pin: UsrDip3Pin, usr_dip2_pin: UsrDip2Pin, usr_dip1_pin: UsrDip1Pin,
    robot_id_selector3_pin: RobotIdSelector3Pin, robot_id_selector2_pin: RobotIdSelector2Pin,
    robot_id_selector1_pin: RobotIdSelector1Pin, robot_id_selector0_pin: RobotIdSelector0Pin,
    usr_led0_pin: UsrLed0Pin, usr_led1_pin: UsrLed1Pin, usr_led2_pin: UsrLed2Pin, usr_led3_pin: UsrLed3Pin,
    robot_id_indicator_fl: RobotIdIndicator0FlPin, robot_id_indicator_bl: RobotIdIndicator1BlPin,
    robot_id_indicator_br: RobotIdIndicator2BrPin, robot_id_indicator_fr: RobotIdIndicator3FrPin,
    robot_id_indicator_isblue: RobotIdIndicator4TeamIsBluePin,
    ) {

    let dip_sw_pins: [AnyPin; 7] = [usr_dip7_pin.into(), usr_dip6_pin.into(), usr_dip5_pin.into(), usr_dip4_pin.into(), usr_dip3_pin.into(), usr_dip2_pin.into(), usr_dip1_pin.into()];
    let dip_switch = DipSwitch::new_from_pins(dip_sw_pins, Pull::None, None);

    let robot_id_selector_pins: [AnyPin; 4] = [robot_id_selector3_pin.into(), robot_id_selector2_pin.into(), robot_id_selector1_pin.into(), robot_id_selector0_pin.into()];
    let robot_id_rotary = RotaryEncoder::new_from_pins(robot_id_selector_pins, Pull::None, None);

    let robot_id_indicator = ShellIndicator::new(robot_id_indicator_fr, robot_id_indicator_fl, robot_id_indicator_br, robot_id_indicator_bl, Some(robot_id_indicator_isblue));

    spawner.spawn(user_io_task_entry(robot_state, dip_switch, robot_id_rotary, robot_id_indicator)).unwrap();
}