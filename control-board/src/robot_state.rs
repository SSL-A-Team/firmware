use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

pub struct SharedRobotState {
    hw_init_state_valid: AtomicBool,

    hw_robot_id: AtomicU8,
    hw_robot_team_is_blue: AtomicBool,
    hw_wifi_network_index: AtomicU8,
    hw_wifi_driver_use_flow_control: AtomicBool,
    hw_debug_mode: AtomicBool,

    radio_inop: AtomicBool,
    imu_inop: AtomicBool,
    kicker_inop: AtomicBool,
    power_inop: AtomicBool,
    wheels_inop: AtomicU8,
    dribbler_inop: AtomicBool,

    // systick is 1us which in a u64 gives an uptime of ~500000 yrs,
    // 32bit Cortex M has no AtomicU64, so we'll sync on milliseconds
    // overflow occurs in ~49.7 days
    last_packet_receive_time_ticks_ms: AtomicU32,
    radio_network_ok: AtomicBool,
    radio_bridge_ok: AtomicBool,
    radio_send_extended_telem: AtomicBool,


    battery_low: AtomicBool,
    battery_crit: AtomicBool,

    robot_tipped: AtomicBool,

    shutdown_requested: AtomicBool,

    ball_detected: AtomicBool,
    kicker_shutdown_complete: AtomicBool,
}

impl SharedRobotState {
    pub const fn new() -> SharedRobotState {
        SharedRobotState {
            hw_init_state_valid: AtomicBool::new(false),
            hw_robot_id: AtomicU8::new(0),
            hw_robot_team_is_blue: AtomicBool::new(false),
            hw_wifi_network_index: AtomicU8::new(0),
            hw_wifi_driver_use_flow_control: AtomicBool::new(true),
            hw_debug_mode: AtomicBool::new(true),
            radio_inop: AtomicBool::new(true),
            imu_inop: AtomicBool::new(true),
            kicker_inop: AtomicBool::new(true),
            power_inop: AtomicBool::new(true),
            wheels_inop: AtomicU8::new(0x0F), 
            dribbler_inop: AtomicBool::new(true),
            last_packet_receive_time_ticks_ms: AtomicU32::new(0),
            radio_network_ok: AtomicBool::new(false), 
            radio_bridge_ok: AtomicBool::new(false),
            radio_send_extended_telem: AtomicBool::new(false),
            battery_low: AtomicBool::new(false),
            battery_crit: AtomicBool::new(false),
            robot_tipped: AtomicBool::new(false),
            shutdown_requested: AtomicBool::new(false),
            ball_detected: AtomicBool::new(false),
            kicker_shutdown_complete: AtomicBool::new(false),
        }
    }

    pub fn get_state(&self) -> RobotState {
        RobotState {
            hw_init_state_valid: self.hw_init_state_valid(),
            hw_robot_id: self.get_hw_robot_id(),
            hw_robot_team_is_blue: self.hw_robot_team_is_blue(),
            hw_wifi_network_index: self.hw_wifi_network_index(),
            hw_wifi_driver_use_flow_control: self.hw_wifi_driver_use_flow_control(),
            hw_debug_mode: self.hw_in_debug_mode(),

            radio_inop: self.get_radio_inop(),
            imu_inop: self.get_imu_inop(),
            kicker_inop: self.get_kicker_inop(),
            power_inop: self.get_power_inop(),
            wheels_inop: self.get_wheels_inop(),
            dribbler_inop: self.get_dribbler_inop(),
        
            last_packet_receive_time_ticks_ms: 0,
            radio_network_ok: self.get_radio_network_ok(),
            radio_bridge_ok: self.get_radio_bridge_ok(),
            radio_send_extended_telem: self.get_radio_send_extended_telem(),

            battery_low: self.get_battery_low(),
            battery_crit: self.get_battery_crit(),

            robot_tipped: self.robot_tipped(),

            ball_detected: self.ball_detected(),
            shutdown_requested: self.shutdown_requested(),
            kicker_shutdown_complete: self.kicker_shutdown_complete(),
        }
    }

    pub fn hw_init_state_valid(&self) -> bool {
        self.hw_init_state_valid.load(Ordering::Relaxed)
    }

    pub fn set_hw_init_state_valid(&self, hw_init_state_valid: bool) {
        self.hw_init_state_valid.store(hw_init_state_valid, Ordering::Relaxed);
    }

    pub fn get_hw_robot_id(&self) -> u8 {
        self.hw_robot_id.load(Ordering::Relaxed)
    }

    pub fn set_hw_robot_id(&self, new_hw_robot_id: u8) {
        self.hw_robot_id.store(new_hw_robot_id, Ordering::Relaxed);
    }

    pub fn hw_robot_team_is_blue(&self) -> bool {
        self.hw_robot_team_is_blue.load(Ordering::Relaxed)
    }

    pub fn set_hw_robot_team_is_blue(&self, is_blue: bool) {
        self.hw_robot_team_is_blue.store(is_blue, Ordering::Relaxed);
    }

    pub fn hw_wifi_network_index(&self) -> usize {
        self.hw_wifi_network_index.load(Ordering::Relaxed) as usize
    }

    pub fn set_hw_network_index(&self, ind: u8) {
        self.hw_wifi_network_index.store(ind, Ordering::Relaxed);
    }

    pub fn hw_wifi_driver_use_flow_control(&self) -> bool {
        self.hw_wifi_driver_use_flow_control.load(Ordering::SeqCst)
    }

    pub fn set_hw_wifi_driver_use_flow_control(&self, use_flow_control: bool) {
        self.hw_wifi_driver_use_flow_control.store(use_flow_control, Ordering::SeqCst);
    }
 
    pub fn hw_in_debug_mode(&self) -> bool {
        self.hw_debug_mode.load(Ordering::Relaxed)
    }

    pub fn set_hw_in_debug_mode(&self, in_debug_mode: bool) {
        self.hw_debug_mode.store(in_debug_mode, Ordering::Relaxed);
    }

    pub fn robot_tipped(&self) -> bool {
        self.robot_tipped.load(Ordering::Relaxed)
    }

    pub fn set_robot_tipped(&self, tipped: bool) {
        self.robot_tipped.store(tipped, Ordering::Relaxed);
    }

    pub fn get_radio_inop(&self) -> bool {
        self.radio_inop.load(Ordering::Relaxed)
    }

    pub fn set_radio_inop(&self, radio_inop: bool) {
        self.radio_inop.store(radio_inop, Ordering::Relaxed);
    }

    pub fn get_imu_inop(&self) -> bool {
        self.imu_inop.load(Ordering::Relaxed)
    }

    pub fn set_imu_inop(&self, imu_inop: bool) {
        self.imu_inop.store(imu_inop, Ordering::Relaxed);
    }

    pub fn get_wheels_inop(&self) -> u8 {
        self.wheels_inop.load(Ordering::Relaxed)
    }

    pub fn set_wheels_inop(&self, wheels_inop: u8) {
        self.wheels_inop.store(wheels_inop, Ordering::Relaxed);
    }

    pub fn get_dribbler_inop(&self) -> bool {
        self.dribbler_inop.load(Ordering::Relaxed)
    }

    pub fn set_dribbler_inop(&self, drib_inop: bool) {
        self.dribbler_inop.store(drib_inop, Ordering::Relaxed);
    }

    pub fn get_kicker_inop(&self) -> bool {
        self.kicker_inop.load(Ordering::Relaxed)
    }

    pub fn set_kicker_inop(&self, kicker_inop: bool) {
        self.kicker_inop.store(kicker_inop, Ordering::Relaxed);
    }

    pub fn get_power_inop(&self) -> bool {
        self.power_inop.load(Ordering::Relaxed)
    }

    pub fn set_power_inop(&self, power_inop: bool) {
        self.power_inop.store(power_inop, Ordering::Relaxed);
    }

    pub fn shutdown_requested(&self) -> bool {
        self.shutdown_requested.load(Ordering::Relaxed)
    }

    pub fn flag_shutdown_requested(&self) {
        self.shutdown_requested.store(true, Ordering::Relaxed);
    }

    pub fn get_battery_low(&self) -> bool {
        self.battery_low.load(Ordering::Relaxed)
    }

    pub fn set_battery_low(&self, battery_low: bool) {
        self.battery_low.store(battery_low, Ordering::Relaxed);
    }

    pub fn get_battery_crit(&self) -> bool {
        self.battery_crit.load(Ordering::Relaxed)
    }

    pub fn set_battery_crit(&self, battery_crit: bool) {
        self.battery_crit.store(battery_crit, Ordering::Relaxed);
    }

    pub fn get_radio_network_ok(&self) -> bool {
        self.radio_network_ok.load(Ordering::Relaxed)
    }

    pub fn set_radio_network_ok(&self, network_ok: bool) {
        self.radio_network_ok.store(network_ok, Ordering::Relaxed);
    }

    pub fn get_radio_bridge_ok(&self) -> bool {
        self.radio_bridge_ok.load(Ordering::Relaxed)
    }

    pub fn set_radio_bridge_ok(&self, bridge_ok: bool) {
        self.radio_bridge_ok.store(bridge_ok, Ordering::Relaxed);
    }

    pub fn get_radio_send_extended_telem(&self) -> bool {
        self.radio_send_extended_telem.load(Ordering::SeqCst)
    }

    pub fn set_radio_send_extended_telem(&self, send_extended_telem: bool) {
        self.radio_send_extended_telem.store(send_extended_telem, Ordering::SeqCst);
    }

    pub fn ball_detected(&self) -> bool {
        self.ball_detected.load(Ordering::Relaxed)
    }

    pub fn set_ball_detected(&self, ball_detected: bool) {
        self.ball_detected.store(ball_detected, Ordering::Relaxed);
    }

    pub fn kicker_shutdown_complete(&self) -> bool {
        self.kicker_shutdown_complete.load(Ordering::Relaxed)
    }

    pub fn set_kicker_shutdown_complete(&self, shutdown_complete: bool) {
        self.kicker_shutdown_complete.store(shutdown_complete, Ordering::Relaxed);
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct RobotState {
    pub hw_init_state_valid: bool,

    pub hw_robot_id: u8,
    pub hw_robot_team_is_blue: bool,
    pub hw_wifi_network_index: usize,
    pub hw_wifi_driver_use_flow_control: bool,
    pub hw_debug_mode: bool,

    pub radio_inop: bool,
    pub imu_inop: bool,
    pub kicker_inop: bool,
    pub power_inop: bool,
    pub wheels_inop: u8,
    pub dribbler_inop: bool,

    pub last_packet_receive_time_ticks_ms: u32,
    pub radio_network_ok: bool,
    pub radio_bridge_ok: bool,
    pub radio_send_extended_telem: bool,

    pub battery_low: bool,
    pub battery_crit: bool,

    pub robot_tipped: bool,

    pub ball_detected: bool,
    pub shutdown_requested: bool,
    pub kicker_shutdown_complete: bool,
}