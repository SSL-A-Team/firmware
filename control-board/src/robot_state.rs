use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

pub struct SharedRobotState {
    hw_init_state_valid: AtomicBool,

    hw_robot_id: AtomicU8,
    hw_robot_team_is_blue: AtomicBool,
    hw_wifi_network_index: AtomicU8,
    hw_debug_mode: AtomicBool,

    radio_inop: AtomicBool,
    imu_inop: AtomicBool,
    kicker_inop: AtomicBool,
    wheels_inop: AtomicU8,
    dribbler_inop: AtomicBool,

    // systick is 1us which in a u64 gives an uptime of ~500000 yrs,
    // 32bit Cortex M has no AtomicU64, so we'll sync on milliseconds
    // overflow occurs in ~49.7 days
    last_packet_receive_time_ticks_ms: AtomicU32,
    radio_connection_ok: AtomicBool,

    battery_pct: AtomicU8,
    battery_ok: AtomicBool,
}

impl SharedRobotState {
    pub const fn new() -> SharedRobotState {
        SharedRobotState {
            hw_init_state_valid: AtomicBool::new(false),
            hw_robot_id: AtomicU8::new(0),
            hw_robot_team_is_blue: AtomicBool::new(false),
            hw_wifi_network_index: AtomicU8::new(0),
            hw_debug_mode: AtomicBool::new(true),
            radio_inop: AtomicBool::new(true),
            imu_inop: AtomicBool::new(true),
            kicker_inop: AtomicBool::new(true),
            wheels_inop: AtomicU8::new(0x0F), 
            dribbler_inop: AtomicBool::new(true),
            last_packet_receive_time_ticks_ms: AtomicU32::new(0),
            radio_connection_ok: AtomicBool::new(false), 
            battery_pct: AtomicU8::new(0),
            battery_ok: AtomicBool::new(false),
        }
    }

    pub fn get_state(&self) -> RobotState {
        RobotState {
            hw_init_state_valid: self.hw_init_state_valid(),
            hw_robot_id: self.get_hw_robot_id(),
            hw_robot_team_is_blue: self.hw_robot_team_is_blue(),
            hw_wifi_network_index: self.hw_wifi_network_index(),
            hw_debug_mode: self.hw_in_debug_mode(),

            radio_inop: true,
            imu_inop: true,
            kicker_inop: true,
            wheels_inop: 0xFF,
            dribbler_inop: true,
        
            last_packet_receive_time_ticks_ms: 0,
            radio_connection_ok: false,
        
            battery_pct: 0,
            battery_ok: false,
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
 
    pub fn hw_in_debug_mode(&self) -> bool {
        self.hw_debug_mode.load(Ordering::Relaxed)
    }

    pub fn set_hw_in_debug_mode(&self, in_debug_mode: bool) {
        self.hw_debug_mode.store(in_debug_mode, Ordering::Relaxed);
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct RobotState {
    pub hw_init_state_valid: bool,

    pub hw_robot_id: u8,
    pub hw_robot_team_is_blue: bool,
    pub hw_wifi_network_index: usize,
    pub hw_debug_mode: bool,

    pub radio_inop: bool,
    pub imu_inop: bool,
    pub kicker_inop: bool,
    pub wheels_inop: u8,
    pub dribbler_inop: bool,

    pub last_packet_receive_time_ticks_ms: u32,
    pub radio_connection_ok: bool,

    pub battery_pct: u8,
    pub battery_ok: bool,
}