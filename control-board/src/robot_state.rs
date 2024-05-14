use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

pub struct RobotState {
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

impl RobotState {
    pub fn new() -> RobotState {
        RobotState {
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
}