use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PowerState {
    // power status packet flags
    pub power_ok: bool,
    pub power_rail_3v3_ok: bool,
    pub power_rail_5v0_ok: bool,
    pub power_rail_12v0_ok: bool,
    pub battery_low_warn: bool,
    pub battery_low_crit: bool,
    pub high_current_operations_allowed: bool,
    pub shutdown_requested: bool,

    // internal state
    pub shutdown_force: bool,
    pub shutdown_ready: bool,
    pub balance_connected: bool,
    pub coms_established: bool,
}

pub struct SharedPowerState {
    inner: Mutex<CriticalSectionRawMutex, PowerState>, // STM32G0 doesn't have atomic instruction support, use mutex
}

impl SharedPowerState {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(PowerState {
                power_ok: false,
                power_rail_3v3_ok: false,
                power_rail_5v0_ok: false,
                power_rail_12v0_ok: false,
                battery_low_warn: false,
                battery_low_crit: false,
                high_current_operations_allowed: true,
                shutdown_requested: false,
                shutdown_force: false,
                shutdown_ready: false,
                balance_connected: false,
                coms_established: false,
            }),
        }
    }

    pub async fn get_state(&self) -> PowerState {
        let guard = self.inner.lock().await;
        PowerState {
            power_ok: guard.power_ok,
            power_rail_3v3_ok: guard.power_rail_3v3_ok,
            power_rail_5v0_ok: guard.power_rail_5v0_ok,
            power_rail_12v0_ok: guard.power_rail_12v0_ok,
            battery_low_warn: guard.battery_low_warn,
            battery_low_crit: guard.battery_low_crit,
            high_current_operations_allowed: guard.high_current_operations_allowed,
            shutdown_requested: guard.shutdown_requested,
            shutdown_force: guard.shutdown_force,
            shutdown_ready: guard.shutdown_ready,
            balance_connected: guard.balance_connected,
            coms_established: guard.coms_established,
        }
    }

    pub async fn get_power_ok(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.power_ok
    }

    pub async fn set_power_ok(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.power_ok = new_val;
    }

    pub async fn get_power_rail_3v3_ok(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.power_rail_3v3_ok
    }

    pub async fn set_power_rail_3v3_ok(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.power_rail_3v3_ok = new_val;
    }

    pub async fn get_power_rail_5v0_ok(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.power_rail_5v0_ok
    }

    pub async fn set_power_rail_5v0_ok(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.power_rail_5v0_ok = new_val;
    }

    pub async fn get_power_rail_12v0_ok(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.power_rail_12v0_ok
    }

    pub async fn set_power_rail_12v0_ok(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.power_rail_12v0_ok = new_val;
    }

    pub async fn get_battery_low_warn(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.battery_low_warn
    }

    pub async fn set_battery_low_warn(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.battery_low_warn = new_val
    }

    pub async fn get_battery_low_crit(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.battery_low_crit
    }

    pub async fn set_battery_low_crit(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.battery_low_crit = new_val
    }

    pub async fn get_high_current_operations_allowed(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.high_current_operations_allowed
    }

    pub async fn set_high_current_operations_allowed(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.high_current_operations_allowed = new_val;
    }

    pub async fn get_shutdown_requested(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.shutdown_requested
    }

    pub async fn set_shutdown_requested(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.shutdown_requested = new_val;
    }

    pub async fn get_shutdown_force(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.shutdown_force
    }

    pub async fn set_shutdown_force(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.shutdown_force = new_val;
    }

    pub async fn get_shutdown_ready(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.shutdown_ready
    }

    pub async fn set_shutdown_ready(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.shutdown_ready = new_val;
    }

    pub async fn get_balance_connected(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.balance_connected
    }

    pub async fn set_balance_connected(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.balance_connected = new_val;
    }

    pub async fn get_coms_established(&self) -> bool {
        let guard = self.inner.lock().await;
        guard.coms_established
    }

    pub async fn set_coms_established(&self, new_val: bool) {
        let mut guard = self.inner.lock().await;
        guard.coms_established = new_val;
    }
}
