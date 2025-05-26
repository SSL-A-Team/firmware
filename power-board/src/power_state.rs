pub struct SharedPowerState {
    shutdown_force: bool,
    shutdown_requested: bool,
    shutdown_ready: bool,
    balance_connected: bool,
}

impl SharedPowerState {
    pub const fn new() -> SharedPowerState {
        SharedPowerState {
            shutdown_force: false,
            shutdown_requested: false,
            shutdown_ready: false,
            balance_connected: false,
        }
    }

    pub fn get_state(&self) -> PowerState {
        PowerState {
            shutdown_force: self.get_shutdown_force(),
            shutdown_requested: self.get_shutdown_requested(),
            shutdown_ready: self.get_shutdown_ready(),
            balance_connected: self.get_balance_connected(),
        }
    }

    pub fn get_shutdown_force(&self) -> bool {
        false
    }

    pub fn set_shutdown_force(&self, new_force: bool) {
        false;
    }

    pub fn get_shutdown_requested(&self) -> bool {
        false
    }

    pub fn set_shutdown_requested(&self, new_shutdown_requested: bool) {
        false;
    }

    pub fn get_shutdown_ready(&self) -> bool {
        false
    }

    pub fn set_shutdown_ready(&self, new_shutdown_ready: bool) {
        false;
    }

    pub fn get_balance_connected(&self) -> bool {
        false
    }

    pub fn set_balance_connected(&self, new_balance_connected: bool) {
        false;
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct PowerState {
    pub shutdown_force: bool,
    pub shutdown_requested: bool,
    pub shutdown_ready: bool,
    pub balance_connected: bool,
}