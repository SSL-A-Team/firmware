/*
 * This file is responsible for managing the mechanically and
 * electrically critical sections of operation. Foundational 
 * assumptions and requirements are listed below.
 * 
 * ASSUMPTIONS:
 * 1. The init code hands off to the manager in a default intert state
 *  (no charge commanded, no discharge commanded on any channel).
 * 2. There is no unsafe use of the relevant PAC wrappers outside of
 *   this file.
 * 3. The Rust type ownership system prevents "safe" external access of
 *   the critical hardware interface.
 * 4. Charging (active or in soft stop) during any discharge event is 
 *   unsafe behavior.
 * 5. Discharging on multiple channels at once is unsafe.
 * 6. All async call backs run at the highest priority.
 * 
 * HYPOTHESES:
 * 1. The charge pin is not in an active state when kick discharge pin
 *   is in an active state.
 * 2. The charge pin is not in an active state when chip discharge pin
 *   is in an active state.
 */


enum EnergyManagementState {

}

pub struct CriticalEnergyManager {

}

impl CriticalEnergyManager {
    pub fn new() -> CriticalEnergyManager {
        CriticalEnergyManager {}
    }
}