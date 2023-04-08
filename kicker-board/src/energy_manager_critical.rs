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

use crate::pins::{ChargePin, RegulatorDonePin, RegulatorFaultPin, HighVoltageReadPin, BatteryVoltageReadPin};

enum EnergyManagementState {
    FaultInitializeSafetyShutdown,
    FaultSafetyShutdownComplete,
}

pub struct CriticalEnergyManager {
    // external interface    
    charge_pin: ChargePin,
    done_pin: RegulatorDonePin,
    fault_pin: RegulatorFaultPin,
    high_voltage_feedback_pin: HighVoltageReadPin, 
    battery_voltage_feedback_pin: BatteryVoltageReadPin,

    // internal peripherals

    // record keeping
}

impl CriticalEnergyManager {
    pub fn new(
            charge_pin: ChargePin, 
            done_pin: RegulatorDonePin, 
            fault_pin: RegulatorFaultPin, 
            high_voltage_feedback_pin: HighVoltageReadPin, 
            battery_voltage_feedback_pin: BatteryVoltageReadPin
    ) -> CriticalEnergyManager {
        CriticalEnergyManager {
            charge_pin,
            done_pin,
            fault_pin,
            high_voltage_feedback_pin,
            battery_voltage_feedback_pin
        }
    }

    fn update_tick() {
        // read adc
    }
}