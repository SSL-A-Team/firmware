// Variable Naming Scheme
// warnings are reported to software and have no uncommanded effects
// criticals are reported to software and force halt high current operations (driving, dribbling, kick/chip charging)
// powerdowns result in an automatic initiation software shutdown

use ateam_lib_stm32::{math::{linear_map::LinearMap, range::Range}, power::{battery::BatteryConfig, model::lipo_model::LIPO_CELL_MAX_VOLTAGE, PowerRailParameters}};

// power regulators on-board can take a max of 40, and 60 volts respectively. 
// stspins can take 45V
// kicker can take 40V
// the ADC measurement range is 0-36V
// Lets set critically high just below that.
pub const VBATT_TOO_HIGH_WARN: f32 = 26.0;
pub const VBATT_TOO_HIGH_CRITICAL: f32 = 30.0;

// Other regulators have a 1ish voltage margin, putting them worst case around 13V for 12V air drib, which is much less
// vaguely based on https://www.ufinebattery.com/blog/useful-overview-of-lipo-battery-voltage/
pub const VBATT_TOO_LOW_WARN: f32 = 19.5;
pub const VBATT_TOO_LOW_CRITICAL: f32 = 19.0;
pub const POWER_RAIL_BATTERY_PARAMETERS: PowerRailParameters<f32> = PowerRailParameters {
    min_value_crit: VBATT_TOO_LOW_CRITICAL,
    min_value_warn: VBATT_TOO_LOW_WARN,
    max_value_warn: VBATT_TOO_HIGH_WARN,
    max_value_crit: VBATT_TOO_HIGH_CRITICAL,
};

pub const REGULATION_HIGH_WARN_MULT: f32 = 1.05;  // tolerate 5% error for warning
pub const REGULATION_HIGH_CRIT_MULT: f32 = 1.10;  // tolerate 10% for critical
pub const REGULATION_LOW_WARN_MULT: f32 = 0.95;   // tolerate 5% error for warning
pub const REGULATION_LOW_CRIT_MULT: f32 = 0.90;   // tolerate 10% for critical

// 12v0
pub const POWER_RAIL_12V0_TOO_HIGH_WARN: f32 = 12.0 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_12V0_TOO_HIGH_CRIT: f32 = 12.0 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_12V0_TOO_LOW_WARN: f32 = 12.0 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_12V0_TOO_LOW_CRIT: f32 = 12.0 * REGULATION_LOW_CRIT_MULT;
pub const POWER_RAIL_12V0_PARAMETERS: PowerRailParameters<f32> = PowerRailParameters {
    min_value_crit: POWER_RAIL_12V0_TOO_LOW_CRIT,
    min_value_warn: POWER_RAIL_12V0_TOO_LOW_WARN,
    max_value_warn: POWER_RAIL_12V0_TOO_HIGH_WARN,
    max_value_crit: POWER_RAIL_12V0_TOO_HIGH_CRIT,
};

// 5v0
pub const POWER_RAIL_5V0_TOO_HIGH_WARN: f32 = 5.00 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_5V0_TOO_HIGH_CRITICAL: f32 = 5.00 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_5V0_TOO_LOW_WARN: f32 = 5.00 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_5V0_TOO_LOW_CRITICAL: f32 = 5.00 * REGULATION_LOW_CRIT_MULT;
pub const POWER_RAIL_5V0_PARAMETERS: PowerRailParameters<f32> = PowerRailParameters {
    min_value_crit: POWER_RAIL_5V0_TOO_LOW_CRITICAL,
    min_value_warn: POWER_RAIL_5V0_TOO_LOW_WARN,
    max_value_warn: POWER_RAIL_5V0_TOO_HIGH_WARN,
    max_value_crit: POWER_RAIL_5V0_TOO_HIGH_CRITICAL,
};

// 3v3
pub const POWER_RAIL_3V3_TOO_HIGH_WARN: f32 = 3.30 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_3V3_TOO_HIGH_CRITICAL: f32 = 3.30 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_3V3_TOO_LOW_WARN: f32 = 3.30 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_3V3_TOO_LOW_CRITICAL: f32 = 3.30 * REGULATION_LOW_CRIT_MULT;
pub const POWER_RAIL_3V3_PARAMETERS: PowerRailParameters<f32> = PowerRailParameters {
    min_value_crit: POWER_RAIL_3V3_TOO_LOW_CRITICAL,
    min_value_warn: POWER_RAIL_3V3_TOO_LOW_WARN,
    max_value_warn: POWER_RAIL_3V3_TOO_HIGH_WARN,
    max_value_crit: POWER_RAIL_3V3_TOO_HIGH_CRITICAL,
};

const DUAL_RTOL_1PCT_CIEL: f32 = 1.02;
const DUAL_RTOL_1PCT_FLOOR: f32 = 0.98;


// battery
pub const LIPO_BALANCE_UNCONNECTED_THRESH: f32 = 0.150;

pub const LIPO_CELL_TOO_HIGH_WARN: f32 = 4.25 * DUAL_RTOL_1PCT_CIEL;
pub const LIPO_CELL_TOO_HIGH_CRITICAL: f32 = 4.35 * DUAL_RTOL_1PCT_CIEL;
pub const LIPO_CELL_TOO_HIGH_POWERDOWN: f32 = 4.4 * DUAL_RTOL_1PCT_CIEL;
pub const LIPO_CELL_TOO_LOW_WARN: f32 = 3.5 * DUAL_RTOL_1PCT_FLOOR;
pub const LIPO_CELL_TOO_LOW_CRITICAL: f32 = 3.4 * DUAL_RTOL_1PCT_FLOOR;
pub const LIPO_CELL_TOO_LOW_POWERDOWN: f32 = 3.3 * DUAL_RTOL_1PCT_FLOOR;
pub const LIPO_CELL_MAX_DIFFERENCE_WARN: f32 = 0.2 * DUAL_RTOL_1PCT_CIEL;
pub const LIPO_CELL_MAX_DIFFERENCE_CRITICAL: f32 = 0.3 * DUAL_RTOL_1PCT_CIEL;
pub const LIPO_CELL_MAX_DIFFERENCE_POWERDOWN: f32 = 0.4 * DUAL_RTOL_1PCT_CIEL;

pub const LIPO_BATTERY_CONFIG_6S: BatteryConfig<f32> = BatteryConfig {
    cell_voltage_low_warn: LIPO_CELL_TOO_LOW_WARN,
    cell_voltage_low_crit: LIPO_CELL_TOO_LOW_CRITICAL,
    cell_voltage_low_power_off: LIPO_CELL_TOO_LOW_POWERDOWN,
    cell_voltage_high_warn: LIPO_CELL_TOO_HIGH_WARN,
    cell_voltage_high_crit: LIPO_CELL_TOO_HIGH_CRITICAL,
    cell_voltage_high_power_off: LIPO_CELL_TOO_HIGH_POWERDOWN,
    cell_voltage_difference_warn: LIPO_CELL_MAX_DIFFERENCE_WARN,
    cell_voltage_difference_crit: LIPO_CELL_MAX_DIFFERENCE_CRITICAL,
    cell_votlage_difference_off: LIPO_CELL_MAX_DIFFERENCE_POWERDOWN,
};

// LHS ranges from resistor dividers in power/power_mon sch page
pub const LIPO6S_BALANCE_RAW_SAMPLES_TO_VOLTAGES: [LinearMap<f32>; 6] = [
    LinearMap::new(Range::new(0.0, 2.000), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE)),
    LinearMap::new(Range::new(0.0, 1.997), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE * 2.0)),
    LinearMap::new(Range::new(0.0, 1.984), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE * 3.0)),
    LinearMap::new(Range::new(0.0, 2.045), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE * 4.0)),
    LinearMap::new(Range::new(0.0, 1.996), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE * 5.0)),
    LinearMap::new(Range::new(0.0, 2.062), Range::new(0.0, LIPO_CELL_MAX_VOLTAGE * 6.0)),
];