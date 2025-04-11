// Variable Naming Scheme
// warnings are reported to software and have no uncommanded effects
// criticals are reported to software and force halt high current operations (driving, dribbling, kick/chip charging)
// powerdowns result in an automatic initiation software shutdown

// power regulators on-board can take a max of 40, and 60 volts respectively. 
// stspins can take 45V
// kicker can take 40V
// the ADC measurement range is 0-36V
// Lets set critically high just below that.
pub const VBATT_TOO_HIGH_WARN: f32 = 26.0;
pub const VBATT_TOO_HIGH_CRITICAL: f32 = 30.0;

// Other regulators have a 1ish voltage margin, putting them worst case around 13V for 12V air drib, which is much less
// vaguely based on https://www.ufinebattery.com/blog/useful-overview-of-lipo-battery-voltage/
pub const VBATT_TOO_LOW_WARN: f32 = 21.0;
pub const VBATT_TOO_LOW_CRITICAL: f32 = 20.0;
pub const VBATT_TOO_LOW_POWERDOWN: f32 = 19.2;

pub const REGULATION_HIGH_WARN_MULT: f32 = 1.05;  // tolerate 5% error for warning
pub const REGULATION_HIGH_CRIT_MULT: f32 = 1.10;  // tolerate 10% for critical
pub const REGULATION_LOW_WARN_MULT: f32 = 0.95;   // tolerate 5% error for warning
pub const REGULATION_LOW_CRIT_MULT: f32 = 0.90;   // tolerate 10% for critical

// 12v0
pub const POWER_RAIL_12V0_TOO_HIGH_WARN: f32 = 12.0 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_12V0_TOO_HIGH_CRIT: f32 = 12.0 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_12V0_TOO_LOW_WARN: f32 = 12.0 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_12V0_TOO_LOW_CRIT: f32 = 12.0 * REGULATION_LOW_CRIT_MULT;

// 5v0
pub const POWER_RAIL_5V0_TOO_HIGH_WARN: f32 = 5.00 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_5V0_TOO_HIGH_CRITICAL: f32 = 5.00 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_5V0_TOO_LOW_WARN: f32 = 5.00 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_5V0_TOO_LOW_CRITICAL: f32 = 5.00 * REGULATION_LOW_CRIT_MULT;

// 3v3
pub const POWER_RAIL_3V3_TOO_HIGH_WARN: f32 = 3.30 * REGULATION_HIGH_WARN_MULT;
pub const POWER_RAIL_3V3_TOO_HIGH_CRITICAL: f32 = 3.30 * REGULATION_HIGH_CRIT_MULT;
pub const POWER_RAIL_3V3_TOO_LOW_WARN: f32 = 3.30 * REGULATION_LOW_WARN_MULT;
pub const POWER_RAIL_3V3_TOO_LOW_CRITICAL: f32 = 3.30 * REGULATION_LOW_CRIT_MULT;

// battery
pub const LIPO_CELL_TOO_HIGH_WARN: f32 = 4.3;
pub const LIPO_CELL_TOO_HIGH_CRITICAL: f32 = 4.4;
pub const LIPO_CELL_TOO_LOW_WARN: f32 = 3.4;
pub const LIPO_CELL_TOO_LOW_CRITICAL: f32 = 3.3;
pub const LIPO_CELL_TOO_LOW_POWERDOWN: f32 = 3.2;
pub const LIPO_CELL_MAX_DIFFERENCE_WARN: f32 = 0.2;
pub const LIPO_CELL_MAX_DIFFERENCE_CRITICAL: f32 = 0.3;