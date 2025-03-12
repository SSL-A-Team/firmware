// Variable Naming Scheme
// warnings are reported to software and have no uncommanded effects
// criticals are reported to software and force halt high current operations (driving, dribbling, kick/chip charging)
// powerdowns result in an automatic initiation software shutdown

// power regulators on-board can take a max of 40, and 60 volts respectively. 
// stspins can take 45V
// kicker can take 40V
// the ADC measurement range is 0-36V
// Lets set critically high just below that.
const VBATT_TOO_HIGH_WARN: f32 = 26.0;
const VBATT_TOO_HIGH_CRITICAL: f32 = 30.0;

// Kicker LDO will approach dropout around 18.5, so we'll set that as critical, and warn just above
// Other regulators have a 1ish voltage margin, putting them worst case around 13V for 12V air drib, which is much less
const VBATT_TOO_LOW_WARN: f32 = 19.0;
const VBATT_TOO_LOW_CRITICAL: f32 = 18.5;

const REGULATION_HIGH_WARN_MULT: f32 = 1.05;  // tolerate 5% error for warning
const REGULATION_HIGH_CRIT_MULT: f32 = 1.10;  // tolerate 10% for critical
const REGULATION_LOW_WARN_MULT: f32 = 0.95;   // tolerate 5% error for warning
const REGULATION_LOW_CRIT_MULT: f32 = 0.90;   // tolerate 10% for critical

// 5v0
const POWER_RAIL_5v0_TOO_HIGH_WARN: f32 = 5.00 * REGULATION_HIGH_WARN_MULT;
const POWER_RAIL_5v0_TOO_HIGH_CRITICAL: f32 = 5.00 * REGULATION_HIGH_CRIT_MULT;
const POWER_RAIL_5v0_TOO_LOW_WARN: f32 = 5.00 * REGULATION_LOW_WARN_MULT;
const POWER_RAIL_5v0_TOO_LOW_CRITICAL: f32 = 5.00 * REGULATION_LOW_CRIT_MULT;

// 3v3
const POWER_RAIL_3v3_TOO_HIGH_WARN: f32 = 3.30 * REGULATION_HIGH_WARN_MULT;
const POWER_RAIL_3v3_TOO_HIGH_CRITICAL: f32 = 3.30 * REGULATION_HIGH_CRIT_MULT;
const POWER_RAIL_3v3_TOO_LOW_WARN: f32 = 3.30 * REGULATION_LOW_WARN_MULT;
const POWER_RAIL_3v3_TOO_LOW_CRITICAL: f32 = 3.30 * REGULATION_LOW_CRIT_MULT;

// battery
const LIPO_CELL_TOO_HIGH_WARN: f32 = 4.3;
const LIPO_CELL_TOO_HIGH_CRITICAL: f32 = 4.4;
const LIPO_CELL_TOO_LOW_WARN: f32 = 3.4;
const LIPO_CELL_TOO_LOW_CRITICAL: f32 = 3.3;
const LIPO_CELL_TOO_LOW_POWERDOWN: f32 = 3.2;
const LIPO_CELL_MAX_DIFFERENCE_WARN: f32 = 0.2;
const LIPO_CELL_MAX_DIFFERENCE_CRITICAL: f32 = 0.3;