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

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};
use libm::{fmaxf, fminf};
use ateam_lib_stm32::math::linear::LinearMap;

const MIN_KICK_DURATION_US: f32 = 500.0;
const MAX_KICK_DURATION_US: f32 = 4500.0;  // 10ms (10k us) max power kick
const MAX_CHIP_DURATION_US: f32 = 10000.0;  // 10ms (10k us) max power kick

const CHARGE_COOLDOWN: Duration = Duration::from_micros(50);  // 50 micros (5 switching cycles) to confirm switching regulator is off
const KICK_COOLDOWN: Duration = Duration::from_millis(500);  // TODO: get estiamted mechanical return time from Matt and pad it
const CHIP_COOLDOWN: Duration = Duration::from_millis(500);  // TODO: get estiamted mechanical return time from Matt and pad it

const MAX_SAFE_RAIL_VOLTAGE: f32 = 190.0;  // rail is rated for 200V, and should stop charging around 180V
const VBATT_OVERVOLTAGE_LOCKOUT: f32 = 27.2;
const VBATT_UNDERVOLTAGE_LOCKOUT: f32 = 17.2;


#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum KickType {
    Kick,
    Chip,
    None
}

pub struct KickManager<'a> {
    // external interface
    charge_pin: Output<'a>,
    kick_pin: Output<'a>,
    chip_pin: Output<'a>,

    // record keeping
    error_latched: bool,
}

impl<'a> KickManager<'a> {
    pub fn new(
            charge_pin: Output<'a>,
            kick_pin: Output<'a>,
            chip_pin: Output<'a>,
    ) -> KickManager<'a> {
        KickManager {
            charge_pin,
            kick_pin,
            chip_pin,
            error_latched: false
        }
    }

    pub async fn command(&mut self, battery_voltage: f32, rail_voltage:f32, charge: bool, kick_type: KickType, kick_speed: f32) -> Result<(), ()> {
        // latch an error for invalid battery voltage
        if battery_voltage > VBATT_OVERVOLTAGE_LOCKOUT || battery_voltage < VBATT_UNDERVOLTAGE_LOCKOUT {
            self.error_latched = true;
        }

        // latch and error for unsafe rail voltage
        if rail_voltage > MAX_SAFE_RAIL_VOLTAGE {
            self.error_latched = true;
        }

        // if an error is latched, disable systems
        if self.error_latched {
            self.charge_pin.set_low();
            self.kick_pin.set_low();
            self.chip_pin.set_low();

            return Err(());
        }

        // set charge duration via mapping from kick speed
        let kick_speed_map = LinearMap::new(0f32, 5.5f32); // Max kick speed is approx 5.5m/s
        let kick_duration_map = LinearMap::new(MIN_KICK_DURATION_US, MAX_KICK_DURATION_US);
        let charge_duration_us: f32 = kick_speed_map.linear_map_to_new_bounds(kick_speed, kick_duration_map);

        // handle charge, kick, and chip
        match kick_type {
            KickType::Kick => {
                // disable kick and chip
                self.kick_pin.set_low();
                self.chip_pin.set_low();

                // disable charging and wait the cooldown
                self.charge_pin.set_low();
                Timer::after(CHARGE_COOLDOWN).await;

                // beign kick, wait time to determine power
                self.kick_pin.set_high();
                Timer::after(Duration::from_micros(charge_duration_us as u64)).await;

                // end kick
                self.kick_pin.set_low();
                Timer::after(Duration::from_micros(10)).await;

                // reenable charging if requested
                if charge {
                    self.charge_pin.set_high();
                }

                // cooldown for mechanical return of the subsystem
                Timer::after(KICK_COOLDOWN).await;
            },
            KickType::Chip => {
                // disable kick and chip
                self.kick_pin.set_low();
                self.chip_pin.set_low();

                // disable charging and wait the cooldown
                self.charge_pin.set_low();
                Timer::after(CHARGE_COOLDOWN).await;

                // begin chip, wait time to determine power
                self.chip_pin.set_high();
                Timer::after(Duration::from_micros((MAX_CHIP_DURATION_US * kick_speed) as u64)).await;

                // end chip
                self.chip_pin.set_low();
                Timer::after(Duration::from_micros(10)).await;

                // reenable charging if requested
                if charge {
                    self.charge_pin.set_high();
                }
                
                // cooldown for mechanical return of the subsystem
                Timer::after(CHIP_COOLDOWN).await;
            }
            KickType::None => {
                // disable kick and chip
                self.kick_pin.set_low();
                self.chip_pin.set_low();

                // enable charging if requested
                if charge {
                    self.charge_pin.set_high();
                } else {
                    self.charge_pin.set_low();
                }
            }            
        }

        return Ok(());
    }

    pub fn clear_error(&mut self) {
        self.error_latched = false;
    }
}