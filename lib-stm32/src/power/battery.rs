use mod::math::LinearMap;

const f32 FULL_BATT_VOLTAGE = 4.2;
const f32 EMPTY_BATT_VOLTAGE = 3.4;
const f32 CRITICAL_BATT_VOLTAGE = 3.2;

pub struct BatteryCell {
    // TODO (Christian) : Eventually we'll use a non linear mapping
    // that more realistically resembles the real "S" curve of battery
    // voltage drain
    ref_map: LinearMap,
    real_map: LinearMap,
    vref_critical: f32
};

pub struct Battery {
    cells: [BatteryCell; 6]
};

impl Battery {
    pub fn new(vref_criticals: [f32; 6], vref_fulls: [f32; 6]) -> Self {

    }

    pub fn get_total_voltage() -> f32 {

    }
}

impl BatteryCell {
    pub fn new(vref_critical: f32, vref_full: f32) -> Self {
        BatteryCell {
            LinearMap::new(vref_critical, vref_full),
            LinearMap::new(CRITICAL_BATT_VOLTAGE, FULL_BATT_VOLTAGE)
        }
    }

    pub fn convert_vref_to_real(vref: f32) -> f32 {
        let voltage = self.ref_map.linear_map_to_new_bounds(vref, self.real_map);
        return voltage;
    }
}