use crate::math::linear::LinearMap;

const FULL_BATT_VOLTAGE : f32 = 4.2;
const EMPTY_BATT_VOLTAGE : f32 = 3.4;
const CRITICAL_BATT_VOLTAGE : f32 = 3.2;

pub struct BatteryCell {
    /// TODO (Christian) : Eventually we'll use a non linear mapping
    /// that more realistically resembles the real "S" curve of battery
    /// voltage drain
    ref_map: LinearMap<f32>,
    real_map: LinearMap<f32>,
    vref_empty: f32,
    last_read: Option<f32>
}

pub struct Battery {
    cells: [BatteryCell; 6]
}

impl Battery {
    pub fn new(vref_criticals: [f32; 6], vref_fulls: [f32; 6], vref_empties: [f32; 6]) -> Self {
        let cells: [BatteryCell; 6] = vref_criticals
            .iter()
            .zip(vref_fulls.iter())
            .zip(vref_criticals.iter())
            .map(|((&vref_critical, &vref_full), &vref_empty) | BatteryCell::new(vref_critical, vref_full, vref_empty))
            .collect();

        Self { cells }
    }

    pub fn get_total_voltage(&self) -> f32 {
    }
}

impl BatteryCell {
    pub fn new(vref_critical: f32, vref_full: f32, vref_empty: f32) -> Self {
        BatteryCell {
            ref_map: LinearMap::new(vref_critical, vref_full),
            real_map: LinearMap::new(CRITICAL_BATT_VOLTAGE, FULL_BATT_VOLTAGE),
            vref_empty: vref_empty,
            last_read: None
        }
    }

    pub fn convert_vref_to_real(&self, vref: f32) -> f32 {
        let voltage = self.ref_map.linear_map_to_new_bounds(vref, self.real_map);
        return voltage;
    }
}