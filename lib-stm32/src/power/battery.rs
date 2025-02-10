use mod::math::LinearMap;

pub struct BatteryCell {
    ref_map: LinearMap,
    real_map: LinearMap 
};

pub struct Battery {
    cells: [BatteryCell; 6]
};

impl Battery {
    pub fn new(vref_empties: [f32; 6], vref_fulls: [f32; 6], v_max: f32) -> Self {
         
    }

    pub fn get_total_voltage() -> f32 {

    }
}

impl BatteryCell {
    pub fn new(vref_empty: f32, vref_full: f32, v_max: f32) -> Self {
        BatteryCell {
            LinearMap::new(vref_empty, vref_full),
            LinearMap::new(0.0, v_max)
        }
    }

    pub fn convert_vref_to_real(vref: f32) -> f32 {
        let voltage = self.ref_map.linear_map_to_new_bounds(vref, self.real_map);
        return voltage;
    }
}