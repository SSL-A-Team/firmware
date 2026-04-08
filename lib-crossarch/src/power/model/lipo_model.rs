use crate::math::{linear_map::LinearMap, range::Range};

pub const LIPO_CELL_MAX_VOLTAGE: f32 = 4.2;

pub fn lipo_pct_interp(voltage: f32, model: &[(f32, f32); 21]) -> f32 {
    if voltage <= model.first().unwrap().0 {
        return model.first().unwrap().1;
    }

    if voltage >= model.last().unwrap().0 {
        return model.last().unwrap().1;
    }

    for i in 0..20 {
        if model[i].0 <= voltage && voltage < model[i + 1].0 {
            return LinearMap::<f32>::map_ranges_bounded(
                voltage,
                Range::new(model[i].0, model[i + 1].0),
                Range::new(model[i].1, model[i + 1].1),
            );
        }
    }

    model.first().unwrap().1
}

pub const LIPO_1S_VOLTAGE_PERCENT: [(f32, f32); 21] = [
    (3.27, 0.0),
    (3.61, 5.0),
    (3.69, 10.0),
    (3.71, 15.0),
    (3.73, 20.0),
    (3.75, 25.0),
    (3.77, 30.0),
    (3.79, 35.0),
    (3.80, 40.0),
    (3.82, 45.0),
    (3.84, 50.0),
    (3.85, 55.0),
    (3.87, 60.0),
    (3.91, 65.0),
    (3.95, 70.0),
    (3.98, 75.0),
    (4.02, 80.0),
    (4.08, 85.0),
    (4.11, 90.0),
    (4.15, 95.0),
    (4.20, 100.0),
];

pub const LIPO_6S_VOLTAGE_PERCENT: [(f32, f32); 21] = [
    (19.64, 0.0),
    (21.65, 5.0),
    (22.12, 10.0),
    (22.24, 15.0),
    (22.36, 20.0),
    (22.48, 25.0),
    (22.60, 30.0),
    (22.72, 35.0),
    (22.77, 40.0),
    (22.89, 45.0),
    (23.01, 50.0),
    (23.13, 55.0),
    (23.25, 60.0),
    (23.48, 65.0),
    (23.72, 70.0),
    (23.90, 75.0),
    (24.14, 80.0),
    (24.49, 85.0),
    (24.67, 90.0),
    (24.90, 95.0),
    (25.20, 100.0),
];
