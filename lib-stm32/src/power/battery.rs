
use crate::{filter::Filter, math::{linear_map::LinearMap, Number}};

use super::model::lipo_model::{lipo_pct_interp, LIPO_1S_VOLTAGE_PERCENT};


pub struct BatteryConfig<N: Number> {
    pub cell_voltage_low_warn: N,
    pub cell_voltage_low_crit: N,
    pub cell_voltage_low_power_off: N,
    pub cell_voltage_high_warn: N,
    pub cell_voltage_high_crit: N,
    pub cell_voltage_high_power_off: N,
    pub cell_voltage_difference_warn: N,
    pub cell_voltage_difference_crit: N,
    pub cell_votlage_difference_off: N,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CellVoltageComputeMode {
    Standalone,
    Chained,
}

pub struct LipoModel<'a, const NUM_CELLS: usize, F: Filter<f32>> {
    config: BatteryConfig<f32>,
    cell_range_maps: &'a [LinearMap<f32>; NUM_CELLS],
    cell_voltage_compute_mode: CellVoltageComputeMode,
    cell_votlage_filters: [F; NUM_CELLS],
    cell_voltages: [f32; NUM_CELLS],
    cell_percentages: [u8; NUM_CELLS],
}

impl<'a, const NUM_CELLS: usize, F: Filter<f32>> LipoModel<'a, NUM_CELLS, F> {
    pub fn new(config: BatteryConfig<f32>, cell_range_maps: &'a[LinearMap<f32>; NUM_CELLS], cell_voltage_compute_mode: CellVoltageComputeMode) -> Self {
        Self {
            config,
            cell_range_maps,
            cell_voltage_compute_mode,
            cell_votlage_filters: core::array::from_fn(|_| F::default()),
            cell_voltages: [0.0; NUM_CELLS],
            cell_percentages: [0; NUM_CELLS],
        }
    }

    pub fn add_cell_voltage_samples(&mut self, cell_adc_voltage_samples: &[f32]) {

        // place raw samples into cell_voltage buffer and use it as scratch space
        self.cell_voltages.copy_from_slice(cell_adc_voltage_samples);

        // inplace convert raw voltage samples to cell range
        for (cv, cv_map) in self.cell_voltages.iter_mut().zip(self.cell_range_maps) {
            *cv = cv_map.map(*cv);
        }

        // update filters and inplace update value with filtered value
        for (cv, cv_filt) in self.cell_voltages.iter_mut().zip(self.cell_votlage_filters.iter_mut()) {
            cv_filt.add_sample(*cv);
            cv_filt.update();
            *cv = cv_filt.filtered_value().unwrap_or(0.0);
        }

        // defmt::info!("unconverted cell volatages {}", self.cell_voltages);

        if self.cell_voltage_compute_mode == CellVoltageComputeMode::Chained {
            for i in (1..NUM_CELLS).rev() {
                self.cell_voltages[i] = self.cell_voltages[i] - self.cell_voltages[i - 1];
            }
        }

        for (cp, cv) in self.cell_percentages.iter_mut().zip(self.cell_voltages.into_iter()) {
            *cp = lipo_pct_interp(cv, &LIPO_1S_VOLTAGE_PERCENT) as u8
        }
    }

    pub fn get_cell_voltages(&self) -> &[f32; NUM_CELLS] {
        &self.cell_voltages
    }

    pub fn get_cell_percentages(&self) -> &[u8; NUM_CELLS] {
        &self.cell_percentages
    }

    pub fn get_worst_cell_imbalance(&self) -> f32 {
        let mut min = f32::MAX;
        let mut max = f32::MIN;
        
        for cv in self.cell_voltages.into_iter() {
            min = f32::min(min, cv);
            max = f32::max(max, cv);
        }

        max - min
    }

    pub fn battery_warn(&self) -> bool {
        self.get_worst_cell_imbalance() > self.config.cell_voltage_difference_warn ||
            self.get_cell_voltages().into_iter().any(|cell_voltage| *cell_voltage > self.config.cell_voltage_high_warn || *cell_voltage < self.config.cell_voltage_low_warn)
    }

    pub fn battery_crit(&self) -> bool {
        self.get_worst_cell_imbalance() > self.config.cell_voltage_difference_crit ||
            self.get_cell_voltages().into_iter().any(|cell_voltage| *cell_voltage > self.config.cell_voltage_high_crit || *cell_voltage < self.config.cell_voltage_low_crit)
    }

    pub fn battery_power_off(&self) -> bool {
        self.get_worst_cell_imbalance() > self.config.cell_votlage_difference_off ||
            self.get_cell_voltages().into_iter().any(|cell_voltage| *cell_voltage > self.config.cell_voltage_high_power_off || *cell_voltage < self.config.cell_voltage_low_power_off)
    }
}