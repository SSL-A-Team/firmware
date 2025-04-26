
use core::{cell, iter::zip};

use crate::{filter::Filter, math::{linear_map::LinearMap, Number}};


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

pub struct LipoModel<'a, const NUM_CELLS: usize, N: Number, F: Filter<N>> {
    config: BatteryConfig<N>,
    cell_range_maps: &'a [LinearMap<N>; NUM_CELLS],
    cell_voltage_compute_mode: CellVoltageComputeMode,
    cell_votlage_filters: [F; NUM_CELLS],
    cell_voltages: [N; NUM_CELLS],
    cell_percentages: [u8; NUM_CELLS],
}

impl<'a, const NUM_CELLS: usize, N: Number, F: Filter<N>> LipoModel<'a, NUM_CELLS, N, F> {
    pub fn new(config: BatteryConfig<N>, cell_range_maps: &'a[LinearMap<N>; NUM_CELLS], cell_voltage_compute_mode: CellVoltageComputeMode) -> Self {
        Self {
            config,
            cell_range_maps,
            cell_voltage_compute_mode,
            cell_votlage_filters: core::array::from_fn(|_| F::default()),
            cell_voltages: [N::zero(); NUM_CELLS],
            cell_percentages: [0; NUM_CELLS],
        }
    }

    pub fn add_cell_voltage_samples(&mut self, cell_adc_voltage_samples: &[N]) {

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
            *cv = cv_filt.filtered_value().unwrap_or(N::zero());
        }

        if self.cell_voltage_compute_mode == CellVoltageComputeMode::Chained {
            for i in (1..NUM_CELLS).rev() {
                self.cell_voltages[i] = self.cell_voltages[i] - self.cell_voltages[i - 1];
            }
        }
    }

    pub fn get_cell_voltages(&self) -> &[N; NUM_CELLS] {
        &self.cell_voltages
    }

    pub fn battery_warn(&self) -> bool {
        false
    }

    pub fn battery_crit(&self) -> bool {
        false
    }

    pub fn battery_power_off(&self) -> bool {
        false
    }
}