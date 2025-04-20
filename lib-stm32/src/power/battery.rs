
use core::iter::zip;

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

    pub fn add_cell_voltage_samples(&mut self, cell_adc_voltage_samples: &[N; NUM_CELLS]) {
        let mut raw_cell_voltages = [N::zero(); NUM_CELLS];

        // this is just a condensed way of mapping an array of adc samples in mv
        // through a filter to cell voltages 
        for ((cell_voltage, cell_voltage_filter), (sample_map, sample)) in 
                zip(raw_cell_voltages.iter_mut().zip(self.cell_votlage_filters.iter_mut()),
                    self.cell_range_maps.into_iter().zip(cell_adc_voltage_samples)) {
            cell_voltage_filter.add_sample(sample_map.map(*sample));
            cell_voltage_filter.update();
            if let Some(filtered_value) = cell_voltage_filter.filtered_value() {
                *cell_voltage = filtered_value;
            }
        }

        if self.cell_voltage_compute_mode == CellVoltageComputeMode::Standalone {
            self.cell_voltages.copy_from_slice(&raw_cell_voltages);
        } else {
            self.cell_voltages[0] = raw_cell_voltages[0];
            for i in 1..NUM_CELLS {
                self.cell_voltages[i] = raw_cell_voltages[i] - raw_cell_voltages[i - 1];
            }
        }
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