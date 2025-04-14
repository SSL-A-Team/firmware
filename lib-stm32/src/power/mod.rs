use crate::{filter::Filter, math::{range::Range, Number}};

pub mod battery;
pub struct PowerRailParameters<T: Number> {
    pub min_value_crit: T,
    pub min_value_warn: T,
    pub max_value_warn: T,
    pub max_value_crit: T,
}

pub struct PowerRail<T: Number, F: Filter<T>>  {
    rail_parameters: PowerRailParameters<T>,
    filter: F,
    sample_range: Range<T>,
    rail_range: Range<T>,
}

impl<T: Number, F: Filter<T>> PowerRail<T, F> {
    pub fn new(rail_parameters: PowerRailParameters<T>, filter: F, sample_range: Range<T>, rail_range: Range<T>) -> Self {
        Self {
            rail_parameters,
            filter,
            sample_range,
            rail_range
        }
    }

    pub fn add_rail_voltage_sample(&mut self, sample: T) {
        let sample_rail_domain = self.sample_range.map_value_to_range(sample, &self.rail_range);
        self.filter.add_sample(sample_rail_domain);
        self.filter.update();
    }

    pub fn get_rail_voltage(&self) -> Option<T> {
        self.filter.filtered_value()
    }

    pub fn warning_flagged(&self) -> bool {
        if self.critical_warning_flagged() {
            return true;
        }

        if let Some(rail_voltage) = self.get_rail_voltage() {
            rail_voltage > self.rail_parameters.max_value_warn
                || rail_voltage < self.rail_parameters.min_value_warn
        } else {
            false
        }
    }

    pub fn critical_warning_flagged(&self) -> bool {
        if let Some(rail_voltage) = self.get_rail_voltage() {
            rail_voltage > self.rail_parameters.max_value_crit
                || rail_voltage < self.rail_parameters.min_value_crit
        } else {
            false
        }
    }
}