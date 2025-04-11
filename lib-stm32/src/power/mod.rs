use crate::math::Number;

pub struct PowerRailParameters<T: Number> {
    pub min_value_crit: T,
    pub min_value_warn: T,
    pub max_value_warn: T,
    pub max_value_crit: T,
}

pub struct PowerRail<T: Number>  {
    parameters: PowerRailParameters<T>,
}

impl<T: Number> PowerRail<T> {
    const fn new(power_rail_params: PowerRailParameters<T>, ) {

    }
}