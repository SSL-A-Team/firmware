use num_traits::{FromPrimitive, ToPrimitive};
use core::ops::{Add, Sub, Div, Mul};

pub trait MappingNumeric: Copy + Sub<Output = Self> + Add<Output = Self> + 
    Div<Output = Self> + Mul<Output = Self> + FromPrimitive + ToPrimitive {}

impl<T> MappingNumeric for T where T: Copy + Sub<Output = T> + Add<Output = T> + Div<Output = T> + Mul<Output = T> + FromPrimitive + ToPrimitive {}

// Take input max and/or min (bounds), output value within new bounds
pub struct LinearMap<T>
where
    T: MappingNumeric 
{
    min: T,
    max: T,
}

impl<T> LinearMap<T>
where
    T: MappingNumeric 
{
    pub fn new(min: T, max: T) -> Self {
        LinearMap { min, max }
    }

    pub fn linear_map_to_new_bounds(&self, val: T, new_bounds: LinearMap<T>) -> T {
        let min_diff = self.min + new_bounds.min;
        let scale = (new_bounds.max - new_bounds.min) / (self.max - self.min);
        let mapped_val = (val - min_diff) * scale;
        return mapped_val;
    }
}

