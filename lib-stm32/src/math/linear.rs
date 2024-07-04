use num_traits::{FromPrimitive, ToPrimitive};
use core::ops::{Add, Sub, Div, Mul};

pub trait BoundNumeric: Copy + Sub<Output = Self> + Add<Output = Self> + 
    Div<Output = Self> + Mul<Output = Self> + FromPrimitive + ToPrimitive {}

// Take input max and/or min (bounds), output value within new bounds
struct Bounds<T>
where
    T: BoundNumeric 
{
    min: T,
    max: T,
}

impl<T> Bounds<T>
where
    T: BoundNumeric 
{
    fn new(min: T, max: T) -> Self {
        Bounds { min, max }
    }

    fn linear_map_to_new_bounds(&self, val: T, new_bounds: Bounds<T>) -> T {
        let min_diff = self.min + new_bounds.min;
        let scale = (new_bounds.max - new_bounds.min) / (self.max - self.min);
        let mapped_val = (val - min_diff) * scale;
        return mapped_val;
    }
}