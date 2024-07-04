use num_traits::{FromPrimitive, ToPrimitive};
use core::ops::{Add, Sub, Div, Mul};

pub trait MappingNumeric: Copy + Sub<Output = Self> + Add<Output = Self> + 
    Div<Output = Self> + Mul<Output = Self> + FromPrimitive + ToPrimitive {}

impl<T> MappingNumeric for T where T: Copy + Sub<Output = T> + Add<Output = T> + Div<Output = T> + Mul<Output = T> + FromPrimitive + ToPrimitive {}

// Take input max and/or min (bounds), output value within new bounds
struct LinearMap<T>
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
    fn new(min: T, max: T) -> Self {
        LinearMap { min, max }
    }

    fn linear_map_to_new_bounds(&self, val: T, new_bounds: LinearMap<T>) -> T {
        let min_diff = self.min + new_bounds.min;
        let scale = (new_bounds.max - new_bounds.min) / (self.max - self.min);
        let mapped_val = (val - min_diff) * scale;
        return mapped_val;
    }
}

#[test]
fn small_to_large() {
    let small = LinearMap::new(0f32, 10f32);
    let large = LinearMap::new(0f32, 50f32);
    let test_val: f32 = 5f32;
    let success_val: f32 = 25f32;
    assert_eq!(small.linear_map_to_new_bounds(test_val, large), success_val);
}

#[test]
fn large_to_small() {
    let small = LinearMap::new(0f32, 10f32);
    let large = LinearMap::new(0f32, 50f32);
    let test_val: f32 = 25f32;
    let success_val: f32 = 5f32;
    assert_eq!(large.linear_map_to_new_bounds(test_val, small), success_val);
}

#[test]
fn move_zero_point() {
    let small = LinearMap::new(0f32, 10f32);
    let large = LinearMap::new(10f32, 50f32);
    let test_val: f32 = 5f32;
    let success_val: f32 = 30f32;
    assert_eq!(small.linear_map_to_new_bounds(test_val, large), success_val);
}
