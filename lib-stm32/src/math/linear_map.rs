use num_traits::{clamp_max, clamp_min};

use super::{range::Range, Number};

// Take input max and/or min (bounds), output value within new bounds
pub struct LinearMap<T>
where
    T: Number 
{
    input_range: Range<T>,
    output_range: Range<T>,
}

impl<T> LinearMap<T>
where
    T: Number 
{
    pub const fn new(input_range: Range<T>, output_range: Range<T>) -> Self {
        Self {
            input_range,
            output_range
        }
    }

    pub fn map_ranges_bounded<N: Number>(val: N, input_range: Range<N>, output_range: Range<N>) -> N {
        let clamped_val = clamp_min(clamp_max(val, input_range.max()), input_range.min());
        input_range.map_value_to_range(clamped_val, &output_range)
    }

    pub fn map(&self, val: T) -> T {
        self.input_range.map_value_to_range(val, &self.output_range)
    }

    pub fn map_boudned(&self, val: T) -> T {
        let val = clamp_min(clamp_max(val, self.input_range.max()), self.input_range.min());
        
        self.map(val)
    }
}

