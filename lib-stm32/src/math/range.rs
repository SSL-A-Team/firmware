use super::Number;

// Take input max and/or min (bounds), output value within new bounds
pub struct Range<T>
where
    T: Number 
{
    min: T,
    max: T,
}

impl<T> Range<T>
where
    T: Number
{
    pub fn new(min: T, max: T) -> Self {
        Range { min, max }
    }

    pub fn map_value_to_range(&self, val: T, new_range: &Range<T>) -> T {
        let scale = (new_range.max - new_range.min) / (self.max - self.min);
        
        (val - self.min) * scale + new_range.min
    }
}

