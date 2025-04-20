use crate::math::Number;

pub trait Filter<T>: Default {
    fn add_sample(&mut self, sample: T);

    fn update(&mut self);

    fn filtered_value(&self) -> Option<T>;

    fn reset(&mut self);
}

pub struct WindowAvergingFilter<const WINDOW_SIZE: usize, T: Number> {
    window: [T; WINDOW_SIZE],
    update_ind: usize,
    filtered_value: T,
    valid: bool,
}

impl<const WINDOW_SIZE: usize, T: Number> WindowAvergingFilter<WINDOW_SIZE, T> {
    pub fn new() -> Self {
        Self {
            window: [T::zero(); WINDOW_SIZE],
            update_ind: 0,
            filtered_value: T::zero(),
            valid: false
        }
    }
}

impl<const WINDOW_SIZE: usize, T: Number> Default for WindowAvergingFilter<WINDOW_SIZE, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const WINDOW_SIZE: usize, T: Number> Filter<T> for WindowAvergingFilter<WINDOW_SIZE, T> {
    fn add_sample(&mut self, sample: T) {
        self.window[self.update_ind] = sample;
        
        self.update_ind += 1;
        if self.update_ind >= WINDOW_SIZE {
            self.update_ind = 0;

            if !self.valid {
                self.valid = true;
            }
        }
    }

    fn update(&mut self) {
        let mut sum = T::zero();
        for val in self.window {
            sum = sum + val;
        }

        self.filtered_value = if let Some(divisor) = T::from_usize(WINDOW_SIZE) {
            sum / divisor
        } else {
            defmt::warn!("type conversion failed in WindowAveragingFilter");
            T::zero()
        }
    }

    fn filtered_value(&self) -> Option<T> {
        if !self.valid {
            return None;
        }

        Some(self.filtered_value)
    }

    fn reset(&mut self) {
        for i in 0..WINDOW_SIZE {
            self.window[i] = T::zero();
        }

        self.update_ind = 0;
        self.valid = false;
    }
}