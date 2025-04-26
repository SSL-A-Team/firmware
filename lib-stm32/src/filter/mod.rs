use crate::math::Number;

pub trait Filter<T>: Default {
    fn add_sample(&mut self, sample: T);

    fn update(&mut self);

    fn filtered_value(&self) -> Option<T>;

    fn reset(&mut self);
}

pub struct WindowAvergingFilter<const WINDOW_SIZE: usize, const SOFT_INIT: bool, T: Number> {
    window: [T; WINDOW_SIZE],
    update_ind: usize,
    filtered_value: T,
    initialized: bool,
}

impl<const WINDOW_SIZE: usize, const SOFT_INIT: bool, T: Number> WindowAvergingFilter<WINDOW_SIZE, SOFT_INIT, T> {
    pub fn new() -> Self {
        Self {
            window: [T::zero(); WINDOW_SIZE],
            update_ind: 0,
            filtered_value: T::zero(),
            initialized: false
        }
    }
}

impl<const WINDOW_SIZE: usize, const SOFT_INIT: bool, T: Number> Default for WindowAvergingFilter<WINDOW_SIZE, SOFT_INIT, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const WINDOW_SIZE: usize, const SOFT_INIT: bool, T: Number> Filter<T> for WindowAvergingFilter<WINDOW_SIZE, SOFT_INIT, T> {
    fn add_sample(&mut self, sample: T) {
        // if we're configured for soft init
        // set every value to the first one
        if SOFT_INIT && !self.initialized {
            for v in self.window.iter_mut() {
                *v = sample;
            }

            self.update_ind = 0;
            self.initialized = true;
        }

        self.window[self.update_ind] = sample;
        
        self.update_ind += 1;
        if self.update_ind >= WINDOW_SIZE {
            self.update_ind = 0;

            if !self.initialized {
                self.initialized = true;
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
        if !self.initialized {
            return None;
        }

        Some(self.filtered_value)
    }

    fn reset(&mut self) {
        for i in 0..WINDOW_SIZE {
            self.window[i] = T::zero();
        }

        self.update_ind = 0;
        self.initialized = false;
    }
}