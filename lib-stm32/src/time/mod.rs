use embassy_time::{Duration, Instant};

pub struct SyncTicker {
    duration: Duration,
    ready_at: Instant,
}

impl SyncTicker {
    pub fn every(duration: Duration) -> Self {
        Self {
            duration: duration,
            ready_at: Instant::now() + duration,
        }
    }

    pub fn reset(&mut self) {
        self.ready_at = Instant::now() + self.duration;
    }

    pub fn next(&mut self) -> bool {
        let cur_time = Instant::now();
        if cur_time >= self.ready_at {
            self.ready_at = cur_time + self.duration;
            true
        } else {
            false
        }
    }
}

pub struct Limiter<T> {
    prev_val: Option<T>,
}

impl<T: PartialEq> Limiter<T> {
    pub const fn new() -> Self {
        Limiter {
            prev_val: None
        }
    }

    pub const fn new_with_value(val: T) -> Self {
        Limiter {
            prev_val: Some(val)
        }
    }

    pub fn is_different(&mut self, val: T) -> bool {
        let res = self.prev_val.as_ref().is_none_or(|f| *f != val);
        self.prev_val = Some(val);
        res
    }

    pub fn is_same(&mut self, val: T) -> bool {
        let res = self.prev_val.as_ref().is_some_and(|f| *f == val);
        self.prev_val = Some(val);
        res
    }
}