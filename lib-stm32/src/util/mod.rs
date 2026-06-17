use embassy_time::{Duration, Instant};

/// Time-based rate limiter gate. Tracks last-allowed timestamp; caller decides
/// what action to take when `is_allowed` returns true.
///
/// Works for any call site — pubsub publishes, defmt prints, or anything else.
///
/// First call after construction always returns `true`.
pub struct RateLimiter {
    min_interval: Duration,
    last_allowed: Instant,
}

impl RateLimiter {
    pub fn new(min_interval: Duration) -> Self {
        Self {
            min_interval,
            last_allowed: Instant::from_ticks(0),
        }
    }

    /// Returns `true` and records current time if `min_interval` has elapsed
    /// since last allowed call. Returns `false` otherwise.
    pub fn is_allowed(&mut self) -> bool {
        let now = Instant::now();
        if now - self.last_allowed >= self.min_interval {
            self.last_allowed = now;
            true
        } else {
            false
        }
    }

    /// Forces next `is_allowed` call to return `true` regardless of interval.
    pub fn reset(&mut self) {
        self.last_allowed = Instant::from_ticks(0);
    }

    pub fn min_interval(&self) -> Duration {
        self.min_interval
    }

    pub fn set_min_interval(&mut self, interval: Duration) {
        self.min_interval = interval;
    }
}
