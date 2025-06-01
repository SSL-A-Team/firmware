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