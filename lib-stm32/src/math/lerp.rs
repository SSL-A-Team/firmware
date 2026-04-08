use core::marker::PhantomData;

use embassy_time::{Duration, Instant};
use num_traits::{Bounded, FromPrimitive};

pub use ateam_lib_crossarch::math::lerp::*;

pub struct TimeLerp<'a, N, L: Lerp<N>>
where
    N: FromPrimitive + Copy + Bounded,
    f32: core::convert::From<N>,
{
    a: L,
    b: L,

    duration: Duration,
    start_time: Instant,

    pd2: PhantomData<&'a N>,
}

impl<'a, N, L> TimeLerp<'a, N, L>
where
    N: FromPrimitive + Copy + Bounded,
    f32: core::convert::From<N>,
    L: Lerp<N>,
{
    pub const fn new(a: L, b: L, duration: Duration) -> TimeLerp<'a, N, L> {
        TimeLerp {
            a,
            b,
            duration,
            start_time: Instant::MIN,
            pd2: PhantomData,
        }
    }

    pub fn start(&mut self) {
        self.start_time = Instant::now();
    }

    pub fn update(&self) -> L {
        let now = Instant::now();
        let elapsed_time = (now - self.start_time).as_millis();
        let pct_complete = elapsed_time as f32 / self.duration.as_millis() as f32;
        L::lerp_f(self.a, self.b, pct_complete)
    }
}
