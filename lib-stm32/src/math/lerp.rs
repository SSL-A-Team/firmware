use core::marker::PhantomData;

use embassy_time::{Duration, Instant};
use num_traits::{clamp, Bounded, CheckedAdd, CheckedDiv, CheckedMul, CheckedSub, FromPrimitive, ToPrimitive};
use smart_leds::RGB8;

pub fn lerp<N: Copy + Bounded + FromPrimitive + ToPrimitive + CheckedAdd<Output=N> + CheckedSub<Output=N> + CheckedMul<Output=N> + CheckedDiv<Output=N>>(a: N, b: N, t: N) -> N {
    let t_pct = t.to_f32().unwrap() / N::max_value().to_f32().unwrap();
    return N::from_f32(lerp_f(a.to_f32().unwrap(), b.to_f32().unwrap(), t_pct)).unwrap();
}

pub fn lerp_f(a: f32, b: f32, t: f32) -> f32 {
    let t = clamp(t, 0.0, 1.0);
    return a + (b - a) * t;
}

pub trait Lerp<N: Copy + Bounded + CheckedAdd<Output=N> + CheckedSub<Output=N> + CheckedMul<Output=N> + CheckedDiv<Output=N>> {
    fn lerp(a: Self, b: Self, t: N) -> Self;
    fn lerp_f(a: Self, b: Self, t: f32) -> Self;
}

impl Lerp<u8> for RGB8 {
    fn lerp(a: RGB8, b: RGB8, t: u8) -> RGB8 {
        RGB8 {
            r: lerp(a.r, b.r, t),
            g: lerp(a.g, b.g, t),
            b: lerp(a.b, b.b, t),
        }
    }

    fn lerp_f(a: RGB8, b: RGB8, t: f32) -> RGB8 {
        RGB8 {
            r: lerp_f(a.r as f32, b.r as f32, t) as u8,
            g: lerp_f(a.g as f32, b.g as f32, t) as u8,
            b: lerp_f(a.b as f32, b.b as f32, t) as u8,
        }
    }
}

pub struct TimeLerp<'a, N: Copy + Bounded + CheckedAdd<Output=N> + CheckedSub<Output=N> + CheckedMul<Output=N> + CheckedDiv<Output=N>, L: Copy + Sized + Lerp<N>> {
    a: L,
    b: L,

    duration: Duration,
    start_time: Instant,

    pd2: PhantomData<&'a N>,
}

impl<'a, N: Copy + Bounded + CheckedAdd<Output=N> + CheckedSub<Output=N> + CheckedMul<Output=N> + CheckedDiv<Output=N>, L: Copy + Sized + Lerp<N>> TimeLerp<'a, N, L> {
    pub fn new(a: L, b: L, duration: Duration) -> TimeLerp<'a, N, L> {
        TimeLerp {
            a: a,
            b: b,
            duration: duration,
            start_time: Instant::now(),
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

#[test]
fn test_lerp() {
    print("hi")
}