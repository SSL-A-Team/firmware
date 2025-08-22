use core::marker::PhantomData;

use embassy_time::{Duration, Instant};
use num_traits::{clamp, Bounded, FromPrimitive};
use smart_leds::RGB8;

pub trait LerpNumeric = Copy + Bounded + FromPrimitive;

// pub trait LerpNumeric = Copy + Bounded + FromPrimitive + ToPrimitive;
// pub trait LerpTrait = Copy + Sized + Lerp<LerpNumeric>;

// pub fn lerp<N: Copy + Bounded + FromPrimitive + ToPrimitive + CheckedAdd<Output=N> + CheckedSub<Output=N> + CheckedMul<Output=N> + CheckedDiv<Output=N>>(a: N, b: N, t: N) -> N {
// {
//     let t_pct = t.to_f32().unwrap() / N::max_value().to_f32().unwrap();
//     return N::from_f32(lerp_f(a.to_f32().unwrap(), b.to_f32().unwrap(), t_pct)).unwrap();
// }

pub fn lerp<N>(a: N, b: N, t: N) -> N
where
    N: FromPrimitive + Copy + Bounded,
    f32: core::convert::From<N>,
{
    let t_pct = Into::<f32>::into(t) / Into::<f32>::into(N::max_value());
    N::from_f32(f_lerp_f(Into::<f32>::into(a), Into::<f32>::into(b), t_pct)).unwrap()
}

pub fn lerp_f<N>(a: N, b: N, t: f32) -> N
where
    N: FromPrimitive + Copy + Bounded,
    f32: core::convert::From<N>,
{
    N::from_f32(f_lerp_f(Into::<f32>::into(a), Into::<f32>::into(b), t)).unwrap()
}

pub fn f_lerp_f(a: f32, b: f32, t: f32) -> f32 {
    let t = clamp(t, 0.0, 1.0);
    a + (b - a) * t
}

pub trait Lerp<N>: Clone + Copy
where
    N: FromPrimitive + Copy + Bounded,
    f32: core::convert::From<N>,
{
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
            r: f_lerp_f(a.r as f32, b.r as f32, t) as u8,
            g: f_lerp_f(a.g as f32, b.g as f32, t) as u8,
            b: f_lerp_f(a.b as f32, b.b as f32, t) as u8,
        }
    }
}

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
