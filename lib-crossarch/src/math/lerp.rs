use num_traits::{clamp, Bounded, FromPrimitive};

pub trait LerpNumeric = Copy + Bounded + FromPrimitive;

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

#[cfg(feature = "smart-leds")]
impl Lerp<u8> for smart_leds::RGB8 {
    fn lerp(a: Self, b: Self, t: u8) -> Self {
        smart_leds::RGB8 {
            r: lerp(a.r, b.r, t),
            g: lerp(a.g, b.g, t),
            b: lerp(a.b, b.b, t),
        }
    }

    fn lerp_f(a: Self, b: Self, t: f32) -> Self {
        smart_leds::RGB8 {
            r: f_lerp_f(a.r as f32, b.r as f32, t) as u8,
            g: f_lerp_f(a.g as f32, b.g as f32, t) as u8,
            b: f_lerp_f(a.b as f32, b.b as f32, t) as u8,
        }
    }
}
