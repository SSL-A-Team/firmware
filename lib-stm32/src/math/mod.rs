use num_traits::{FromPrimitive, ToPrimitive};
use core::ops::{Add, Div, Mul, Sub};

pub mod linear_map;
pub mod lerp;
pub mod range;

// we define numbers as ring including the additive and multiplicative inverses
// comutation on multiplication is implied (monoid). This means matrices are not
// allowed, which is enforced by FromPrimitive and ToPrimitive. This is a lose
// definition, lets see if it bites us in the rear end 
pub trait Number: Copy + Sub<Output = Self> + Add<Output = Self> + 
    Div<Output = Self> + Mul<Output = Self> + FromPrimitive + ToPrimitive {}

impl<T> Number for T where T: Copy + Sub<Output = T> + Add<Output = T> + Div<Output = T> + Mul<Output = T> + FromPrimitive + ToPrimitive {}
