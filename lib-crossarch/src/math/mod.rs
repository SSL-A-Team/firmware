use num_traits::{FromPrimitive, Num, ToPrimitive};

pub mod lerp;
pub mod linear_map;
pub mod range;

// We define numbers as a ring including the additive and multiplicative inverses.
// Commutativity on multiplication is implied (monoid). This means matrices are not
// allowed, which is enforced by FromPrimitive and ToPrimitive.
#[cfg(feature = "defmt")]
pub trait Number:
    Copy + Num + PartialOrd + PartialEq + FromPrimitive + ToPrimitive + defmt::Format
{
}

#[cfg(not(feature = "defmt"))]
pub trait Number: Copy + Num + PartialOrd + PartialEq + FromPrimitive + ToPrimitive {}

#[cfg(feature = "defmt")]
impl<T> Number for T where
    T: Copy + Num + PartialOrd + PartialEq + FromPrimitive + ToPrimitive + defmt::Format
{
}

#[cfg(not(feature = "defmt"))]
impl<T> Number for T where
    T: Copy + Num + PartialOrd + PartialEq + FromPrimitive + ToPrimitive
{
}
