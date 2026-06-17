pub use ateam_lib_crossarch::math::linear_map;
pub use ateam_lib_crossarch::math::range;
pub use ateam_lib_crossarch::math::Number;

// lerp is a local module: re-exports the portable crossarch lerp items and
// adds the embassy_time-dependent TimeLerp and the smart_leds RGB8 impl.
pub mod lerp;
