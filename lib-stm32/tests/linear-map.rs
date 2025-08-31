#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use ateam_lib_stm32::math::linear::LinearMap;

    #[test]
    fn small_to_large() {
        let small = LinearMap::new(0f32, 10f32);
        let large = LinearMap::new(0f32, 50f32);
        let test_val: f32 = 5f32;
        let success_val: f32 = 25f32;
        defmt::assert_eq!(small.linear_map_to_new_bounds(test_val, large), success_val);
    }

    #[test]
    fn large_to_small() {
        let small = LinearMap::new(0f32, 10f32);
        let large = LinearMap::new(0f32, 50f32);
        let test_val: f32 = 25f32;
        let success_val: f32 = 5f32;
        defmt::assert_eq!(large.linear_map_to_new_bounds(test_val, small), success_val);
    }

    #[test]
    fn move_zero_point() {
        let small = LinearMap::new(0f32, 10f32);
        let large = LinearMap::new(10f32, 50f32);
        let test_val: f32 = 5f32;
        let success_val: f32 = 30f32;
        defmt::assert_eq!(small.linear_map_to_new_bounds(test_val, large), success_val);
    }
}
