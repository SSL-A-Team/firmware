use nalgebra::Vector4;
pub const WHEEL_ANGLES_DEG: Vector4<f32> = Vector4::new(300.0, 45.0, 135.0, 240.0); // Degrees from y+
pub const WHEEL_RADIUS_M: f32 = 0.0247; // wheel dia 49mm
pub const WHEEL_DISTANCE_TO_ROBOT_CENTER_M: f32 = 0.0814; // from center of wheel body to center of robot