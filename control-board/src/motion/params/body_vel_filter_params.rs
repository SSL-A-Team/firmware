use nalgebra::{matrix, Matrix3, Matrix3x4, Matrix3x5, Matrix5, Matrix5x3};

pub const KF_NUM_STATES: usize = 3;
pub const KF_NUM_CONTROL_INPUTS: usize = 4;
pub const KF_NUM_OBSERVATIONS: usize = 5;

const EXPECTED_DT: f32 = 10000.0;  // 10mS = 10000uS
const EXPECTED_DT_2: f32 = EXPECTED_DT * EXPECTED_DT;
const ENCODER_NOISE: f32 = 0.11;  // noise in rad / sampling time 
const GYRO_NOISE: f32 = 0.002;  // BMI085 compensated for spectral noise at 100Hz sample rate
const PROCESS_NOISE: f32 = 0.05;
const INITIAL_COV: f32 = 0.11;


// Assume constant velocity as a valid linearization of the transition system
pub static STATE_TRANSITION: Matrix3<f32> = 
        matrix![1.0, 0.0, 0.0;
                0.0, 1.0, 0.0;
                0.0, 0.0, 1.0];

// Assume no input for the moment
pub static CONTROL_INPUT: Matrix3x4<f32> = 
        matrix![0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0];

pub static OBSERVATION_MODEL: Matrix5x3<f32> = 
        matrix![-20.2429, 35.0618, 3.29555;
                -28.6278, -28.6278, 3.29555; 
                28.6278, -28.6278, 3.29555;
                20.2429, 35.0618, 3.29555; 
                0.0, 0.0, 1.0];
                
pub static PROCESS_COV: Matrix3<f32> =
        matrix![KF_NUM_STATES as f32 * PROCESS_NOISE / EXPECTED_DT_2, 0.0, 0.0;
                0.0, KF_NUM_STATES as f32 * PROCESS_NOISE / EXPECTED_DT_2, 0.0;
                0.0, 0.0, KF_NUM_STATES as f32 * PROCESS_NOISE / EXPECTED_DT_2];

pub static OBSERVATION_COV: Matrix5<f32> = 
        matrix![ENCODER_NOISE, 0.0,          0.0,          0.0,          0.0;
                0.0,          ENCODER_NOISE, 0.0,          0.0,          0.0;
                0.0,          0.0,          ENCODER_NOISE, 0.0,          0.0;
                0.0,          0.0,          0.0,          ENCODER_NOISE, 0.0;
                0.0,          0.0,          0.0,          0.0,          GYRO_NOISE];

pub static INIT_ESTIMATE_COV: Matrix3<f32> =
        matrix![INITIAL_COV, 0.0,         0.0;
                0.0,         INITIAL_COV, 0.0;
                0.0,         0.0,         INITIAL_COV];

pub static KALMAN_GAIN: Matrix3x5<f32> = 
        matrix![-0.006554, -0.009269, 0.009269, 0.0065542, 0.0;
                0.007210, -0.006147, -0.006147,  0.007214, 0.0;
                0.01268,  0.015415,  0.015415, 0.01268, 0.04181];

