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
        matrix![-28.86751346,  16.66666667, 2.71333333;
                -23.57022604, -23.57022604, 2.71333333;
                 23.57022604, -23.57022604, 2.71333333;
                 28.86751346,  16.66666667, 2.71333333;
                 0.0,          0.0,         1.0        ];
                
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
        matrix![-1.03923044e-02, -8.48528101e-03,  8.48528101e-03, 1.03923044e-02, -8.82095698e-11;
                 1.05519794e-02, -1.37518257e-02, -1.37518257e-02, 1.05519794e-02,  1.73644781e-02;
                 2.45564740e-02,  1.73640497e-02,  1.73640497e-02, 2.45564740e-02,  7.72510348e-01 ];