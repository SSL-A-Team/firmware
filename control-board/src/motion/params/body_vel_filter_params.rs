use nalgebra::{matrix, Matrix3, Matrix3x4, Matrix3x5, Matrix5, Matrix5x3};

pub const KfNumStates: usize = 3;
pub const KfNumControlInputs: usize = 4;
pub const KfNumObservations: usize = 5;

const ExpectedDt: f32 = 10000.0;  // 10mS = 10000uS
const ExpectedDt2: f32 = ExpectedDt * ExpectedDt;
const EncoderNoise: f32 = 0.11;  // noise in rad / sampling time 
const GyroNoise: f32 = 0.002;  // BMI085 compensated for spectral noise at 100Hz sample rate
const ProcessNoise: f32 = 0.05;
const InitialCovariance: f32 = 0.11;

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
        matrix![-20.24291498, 35.06175724, -3.23076923;
                -28.62780491, -28.62780491, -3.38866397;
                28.62780491, -28.62780491, -3.38866397;
                20.24291498, 35.06175724, -3.23076923;
                0.0, 0.0, 1.0];
                
pub static PROCESS_COV: Matrix3<f32> =
        matrix![KfNumStates as f32 * ProcessNoise / ExpectedDt2, 0.0,                                           0.0;
                0.0,                                           KfNumStates as f32 * ProcessNoise / ExpectedDt2, 0.0;
                0.0,                                           0.0,                                           KfNumStates as f32 * ProcessNoise / ExpectedDt2];

pub static OBSERVATION_COV: Matrix5<f32> = 
        matrix![EncoderNoise, 0.0,          0.0,          0.0,          0.0;
                0.0,          EncoderNoise, 0.0,          0.0,          0.0;
                0.0,          0.0,          EncoderNoise, 0.0,          0.0;
                0.0,          0.0,          0.0,          EncoderNoise, 0.0;
                0.0,          0.0,          0.0,          0.0,          GyroNoise];

pub static KALMAN_GAIN: Matrix3x5<f32> = 
        matrix![0.008483887, -0.008483887, -0.006927967, 0.006928444, 0.0;
                0.009064674, 0.009062767, -0.01090765, -0.010907173, 0.012550354;
                0.038414717, 0.038418293, 0.027166963, 0.027170658, 0.53518677];

