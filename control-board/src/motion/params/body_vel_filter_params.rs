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
pub static F: Matrix3<f32> = 
        matrix![1.0, 0.0, 0.0;
                0.0, 1.0, 0.0;
                0.0, 0.0, 1.0];

// Assume no input for the moment
pub static B: Matrix3x4<f32> = 
        matrix![0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0];

pub static H: Matrix5x3<f32> = 
        matrix![0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0;
                0.0, 0.0, 0.0];

pub static Q: Matrix3<f32> =
        matrix![KfNumStates as f32 * ProcessNoise / ExpectedDt2, 0.0,                                           0.0;
                0.0,                                           KfNumStates as f32 * ProcessNoise / ExpectedDt2, 0.0;
                0.0,                                           0.0,                                           KfNumStates as f32 * ProcessNoise / ExpectedDt2];

pub static R: Matrix5<f32> = 
        matrix![EncoderNoise, 0.0,          0.0,          0.0,          0.0;
                0.0,          EncoderNoise, 0.0,          0.0,          0.0;
                0.0,          0.0,          EncoderNoise, 0.0,          0.0;
                0.0,          0.0,          0.0,          EncoderNoise, 0.0;
                0.0,          0.0,          0.0,          0.0,          GyroNoise];

pub static P: Matrix3<f32> = 
        matrix![InitialCovariance, 0.0,               0.0;
                0.0,               InitialCovariance, 0.0;
                0.0,               0.0,               InitialCovariance];

pub static K: Matrix3x5<f32> = 
        matrix![0.0, 0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0, 0.0;
                0.0, 0.0, 0.0, 0.0, 0.0];

