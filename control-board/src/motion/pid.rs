use nalgebra::{
    base::{SMatrix, SVector},
    clamp
};

pub struct PidController<const NUM_STATES: usize> {
    gain: SMatrix<f32, NUM_STATES, 5>,
    prev_error: SVector<f32, NUM_STATES>,
    integrated_error: SVector<f32, NUM_STATES>,
}

impl<'a, const NUM_STATES: usize> PidController<NUM_STATES> {
    pub fn from_gains_matrix(gain: &'a SMatrix<f32, NUM_STATES, 5>) -> PidController<NUM_STATES> {
        PidController {
            gain: gain.clone(), 
            prev_error: SVector::<f32, NUM_STATES>::zeros(),
            integrated_error: SVector::<f32, NUM_STATES>::zeros(),
        }
    }

    pub fn calculate(&mut self, setpoint: &SVector<f32, NUM_STATES>, process_variable: &SVector<f32, NUM_STATES>, _dt: f32) -> SVector<f32, NUM_STATES> {
        let error = setpoint - process_variable;

        // Calculate integrated error.
        let cur_integrated_error = self.integrated_error + error;
        // clamp error
        // is there a better way to do this? 
        self.integrated_error = cur_integrated_error.zip_zip_map(&self.gain.column(3), &self.gain.column(4), |err, min_err, max_err| clamp(err, min_err, max_err));

        // calculate derivative error
        let de_dt = error - self.prev_error;
        self.prev_error = error;

        let p = self.gain.column(0).component_mul(&error);
        let i = self.gain.column(1).component_mul(&self.integrated_error);
        let d = self.gain.column(2).component_mul(&de_dt);
        setpoint + (p + i + d)
    }

    pub fn get_gain(&self) -> SMatrix<f32, NUM_STATES, 5> {
        return self.gain;
    }

    pub fn set_gain(&mut self, new_gain: SMatrix<f32, NUM_STATES, 5>) {
        self.gain.copy_from(&new_gain)
    }
}