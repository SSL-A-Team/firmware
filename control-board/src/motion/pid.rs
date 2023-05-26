use nalgebra::{
    base::{SMatrix, SVector},
    clamp
};

pub struct PidController<'a, const NUM_STATES: usize> {
    K: &'a SMatrix<f32, NUM_STATES, 5>,
    u: SVector<f32, NUM_STATES>,
    prev_error: SVector<f32, NUM_STATES>,
    integrated_error: SVector<f32, NUM_STATES>,
}

impl<'a, const NUM_STATES: usize> PidController<'a, NUM_STATES> {
    pub fn from_gains_matrix(K: &'a SMatrix<f32, NUM_STATES, 5>) -> PidController<'a, NUM_STATES> {
        PidController {
            K: K, 
            u: SVector::<f32, NUM_STATES>::zeros(),
            prev_error: SVector::<f32, NUM_STATES>::zeros(),
            integrated_error: SVector::<f32, NUM_STATES>::zeros(),
        }
    }

    pub fn calculate(&mut self, r: SVector<f32, NUM_STATES>, y: SVector<f32, NUM_STATES>, dt: f32) {
        let error = r - y;

        // calculate integrated error
        let ie = self.integrated_error + error;
        // clamp error
        // is there a better way to do this? 
        self.integrated_error = ie.zip_zip_map(&self.K.column(3), &self.K.column(4), |err, min_err, max_err| clamp(err, min_err, max_err));

        // calculate derivative error
        let de_dt = error - self.prev_error;
        self.prev_error = error;

        let p = self.K.column(0).component_mul(&error);
        let i = self.K.column(1).component_mul(&self.integrated_error);
        let d = self.K.column(2).component_mul(&de_dt);
        self.u = r + (p + i + d);
    }

    pub fn read_u(&self, u: &mut SVector<f32, NUM_STATES>) {
        *u = self.u;
    }

    pub fn get_u(&self) -> SVector<f32, NUM_STATES> {
        self.u
    }
}