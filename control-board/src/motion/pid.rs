use nalgebra::{
    base::{SMatrix, SVector},
    clamp,
};

pub struct PidController<const NUM_STATES: usize> {
    gain: SMatrix<f32, NUM_STATES, 5>,
    anti_jitter_thresh: Option<SVector<f32, NUM_STATES>>,
    prev_error: SVector<f32, NUM_STATES>,
    integrated_error: SVector<f32, NUM_STATES>,
}

impl<'a, const NUM_STATES: usize> PidController<NUM_STATES> {
    pub fn from_gains_matrix(gain: &'a SMatrix<f32, NUM_STATES, 5>) -> PidController<NUM_STATES> {
        PidController {
            gain: gain.clone(),
            anti_jitter_thresh: None,
            prev_error: SVector::<f32, NUM_STATES>::zeros(),
            integrated_error: SVector::<f32, NUM_STATES>::zeros(),
        }
    }

    pub fn from_gains_matrix_with_anti_jitter(
        gain: &'a SMatrix<f32, NUM_STATES, 5>,
        anti_jitter_thresh: Option<SVector<f32, NUM_STATES>>,
    ) -> PidController<NUM_STATES> {
        PidController {
            gain: gain.clone(),
            anti_jitter_thresh,
            prev_error: SVector::<f32, NUM_STATES>::zeros(),
            integrated_error: SVector::<f32, NUM_STATES>::zeros(),
        }
    }

    pub fn set_anti_jitter_thresh(&mut self, thresh: Option<SVector<f32, NUM_STATES>>) {
        self.anti_jitter_thresh = thresh;
    }

    pub fn calculate(
        &mut self,
        setpoint: &SVector<f32, NUM_STATES>,
        process_variable: &SVector<f32, NUM_STATES>,
        dt_s: f32,
    ) -> SVector<f32, NUM_STATES> {
        let error = setpoint - process_variable;

        // Calculate integrated error.
        let cur_integrated_error = self.integrated_error + (error * dt_s);
        // clamp error
        // is there a better way to do this?
        self.integrated_error = cur_integrated_error.zip_zip_map(
            &self.gain.column(3),
            &self.gain.column(4),
            |err, min_err, max_err| clamp(err, min_err, max_err),
        );

        // calculate derivative error
        let de_dt = (error - self.prev_error) / dt_s;
        self.prev_error = error;

        let p = self.gain.column(0).component_mul(&error);
        let i = self.gain.column(1).component_mul(&self.integrated_error);
        let d = self.gain.column(2).component_mul(&de_dt);

        self.apply_anti_jitter(p + i + d, &error)
    }

    /// Like `calculate`, but uses an externally provided derivative signal
    /// for the D term instead of numerically differentiating the error.
    pub fn calculate_with_derivative(
        &mut self,
        setpoint: &SVector<f32, NUM_STATES>,
        process_variable: &SVector<f32, NUM_STATES>,
        derivative: &SVector<f32, NUM_STATES>,
        dt_s: f32,
    ) -> SVector<f32, NUM_STATES> {
        let error = setpoint - process_variable;

        let cur_integrated_error = self.integrated_error + (error * dt_s);
        self.integrated_error = cur_integrated_error.zip_zip_map(
            &self.gain.column(3),
            &self.gain.column(4),
            |err, min_err, max_err| clamp(err, min_err, max_err),
        );

        self.prev_error = error;

        let p = self.gain.column(0).component_mul(&error);
        let i = self.gain.column(1).component_mul(&self.integrated_error);
        let d = self.gain.column(2).component_mul(derivative);

        self.apply_anti_jitter(p + i + d, &error)
    }

    /// Scales output to zero when per-axis |error| is below the threshold, matching
    /// the fixedpoint PI anti-jitter behavior. No-op when `anti_jitter_thresh` is None.
    fn apply_anti_jitter(
        &self,
        output: SVector<f32, NUM_STATES>,
        error: &SVector<f32, NUM_STATES>,
    ) -> SVector<f32, NUM_STATES> {
        if let Some(thresh) = &self.anti_jitter_thresh {
            output.zip_zip_map(error, thresh, |out, err, t| {
                if t > 0.0 && err.abs() < t {
                    out * err.abs() / t
                } else {
                    out
                }
            })
        } else {
            output
        }
    }

    pub fn get_gain(&self) -> SMatrix<f32, NUM_STATES, 5> {
        return self.gain;
    }

    pub fn set_gain(&mut self, new_gain: SMatrix<f32, NUM_STATES, 5>) {
        self.gain.copy_from(&new_gain)
    }

    pub fn reset(&mut self) {
        self.prev_error = SVector::<f32, NUM_STATES>::zeros();
        self.integrated_error = SVector::<f32, NUM_STATES>::zeros();
    }
}
