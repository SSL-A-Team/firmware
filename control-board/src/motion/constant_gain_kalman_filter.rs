use nalgebra::base::SMatrix;
pub struct CgKalmanFilter<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> {
    state_transition: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    control_input: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
    observation_model: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
    process_cov: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    kalman_gain: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
    estimate_cov: SMatrix<f32, NUM_STATES, NUM_STATES>,
    state_estimate: SMatrix<f32, NUM_STATES, 1>,
    pred_state_estimate: SMatrix<f32, NUM_STATES, 1>,
    pred_estimate_cov: SMatrix<f32, NUM_STATES, NUM_STATES>,
    measurement_residual: SMatrix<f32, NUM_OBSERVATIONS, 1>
}

impl<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
    pub fn new(state_transition: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                control_input: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
                observation_model: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
                process_cov: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                kalman_gain: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
            ) -> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
        let filter = CgKalmanFilter {
            state_transition: state_transition,
            control_input: control_input,
            observation_model: observation_model,
            process_cov: process_cov,
            kalman_gain: kalman_gain,
            estimate_cov: SMatrix::<f32, NUM_STATES, NUM_STATES>::zeros(),
            state_estimate: SMatrix::<f32, NUM_STATES, 1>::zeros(),
            pred_state_estimate: SMatrix::<f32, NUM_STATES, 1>::zeros(),
            pred_estimate_cov: SMatrix::<f32, NUM_STATES, NUM_STATES>::zeros(),
            measurement_residual: SMatrix::<f32, NUM_OBSERVATIONS, 1>::zeros()
        };

        filter
    }

    pub fn predict(&mut self, u: &SMatrix<f32, NUM_CONTROL_INPUTS, 1>) {
        self.pred_state_estimate = self.state_transition * self.state_estimate + self.control_input * u;
        self.pred_estimate_cov = self.state_transition * self.estimate_cov * self.state_transition.transpose() + self.process_cov;
    }

    pub fn update(&mut self, measurement: &SMatrix<f32, NUM_OBSERVATIONS, 1>) {
        let innovation_residual = measurement - self.observation_model * self.pred_state_estimate;
        
        // Constant gain so don't need Innovation Covariance / Optimal Kalman gain calculation
        
        self.state_estimate = self.pred_state_estimate + self.kalman_gain * innovation_residual;

        self.estimate_cov = (SMatrix::<f32, NUM_STATES, NUM_STATES>::identity() -  self.kalman_gain * self.observation_model) * self.pred_estimate_cov;

        self.measurement_residual = measurement - self.observation_model * self.state_estimate;
    }

    pub fn get_state(&self) -> SMatrix<f32, NUM_STATES, 1> {
        self.state_estimate
    }

    pub fn get_measurement_residual(&self) -> SMatrix<f32, NUM_OBSERVATIONS, 1> {
        self.measurement_residual
    }
}

