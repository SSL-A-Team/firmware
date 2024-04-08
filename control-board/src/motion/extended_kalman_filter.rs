use nalgebra::base::SMatrix;
pub struct CgKalmanFilter<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> {
    state_transition: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    control_input: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
    observation_model: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
    process_covar: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    observation_covar: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_OBSERVATIONS>,
    covar_estimate: SMatrix<f32, NUM_STATES, NUM_STATES>,
    kalman_gain: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
    state_estimate: SMatrix<f32, NUM_STATES, 1>,
}

impl<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
    pub fn new(state_transition: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                control_input: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
                observation_model: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
                process_covar: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                observation_covar: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_OBSERVATIONS>,
                covar_estimate: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                kalman_gain: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
                // TODO: ? accept starting state? 
            ) -> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
        let mut filter = CgKalmanFilter {
            state_transition: state_transition,
            control_input: control_input,
            observation_model: observation_model,
            process_covar: process_covar,
            observation_covar: observation_covar,
            covar_estimate: SMatrix::<f32, NUM_STATES, NUM_STATES>::zeros(),
            kalman_gain: kalman_gain,
            state_estimate: SMatrix::<f32, NUM_STATES, 1>::zeros(),
        };

        filter.covar_estimate.copy_from(covar_estimate);

        filter
    }

    pub fn predict(&mut self, u: &SMatrix<f32, NUM_CONTROL_INPUTS, 1>) {
        self.state_estimate = self.state_transition * self.state_estimate + self.control_input * u;
        let covar_estimate_hat = self.state_transition * self.covar_estimate * self.state_transition.transpose() + self.process_covar;
        self.covar_estimate.copy_from(&covar_estimate_hat);
    }

    pub fn update(&mut self, measurement: &SMatrix<f32, NUM_OBSERVATIONS, 1>) {
        //let innovation_residual = measurement - self.observation_model*self.state_estimate;
        //let innovation_covar: SMatrix<f32, NUM_OBSERVATIONS, NUM_OBSERVATIONS> = self.observation_model * self.covar_estimate * self.observation_model.transpose() + self.observation_covar;
        //let optimal_kalman_gain = self.covar_estimate * self.observation_model.transpose() * innovation_covar.try_inverse().unwrap();
        //self.kalman_gain.copy_from(&optimal_kalman_gain);
        //self.state_estimate += self.kalman_gain*innovation_residual;

        // P = (I - K*H)*P

        /*
        defmt::info!("start K");
        for r in 0..3 {
            for c in 0..5 {
                defmt::info!("{:?}, ", K.row(r)[c]);
            }
            defmt::info!("___________");
        }
        defmt::info!("end");
        */
        

        let z_hat: SMatrix<f32, NUM_OBSERVATIONS, 1> = self.observation_model * self.state_estimate;
        let y: SMatrix<f32, NUM_OBSERVATIONS, 1> = measurement - z_hat;
        self.state_estimate += self.kalman_gain * y;

        // defmt::info!("x predictor: {:?}, {:?}, {:?}", self.state_estimate[0], self.state_estimate[1], self.state_estimate[2]);
    }

    pub fn read_state(&self, state_estimate: &mut SMatrix<f32, NUM_STATES, 1>) {
       state_estimate.copy_from(&self.state_estimate);
    }

    pub fn get_state(&self) -> SMatrix<f32, NUM_STATES, 1> {
        self.state_estimate
    }
}

