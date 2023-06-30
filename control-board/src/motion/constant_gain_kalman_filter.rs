use nalgebra::base::SMatrix;

#[allow(non_camel_case_types)]
pub struct CgKalmanFilter<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> {
    F_k: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    B_k: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
    H_k: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
    Q_k: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
    R_k: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_OBSERVATIONS>,
    P_k: SMatrix<f32, NUM_STATES, NUM_STATES>,
    K_k: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
    x_hat: SMatrix<f32, NUM_STATES, 1>,
}

impl<'a, const NUM_STATES: usize, const NUM_CONTROL_INPUTS: usize, const NUM_OBSERVATIONS: usize> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
    pub fn new(F_k: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                B_k: &'a SMatrix<f32, NUM_STATES, NUM_CONTROL_INPUTS>,
                H_k: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_STATES>,
                Q_k: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                R_k: &'a SMatrix<f32, NUM_OBSERVATIONS, NUM_OBSERVATIONS>,
                P_k: &'a SMatrix<f32, NUM_STATES, NUM_STATES>,
                K_k: &'a SMatrix<f32, NUM_STATES, NUM_OBSERVATIONS>,
                // TODO: ? accept starting state? 
            ) -> CgKalmanFilter<'a, NUM_STATES, NUM_CONTROL_INPUTS, NUM_OBSERVATIONS> {
        let mut filter = CgKalmanFilter {
            F_k: F_k,
            B_k: B_k,
            H_k: H_k,
            Q_k: Q_k,
            R_k: R_k,
            P_k: SMatrix::<f32, NUM_STATES, NUM_STATES>::zeros(),
            K_k: K_k,
            x_hat: SMatrix::<f32, NUM_STATES, 1>::zeros(),
        };

        filter.P_k.copy_from(P_k);

        filter
    }

    pub fn predict(&mut self, u: &SMatrix<f32, NUM_CONTROL_INPUTS, 1>) {
        self.x_hat = self.F_k * self.x_hat + self.B_k * u;
        let P_k_hat = self.F_k * self.P_k * self.F_k.transpose() + self.Q_k;
        self.P_k.copy_from(&P_k_hat);
    }

    pub fn update(&mut self, z: &SMatrix<f32, NUM_OBSERVATIONS, 1>) {
        // y = z - H*x_hat
        // S = H*P*H' + R
        // K = P*H'*S^-1
        // x_hat += K*y
        // P = (I - K*H)*P
        let z_hat: SMatrix<f32, NUM_OBSERVATIONS, 1> = self.H_k * self.x_hat;
        let y: SMatrix<f32, NUM_OBSERVATIONS, 1> = z - z_hat;
        self.x_hat += self.K_k * y;
    }

    pub fn read_state(&self, x_hat: &mut SMatrix<f32, NUM_STATES, 1>) {
       x_hat.copy_from(&self.x_hat);
    }

    pub fn get_state(&self) -> SMatrix<f32, NUM_STATES, 1> {
        self.x_hat
    }
}

