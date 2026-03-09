use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke};

#[derive(Debug, Default, Clone)]
pub struct ExFluxObserver {
    pub last_i: [f64; 2],

    pub rs: f64,
    pub inductance_dq: [f64; 2],
    pub flux: f64,

    pub alpha: f64,
    pub gamma: f64,
    pub epsilon: f64,

    pub omega1: [f64; 2],
    pub pi_lp: [f64; 2],
    pub i_lp: [f64; 2],
    /// 1/(p+alpha)*(omega2^T omega1)
    pub omega_l: f64,
    pub last_i_sigma: f64,
    pub p_i_sigma_lp: f64,

    pub x: [f64; 2],
    pub y: f64,
    pub lambda: [f64; 2],

    pub last_theta: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for ExFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let i = clarke(input.current);
        let v = clarke(input.voltage);
        let pi = [
            (i[0] - self.last_i[0]) / delta_time,
            (i[1] - self.last_i[1]) / delta_time,
        ];
        self.last_i = i;

        let ld = self.inductance_dq[0];
        let lq = self.inductance_dq[1];
        let l0 = ld - lq;
        let ell = self.flux * l0;
        let alpha_delta_time = self.alpha * delta_time;

        self.omega1[0] += (v[0] - self.rs * i[0] - lq * pi[0] - self.omega1[0]) * alpha_delta_time;
        self.omega1[1] += (v[1] - self.rs * i[1] - lq * pi[1] - self.omega1[1]) * alpha_delta_time;
        self.pi_lp[0] += (pi[0] - self.pi_lp[0]) * alpha_delta_time;
        self.pi_lp[1] += (pi[1] - self.pi_lp[1]) * alpha_delta_time;
        self.i_lp[0] += (i[0] - self.i_lp[0]) * alpha_delta_time;
        self.i_lp[1] += (i[1] - self.i_lp[1]) * alpha_delta_time;

        let omega2 = [
            self.omega1[0] - l0 * self.pi_lp[0],
            self.omega1[1] - l0 * self.pi_lp[1],
        ];

        self.omega_l += (self.omega1[0] * omega2[0] + self.omega1[1] * omega2[1]
            - self.omega_l * self.alpha)
            * delta_time;
        let y = l0 * (self.i_lp[0] * self.omega1[0] + self.i_lp[1] * self.omega1[1])
            + (self.omega1[0] * self.omega1[0] + self.omega1[1] * self.omega1[1]) / self.alpha
            + self.omega_l;
        self.y = y;
        let phi = [self.omega1[0] + omega2[0], self.omega1[1] + omega2[1]];

        let x = [self.lambda[0] - lq * i[0], self.lambda[1] - lq * i[1]];
        self.x = x;
        let x_len = f64::sqrt(x[0] * x[0] + x[1] * x[1]);
        let sigma = if x_len > self.epsilon {
            [x[0] / x_len, x[1] / x_len]
        } else {
            [0.0, 0.0]
        };
        let i_sigma = i[0] * sigma[0] + i[1] * sigma[1];
        let p_i_sigma = (i_sigma - self.last_i_sigma) / delta_time;
        self.last_i_sigma = i_sigma;
        self.p_i_sigma_lp += (p_i_sigma - self.p_i_sigma_lp) * alpha_delta_time;
        let error = y - (phi[0] * x[0] + phi[1] * x[1]) + ell * self.p_i_sigma_lp;
        self.lambda[0] += (v[0] - self.rs * i[0] + self.gamma * phi[0] * error) * delta_time;
        self.lambda[1] += (v[1] - self.rs * i[1] + self.gamma * phi[1] * error) * delta_time;
        let theta = atan2(x);
        self.speed_lp += (angle_normal(theta - self.last_theta) - self.speed_lp * delta_time)
            * self.speed_lp_factor;
        self.last_theta = theta;
        ObserverOutput {
            electrical_angle: theta,
            electrical_speed: self.speed_lp,
            continuous_speed: self.speed_lp,
        }
    }
}
