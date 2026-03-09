use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke};

#[derive(Debug, Default, Clone)]
pub struct BaseFluxObserver {
    pub last_i: [f64; 2],

    pub rs: f64,
    pub inductance: f64,
    pub flux: f64,

    pub alpha: f64,
    pub gamma: f64,
    pub epsilon: f64,

    pub omega1: [f64; 2],
    /// 1/(p+alpha)*(omega2^T omega1)
    pub omega_l: f64,

    pub x: [f64; 2],
    pub lambda: [f64; 2],

    pub last_theta: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for BaseFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let i = clarke(input.current);
        let v = clarke(input.voltage);
        let pi = [
            (i[0] - self.last_i[0]) / delta_time,
            (i[1] - self.last_i[1]) / delta_time,
        ];
        self.last_i = i;

        let vi = [v[0] - self.rs * i[0], v[1] - self.rs * i[1]];

        let lq = self.inductance;
        let alpha_delta_time = self.alpha * delta_time;

        self.omega1[0] += (vi[0] - lq * pi[0] - self.omega1[0]) * alpha_delta_time;
        self.omega1[1] += (vi[1] - lq * pi[1] - self.omega1[1]) * alpha_delta_time;

        let y = (self.omega1[0] * self.omega1[0] + self.omega1[1] * self.omega1[1]) / self.alpha;
        self.omega_l = (y - self.omega_l) * alpha_delta_time;

        let x = [self.lambda[0] - lq * i[0], self.lambda[1] - lq * i[1]];
        self.x = x;
        let error = y + self.omega_l - 2.0 * (self.omega1[0] * x[0] + self.omega1[1] * x[1]);
        self.lambda[0] += (vi[0] + self.gamma * 2.0 * self.omega1[0] * error) * delta_time;
        self.lambda[1] += (vi[1] + self.gamma * 2.0 * self.omega1[1] * error) * delta_time;
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
