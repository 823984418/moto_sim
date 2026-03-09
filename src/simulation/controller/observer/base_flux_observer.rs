use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke};

#[derive(Debug, Default, Clone)]
pub struct BaseFluxObserver {
    pub rs: f64,
    pub inductance: f64,

    pub alpha: f64,
    pub gamma: f64,

    pub omega1_div_alpha_add_li: [f64; 2],

    pub x: [f64; 2],
    pub lambda: [f64; 2],

    pub omega_l_div_alpha: f64,
    pub flux_power: f64,
    pub power: f64,
    pub last_theta: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for BaseFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let i = clarke(input.current);
        let v = clarke(input.voltage);

        let vi_delta_time = [
            (v[0] - self.rs * i[0]) * delta_time,
            (v[1] - self.rs * i[1]) * delta_time,
        ];

        let lq = self.inductance;
        self.omega1_div_alpha_add_li[0] +=
            vi_delta_time[0] - self.omega1_div_alpha_add_li[0] * delta_time;
        self.omega1_div_alpha_add_li[1] +=
            vi_delta_time[1] - self.omega1_div_alpha_add_li[1] * delta_time;

        let lq_i = [lq * i[0], lq * i[1]];
        let omega1_div_alpha = [
            self.omega1_div_alpha_add_li[0] - lq_i[0],
            self.omega1_div_alpha_add_li[1] - lq_i[1],
        ];

        self.lambda[0] += vi_delta_time[0];
        self.lambda[1] += vi_delta_time[1];
        let x = [self.lambda[0] - lq_i[0], self.lambda[1] - lq_i[1]];
        self.x = x;

        self.omega_l_div_alpha = ((omega1_div_alpha[0] * omega1_div_alpha[0]
            + omega1_div_alpha[1] * omega1_div_alpha[1])
            * self.alpha
            - self.omega_l_div_alpha)
            * delta_time;
        let error = omega1_div_alpha[0] * (omega1_div_alpha[0] - 2.0 * x[0])
            + omega1_div_alpha[1] * (omega1_div_alpha[1] - 2.0 * x[1]) + self.omega_l_div_alpha;
        let p = 2.0 * self.alpha * self.alpha * self.gamma * delta_time * error;
        self.lambda[0] += p * omega1_div_alpha[0];
        self.lambda[1] += p * omega1_div_alpha[1];

        self.flux_power += (f64::sqrt(x[0] * x[0] + x[1] * x[1]) * self.speed_lp * self.speed_lp
            - self.flux_power)
            * 0.01
            * delta_time;
        self.power += (self.speed_lp * self.speed_lp - self.power) * 0.01 * delta_time;

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
