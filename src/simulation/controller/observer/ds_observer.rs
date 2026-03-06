use rand_distr::num_traits::real::Real;

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke, complex_div, rotate};

#[derive(Debug, Default, Clone)]
pub struct DsObserver {
    pub rs: f64,
    pub inductance_dq: [f64; 2],
    pub flux: f64,
    pub last_current: [f64; 2],
    pub sync_speed_lp: [f64; 2],

    pub angle: f64,
    pub last_static_speed: [f64; 2],
    pub sync_angle_lp: [f64; 2],
    pub sync_angle_lp_factor: f64,

    pub pll_speed: f64,
    pub pll_angle_kp: f64,
    pub pll_angle_ki: f64,
    pub pll_speed_ki: f64,

    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for DsObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        let di = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = current;
        self.angle += self.pll_speed * delta_time;

        let l0 = (self.inductance_dq[0] + self.inductance_dq[1]) * 0.5;
        let l1 = (self.inductance_dq[0] - self.inductance_dq[1]) * 0.5;
        let p2zj_dib = rotate([di[0], -di[1]], 2.0 * self.angle);
        let px = voltage[0] - self.rs * current[0] - l0 * di[0] - l1 * p2zj_dib[0];
        let py = voltage[1] - self.rs * current[1] - l0 * di[1] - l1 * p2zj_dib[1];

        let pzj_ib = rotate([current[0], -current[1]], self.angle);
        let s = [2.0 * l1 * pzj_ib[0] + self.flux, 2.0 * l1 * pzj_ib[1]];

        let static_speed = complex_div([px, py], s);
        let sync_angle = rotate(
            [
                (self.last_static_speed[0] - static_speed[0]) / delta_time,
                (self.last_static_speed[1] - static_speed[1]) / delta_time,
            ],
            -self.angle,
        );
        self.last_static_speed = static_speed;

        self.sync_angle_lp[0] +=
            (sync_angle[0] - self.sync_angle_lp[0]) * self.sync_angle_lp_factor * delta_time;
        self.sync_angle_lp[1] +=
            (sync_angle[1] - self.sync_angle_lp[1]) * self.sync_angle_lp_factor * delta_time;

        let sync_p_speed_normal = 1.0
            / (self.sync_angle_lp[0] * self.sync_angle_lp[0]
                + self.sync_angle_lp[1] * self.sync_angle_lp[1])
                .sqrt()
                .sqrt();

        let sync_p_speed = [
            self.sync_angle_lp[0] * sync_p_speed_normal,
            self.sync_angle_lp[1] * sync_p_speed_normal,
        ];

        let est_speed = sync_p_speed[0];

        let angle_error = sync_p_speed[1];
        let speed_kp = angle_error * self.pll_angle_kp;
        self.pll_speed += (angle_error * self.pll_angle_ki
            + (est_speed - self.pll_speed) * self.pll_speed_ki)
            * delta_time;
        self.angle += speed_kp * delta_time;
        self.angle = angle_normal(self.angle);
        self.speed_lp +=
            (speed_kp + self.pll_speed - self.speed_lp) * self.speed_lp_factor * delta_time;

        ObserverOutput {
            electrical_angle: self.angle,
            electrical_speed: self.speed_lp,
            continuous_speed: self.pll_speed,
        }
    }
}
