use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke, complex_div, rotate};

#[derive(Debug, Default, Clone)]
pub struct MixObserver {
    pub rs: f64,
    pub inductance_dq: [f64; 2],
    pub flux: f64,
    pub last_current: [f64; 2],
    pub static_speed_lp: [f64; 2],
    pub sync_speed_lp: [f64; 2],

    pub angle: f64,
    pub speed: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,

    pub speed_error_kp: f64,
    pub speed_error_ki: f64,
    pub theta_error_kp: f64,
    pub theta_error_ki: f64,
}

impl Observer<3> for MixObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        let di = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = current;
        let last_angle = self.angle;
        self.angle += self.speed * delta_time;

        let l0 = (self.inductance_dq[0] + self.inductance_dq[1]) * 0.5;
        let l1 = (self.inductance_dq[0] - self.inductance_dq[1]) * 0.5;
        let p2zj_dib = rotate([di[0], -di[1]], 2.0 * self.angle);
        let px = voltage[0] - self.rs * current[0] - l0 * di[0] - l1 * p2zj_dib[0];
        let py = voltage[1] - self.rs * current[1] - l0 * di[1] - l1 * p2zj_dib[1];

        let pzj_ib = rotate([current[0], -current[1]], self.angle);
        let s = [2.0 * l1 * pzj_ib[0] + self.flux, 2.0 * l1 * pzj_ib[1]];

        let static_speed = complex_div([px, py], s);
        self.static_speed_lp[0] += (static_speed[0] - self.static_speed_lp[0]) * 0.1 * delta_time;
        self.static_speed_lp[1] += (static_speed[1] - self.static_speed_lp[1]) * 0.1 * delta_time;

        let sync_speed = rotate(
            [
                static_speed[0] - self.static_speed_lp[0],
                static_speed[1] - self.static_speed_lp[1],
            ],
            -self.angle,
        );
        self.sync_speed_lp[0] += (sync_speed[0] - self.sync_speed_lp[0]) * 10000.0 * delta_time;
        self.sync_speed_lp[1] += (sync_speed[1] - self.sync_speed_lp[1]) * 10000.0 * delta_time;

        let mut theta_error = self.sync_speed_lp[0];
        if self.speed >= 0.0 {
            theta_error = -theta_error;
        }
        let speed_error = self.sync_speed_lp[1] - self.speed;

        self.angle +=
            (speed_error * self.speed_error_kp + theta_error * self.theta_error_kp) * delta_time;
        self.speed +=
            (speed_error * self.speed_error_ki + theta_error * self.theta_error_ki) * delta_time;
        self.angle = angle_normal(self.angle);

        self.speed_lp += (angle_normal(self.angle - last_angle) / delta_time - self.speed_lp)
            * delta_time
            * self.speed_lp_factor;

        ObserverOutput {
            electrical_angle: self.angle,
            electrical_speed: self.speed_lp,
            continuous_speed: sync_speed[1],
        }
    }
}
