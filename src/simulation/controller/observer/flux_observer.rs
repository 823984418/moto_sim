use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke, complex_div, rotate};

#[derive(Debug, Default, Clone)]
pub struct FluxObserver {
    pub rs: f64,

    pub inductance_dq: [f64; 2],

    pub flux: f64,

    pub last_current: [f64; 2],
    pub position: [f64; 2],

    pub last_angle: f64,

    pub speed_lp: f64,
    pub speed_lp_factor: f64,
    pub position_factor: f64,
}

impl Observer<3> for FluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        let delta_current = [
            current[0] - self.last_current[0],
            current[1] - self.last_current[1],
        ];
        self.last_current = current;

        let l0 = (self.inductance_dq[0] + self.inductance_dq[1]) * 0.5;
        let l1 = (self.inductance_dq[0] - self.inductance_dq[1]) * 0.5;
        let p2dib = rotate([delta_current[0], -delta_current[1]], 2.0 * self.last_angle);
        let px = (voltage[0] - self.rs * current[0]) * delta_time
            - l0 * delta_current[0]
            - l1 * p2dib[0];
        let py = (voltage[1] - self.rs * current[1]) * delta_time
            - l0 * delta_current[1]
            - l1 * p2dib[1];

        let pib = rotate([current[0], -current[1]], self.last_angle);
        let s = [pib[0] * 2.0 * l1 + self.flux, pib[1] * 2.0 * l1];

        let delta_position = complex_div([px, py], s);

        self.position[0] += delta_position[0];
        self.position[1] += delta_position[1];

        let len2 = self.position[0] * self.position[0] + self.position[1] * self.position[1];
        let position_factor = (0.0 - len2 * len2) * self.position_factor * delta_time;
        self.position[0] += position_factor * self.position[0];
        self.position[1] += position_factor * self.position[1];

        let angle = atan2(self.position);
        let speed = angle_normal(angle - self.last_angle) / delta_time;
        self.last_angle = angle;
        self.speed_lp += (speed - self.speed_lp) * self.speed_lp_factor * delta_time;

        ObserverOutput {
            electrical_angle: angle,
            electrical_speed: self.speed_lp,
        }
    }
}
