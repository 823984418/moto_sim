use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke, complex_div, rotate};

#[derive(Debug, Default, Clone)]
pub struct MpObserver {
    pub last_current: [f64; 2],

    pub rs: f64,
    pub inductance_dq: [f64; 2],
    pub flux: f64,

    pub error_factor: f64,
    pub background: f64,

    pub sample: Vec<f64>,

    pub angle: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for MpObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        let di = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = current;

        let l0 = (self.inductance_dq[0] + self.inductance_dq[1]) * 0.5;
        let l1 = (self.inductance_dq[0] - self.inductance_dq[1]) * 0.5;
        let px = voltage[0] - self.rs * current[0] - l0 * di[0];
        let py = voltage[1] - self.rs * current[1] - l0 * di[1];

        let sample_count = self.sample.len();

        let step_angle = std::f64::consts::TAU / sample_count as f64;

        let mut new_sample = vec![0.0; sample_count];
        for i in 0..sample_count {
            let angle = step_angle * i as f64;
            let p2zj_dib = rotate([di[0], -di[1]], 2.0 * angle);
            let pzj_ib = rotate([current[0], -current[1]], angle);
            let s = [2.0 * l1 * pzj_ib[0] + self.flux, 2.0 * l1 * pzj_ib[1]];
            let static_angle = complex_div([px - l1 * p2zj_dib[0], py - l1 * p2zj_dib[1]], s);
            let sync_speed = rotate(static_angle, -angle);
            let move_sample = sync_speed[1] * delta_time / step_angle;
            let sample = self.sample[i]
                * (-sync_speed[0].powi(2) * self.error_factor * delta_time).exp()
                + self.background / sample_count as f64 * delta_time;

            let new_index = i as f64 + move_sample;
            let left = new_index.floor();
            let left_index = (left as i32).rem_euclid(sample_count as i32);
            let right_index = (left as i32 + 1).rem_euclid(sample_count as i32);
            new_sample[left_index as usize] += sample * (1.0 - (new_index - left));
            new_sample[right_index as usize] += sample * (new_index - left);
        }

        let mut weight = 0.0;
        let mut angle_x_weight = 0.0;
        let mut angle_y_weight = 0.0;
        for i in 0..sample_count {
            let angle = step_angle * i as f64;
            let sample = new_sample[i];
            weight += sample;
            angle_x_weight += sample * angle.cos();
            angle_y_weight += sample * angle.sin();
        }

        let inv_weight = 1.0 / weight;
        for i in 0..sample_count {
            self.sample[i] = new_sample[i] * inv_weight;
        }

        let last_angle = self.angle;
        self.angle = atan2([angle_x_weight, angle_y_weight]);

        self.speed_lp += (angle_normal(self.angle - last_angle) / delta_time - self.speed_lp)
            * self.speed_lp_factor
            * delta_time;
        ObserverOutput {
            electrical_angle: self.angle,
            electrical_speed: self.speed_lp,
            continuous_speed: self.speed_lp,
        }
    }
}
