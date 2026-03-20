use rand::distr::{StandardUniform, Uniform};
use rand::prelude::StdRng;
use rand::{RngExt, SeedableRng};
use rand_distr::StandardNormal;

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke, complex_div, nn, rotate};

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub struct S2Sample {
    pub power: f64,
    pub rs: f64,
    pub l0: f64,
    pub l1: f64,
    pub flux: f64,
    pub angle: f64,
}

#[derive(Debug)]
pub struct SS2Observer {
    pub last_current: [f64; 2],
    pub error_factor: f64,
    pub samples: Vec<S2Sample>,
    pub rng: StdRng,
    pub target_sample_count: usize,

    pub dist_rs: f64,
    pub dist_l0: f64,
    pub dist_l1: f64,
    pub dist_flux: f64,
    pub dist_angle: f64,

    pub rs_est: f64,
    pub l0_est: f64,
    pub l1_est: f64,
    pub flux_est: f64,
    pub angle_est: f64,

    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Clone for SS2Observer {
    fn clone(&self) -> Self {
        Self {
            last_current: self.last_current,
            error_factor: self.error_factor,
            samples: self.samples.clone(),
            rng: unsafe { std::ptr::read(&self.rng) },
            target_sample_count: self.target_sample_count,
            dist_rs: self.dist_rs,
            dist_l0: self.dist_l0,
            dist_l1: self.dist_l1,
            dist_flux: self.dist_flux,
            dist_angle: self.dist_angle,
            rs_est: self.rs_est,
            l0_est: self.l0_est,
            l1_est: self.l1_est,
            flux_est: self.flux_est,
            angle_est: self.angle_est,
            speed_lp: self.speed_lp,
            speed_lp_factor: self.speed_lp_factor,
        }
    }
}

impl Default for SS2Observer {
    fn default() -> Self {
        Self {
            last_current: [0.0, 0.0],
            error_factor: 0.0,
            samples: Vec::new(),
            rng: StdRng::seed_from_u64(0),
            target_sample_count: 0,
            dist_rs: 0.0,
            dist_l0: 0.0,
            dist_l1: 0.0,
            dist_flux: 0.0,
            dist_angle: 0.0,
            rs_est: 0.0,
            l0_est: 0.0,
            l1_est: 0.0,
            flux_est: 0.0,
            angle_est: 0.0,
            speed_lp: 0.0,
            speed_lp_factor: 0.0,
        }
    }
}

impl Observer<3> for SS2Observer {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        // diff i
        let di = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = current;

        for sample in &mut self.samples {
            let rs = sample.rs;
            let l0 = sample.l0;
            let l1 = sample.l1;
            let flux = sample.flux;
            let angle = sample.angle;

            // exp{2 theta j} diff i bar
            let p2zj_dib = rotate([di[0], -di[1]], 2.0 * angle);
            let px = voltage[0] - rs * current[0] - l0 * di[0] - l1 * p2zj_dib[0];
            let py = voltage[1] - rs * current[1] - l0 * di[1] - l1 * p2zj_dib[1];
            // exp{theta j} i bar
            let pzj_ib = rotate([current[0], -current[1]], angle);
            let s = [2.0 * l1 * pzj_ib[0] + flux, 2.0 * l1 * pzj_ib[1]];
            let static_speed = complex_div([px, py], s);
            let sync_speed = rotate(static_speed, -angle);

            let ref_speed = if sync_speed[1] >= 0.0 {
                -sync_speed[0]
            } else {
                sync_speed[0]
            };
            sample.angle =
                angle_normal(sample.angle + (sync_speed[1] + ref_speed * 0.1) * delta_time);
            sample.power = -sync_speed[0].abs();
        }
        if self.target_sample_count > 0 && self.samples.is_empty() {
            self.samples.push(S2Sample::default());
        }
        while self.samples.len() < self.target_sample_count {
            let i = self
                .rng
                .sample(Uniform::new(0, self.samples.len()).unwrap());
            let mut new_sample = self.samples[i];
            new_sample.rs += self.dist_rs * self.rng.sample::<f64, _>(StandardNormal);
            new_sample.l0 += self.dist_l0 * self.rng.sample::<f64, _>(StandardNormal);
            new_sample.l1 += self.dist_l1 * self.rng.sample::<f64, _>(StandardNormal);
            new_sample.flux += self.dist_flux * self.rng.sample::<f64, _>(StandardNormal);
            new_sample.angle = angle_normal(
                new_sample.angle + self.dist_angle * self.rng.sample::<f64, _>(StandardNormal),
            );
            if new_sample.rs >= 0.0
                && new_sample.l0 >= 0.0
                && new_sample.l1 <= 0.0
                && new_sample.flux >= 0.0
            {
                self.samples.push(new_sample);
            }
        }

        let mut rs_sum = 0.0;
        let mut l0_sum = 0.0;
        let mut l1_sum = 0.0;
        let mut flux_sum = 0.0;
        let mut angle_sum = [0.0; 2];
        for i in &self.samples {
            rs_sum += i.rs;
            l0_sum += i.l0;
            l1_sum += i.l1;
            flux_sum += i.flux;
            angle_sum[0] += i.angle.cos();
            angle_sum[1] += i.angle.sin();
        }
        let last_angle = self.angle_est;
        let inv_power = nn(1.0 / self.samples.len() as f64);
        self.rs_est = rs_sum * inv_power;
        self.l0_est = l0_sum * inv_power;
        self.l1_est = l1_sum * inv_power;
        self.flux_est = flux_sum * inv_power;
        self.angle_est = atan2(angle_sum);
        self.speed_lp += (angle_normal(self.angle_est - last_angle) / delta_time - self.speed_lp)
            * self.speed_lp_factor
            * delta_time;
        ObserverOutput {
            electrical_angle: self.angle_est,
            electrical_speed: self.speed_lp,
            continuous_speed: self.speed_lp,
        }
    }
}
