use std::collections::HashMap;

use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use rand_distr::StandardNormal;

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke, complex_div, nn, rotate};

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct SSampleCell {
    pub rs: i32,
    pub l0: i32,
    pub l1: i32,
    pub flux: i32,
    pub angle: i32,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub struct SSample {
    pub power: f64,
    pub rs: f64,
    pub l0: f64,
    pub l1: f64,
    pub flux: f64,
    pub angle: f64,
}

impl SSample {
    pub fn mix_with(&mut self, rhs: &Self) {
        // s = (a * pa + b * pb) / (pa + pb) = a + (b - a) * pb / (pa + pb)
        let pd = rhs.power / (self.power + rhs.power);
        self.power += rhs.power;
        self.rs += (rhs.rs - self.rs) * pd;
        self.l0 += (rhs.l0 - self.l0) * pd;
        self.l1 += (rhs.l1 - self.l1) * pd;
        self.flux += (rhs.flux - self.flux) * pd;
        self.angle = angle_normal(self.angle + angle_normal(rhs.angle - self.angle) * pd);
    }
}

#[derive(Debug)]
pub struct SSObserver {
    pub last_current: [f64; 2],
    pub error_factor: f64,

    pub rng: StdRng,
    pub gen_rs: f64,
    pub gen_l0: f64,
    pub gen_l1: f64,
    pub gen_flux: f64,
    pub gen_angle: f64,

    pub rs_cell_inv: f64,
    pub l0_cell_inv: f64,
    pub l1_cell_inv: f64,
    pub flux_cell_inv: f64,
    pub angle_cell_inv: f64,

    pub min_power: f64,
    pub gen_power: f64,
    pub target_sample_count: usize,
    pub samples: HashMap<SSampleCell, SSample>,

    pub speed_lp: f64,
    pub speed_lp_factor: f64,

    pub rs_est: f64,
    pub l0_est: f64,
    pub l1_est: f64,
    pub flux_est: f64,
    pub speed_est: f64,
    pub angle_est: f64,
}

impl Default for SSObserver {
    fn default() -> Self {
        Self {
            last_current: [0.0; 2],
            error_factor: 0.0,
            rng: StdRng::seed_from_u64(0),
            gen_rs: 0.0,
            gen_l0: 0.0,
            gen_l1: 0.0,
            gen_flux: 0.0,
            gen_angle: 0.0,
            rs_cell_inv: 0.0,
            l0_cell_inv: 0.0,
            l1_cell_inv: 0.0,
            flux_cell_inv: 0.0,
            angle_cell_inv: 0.0,
            min_power: 0.0,
            gen_power: 0.0,
            target_sample_count: 0,
            samples: HashMap::new(),
            speed_lp: 0.0,
            speed_lp_factor: 0.0,
            rs_est: 0.0,
            l0_est: 0.0,
            l1_est: 0.0,
            flux_est: 0.0,
            speed_est: 0.0,
            angle_est: 0.0,
        }
    }
}

impl Clone for SSObserver {
    fn clone(&self) -> Self {
        Self {
            last_current: self.last_current,
            error_factor: self.error_factor,
            rng: unsafe { std::ptr::read(&self.rng) },
            gen_rs: self.gen_rs,
            gen_l0: self.gen_l0,
            gen_l1: self.gen_l1,
            gen_flux: self.gen_flux,
            gen_angle: self.gen_angle,
            rs_cell_inv: self.rs_cell_inv,
            l0_cell_inv: self.l0_cell_inv,
            l1_cell_inv: self.l1_cell_inv,
            flux_cell_inv: self.flux_cell_inv,
            angle_cell_inv: self.angle_cell_inv,
            min_power: self.min_power,
            gen_power: self.gen_power,
            target_sample_count: self.target_sample_count,
            samples: self.samples.clone(),
            speed_lp: self.speed_lp,
            speed_lp_factor: self.speed_lp_factor,
            rs_est: self.rs_est,
            l0_est: self.l0_est,
            l1_est: self.l1_est,
            flux_est: self.flux_est,
            speed_est: self.speed_est,
            angle_est: self.angle_est,
        }
    }
}

impl SSObserver {
    pub fn sample_cell(&self, sample: &SSample) -> SSampleCell {
        SSampleCell {
            rs: f64::floor(sample.rs * self.rs_cell_inv) as i32,
            l0: f64::floor(sample.l0 * self.l0_cell_inv) as i32,
            l1: f64::floor(sample.l1 * self.l1_cell_inv) as i32,
            flux: f64::floor(sample.flux * self.flux_cell_inv) as i32,
            angle: f64::floor(sample.angle * self.angle_cell_inv) as i32,
        }
    }

    pub fn add_sample(&mut self, sample: SSample) {
        let cell = self.sample_cell(&sample);
        self.samples
            .entry(cell)
            .and_modify(|i| i.mix_with(&sample))
            .or_insert(sample);
    }
}

impl Observer<3> for SSObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        // diff i
        let di = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = current;

        let mut all_power = 0.0;
        let mut all_rs_power = 0.0;
        let mut all_l0_power = 0.0;
        let mut all_l1_power = 0.0;
        let mut all_flux_power = 0.0;
        let mut all_angle_power = [0.0; 2];

        let min_power = self.min_power / self.target_sample_count as f64;
        let samples = std::mem::take(&mut self.samples);
        for sample in samples.values() {
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

            let mut sample = sample.clone();
            sample.power *= ((-sync_speed[0].powi(2)) * self.error_factor * delta_time).exp();

            if sample.power < min_power {
                continue;
            }

            let ref_speed = if sync_speed[1] >= 0.0 {
                -sync_speed[0]
            } else {
                sync_speed[0]
            };
            sample.angle = angle_normal(angle + (sync_speed[1] + ref_speed * 0.01) * delta_time);

            let power = sample.power;
            all_power += power;
            all_rs_power += sample.rs * power;
            all_l0_power += sample.l0 * power;
            all_l1_power += sample.l1 * power;
            all_flux_power += sample.flux * power;
            all_angle_power[0] += sample.angle.cos() * power;
            all_angle_power[1] += sample.angle.sin() * power;

            self.add_sample(sample);
        }
        drop(samples);
        let last_angle = self.angle_est;

        let inv_power = nn(1.0 / all_power);
        self.rs_est = all_rs_power * inv_power;
        self.l0_est = all_l0_power * inv_power;
        self.l1_est = all_l1_power * inv_power;
        self.flux_est = all_flux_power * inv_power;
        self.angle_est = atan2(all_angle_power);
        for i in self.samples.values_mut() {
            i.power *= inv_power;
        }
        let max_sample = self
            .samples
            .values()
            .max_by(|a, b| f64::total_cmp(&a.power, &b.power))
            .cloned()
            .unwrap_or_default();

        if self.samples.len() < self.target_sample_count {
            // println!("add {}", self.target_sample_count - self.samples.len());
        }
        let pp = 1.0;
        let power = self.gen_power
            / self.target_sample_count as f64
            / (self.target_sample_count - self.samples.len()) as f64;
        while self.samples.len() < self.target_sample_count {
            let sample = SSample {
                power,
                rs: max_sample.rs + self.rng.sample::<f64, _>(StandardNormal) * self.gen_rs * pp,
                l0: max_sample.l0 + self.rng.sample::<f64, _>(StandardNormal) * self.gen_l0 * pp,
                l1: max_sample.l1 + self.rng.sample::<f64, _>(StandardNormal) * self.gen_l1 * pp,
                flux: max_sample.flux
                    + self.rng.sample::<f64, _>(StandardNormal) * self.gen_flux * pp,
                angle: angle_normal(
                    max_sample.angle
                        + self.rng.sample::<f64, _>(StandardNormal) * self.gen_angle * pp,
                ),
            };
            if sample.rs > 10.0 || sample.l0 > 0.1 || sample.flux > 0.1 {
                continue;
            }
            if sample.rs > 0.0 && sample.l0 > 0.0 && sample.l1 < 0.0 && sample.flux > 0.0 {
                self.add_sample(sample);
            }
        }

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
