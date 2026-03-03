use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use rand_distr::Normal;

#[derive(Debug)]
pub struct Noise {
    distribution: Normal<f64>,
    rng: StdRng,
    pub value: f64,
}

impl Default for Noise {
    fn default() -> Self {
        Self {
            distribution: Normal::new(0.0, 0.0).unwrap(),
            rng: StdRng::seed_from_u64(0),
            value: 0.0,
        }
    }
}

impl Clone for Noise {
    fn clone(&self) -> Self {
        unsafe { core::ptr::read(self) }
    }
}

impl Noise {
    pub fn new(std_dev: f64, seed: u64) -> Self {
        Self {
            distribution: Normal::new(0.0, std_dev).unwrap(),
            rng: StdRng::seed_from_u64(seed),
            value: 0.0,
        }
    }

    pub fn update(&mut self, delta_time: f64) {
        self.value = self.rng.sample(self.distribution);
    }
}
