use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke, complex_mode};

#[derive(Debug, Default, Clone)]
pub struct SuperFluxSample {
    pub rs: f64,
    pub inductance: f64,

    pub power: f64,

    pub omega_l: f64,
    pub lambda: [f64; 2],
}

#[derive(Debug, Default, Clone)]
pub struct SuperFluxObserver {
    pub last_i: [f64; 2],
    pub alpha: f64,
    pub gamma: f64,
    pub power_alpha: f64,

    pub v_lp: [f64; 2],
    pub i_lp: [f64; 2],
    pub di_lp: [f64; 2],

    pub samples: Vec<SuperFluxSample>,

    pub theta: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,

    pub rs: f64,
    pub inductance: f64,
    pub flux: f64,
}

impl SuperFluxObserver {
    pub fn add_sample(&mut self, rs: f64, inductance: f64) {
        self.samples.push(SuperFluxSample {
            rs,
            inductance,
            power: 0.0,
            omega_l: 0.0,
            lambda: [0.0, 0.0],
        });
    }
}

impl Observer<3> for SuperFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let i = clarke(input.current);
        let v = clarke(input.voltage);
        let alpha = self.alpha;
        let alpha_delta_time = alpha * delta_time;

        self.v_lp[0] += (v[0] - self.v_lp[0]) * alpha_delta_time;
        self.v_lp[1] += (v[1] - self.v_lp[1]) * alpha_delta_time;
        self.i_lp[0] += (i[0] - self.i_lp[0]) * alpha_delta_time;
        self.i_lp[1] += (i[1] - self.i_lp[1]) * alpha_delta_time;
        self.di_lp[0] += (i[0] - self.last_i[0]) * alpha - self.di_lp[0] * alpha_delta_time;
        self.di_lp[1] += (i[1] - self.last_i[1]) * alpha - self.di_lp[1] * alpha_delta_time;
        self.last_i = i;

        let v_lp = self.v_lp;
        let i_lp = self.i_lp;
        let di_lp = self.di_lp;

        let mut all_power = 0.0;
        let mut angle_power = [0.0; 2];
        let mut rs_power = 0.0;
        let mut inductance_power = 0.0;
        let mut flux_power = 0.0;
        for sample in &mut self.samples {
            let rs = sample.rs;
            let lq = sample.inductance;
            let omega1 = [
                v_lp[0] - rs * i_lp[0] - lq * di_lp[0],
                v_lp[1] - rs * i_lp[1] - lq * di_lp[1],
            ];
            let y = (omega1[0] * omega1[0] + omega1[1] * omega1[1]) / alpha;
            sample.omega_l += (y - sample.omega_l) * alpha_delta_time;
            let x = [sample.lambda[0] - lq * i[0], sample.lambda[1] - lq * i[1]];
            let error = self.gamma
                * 2.0
                * (y + sample.omega_l - 2.0 * (omega1[0] * x[0] + omega1[1] * x[1]));
            sample.lambda[0] += (v[0] - rs * i[0] + omega1[0] * error) * delta_time;
            sample.lambda[1] += (v[1] - rs * i[1] + omega1[1] * error) * delta_time;

            let flux = complex_mode(sample.lambda);
            let e = error * y;
            sample.power +=
                (1.0 / (e * e + 0.0000001) - sample.power) * self.power_alpha * delta_time;
            let inv_flux = 1.0 / flux;
            let angle = [sample.lambda[0] * inv_flux, sample.lambda[1] * inv_flux];

            let power = sample.power;
            all_power += power;
            angle_power[0] += angle[0] * power;
            angle_power[1] += angle[1] * power;
            rs_power += rs * power;
            inductance_power += lq * power;
            flux_power += flux * power;
        }

        let inv_power = 1.0 / all_power;
        self.rs = rs_power * inv_power;
        self.inductance = inductance_power * inv_power;
        self.flux = flux_power * inv_power;

        let theta = atan2(angle_power);
        self.speed_lp +=
            (angle_normal(theta - self.theta) - self.speed_lp * delta_time) * self.speed_lp_factor;
        self.theta = theta;
        ObserverOutput {
            electrical_angle: theta,
            electrical_speed: self.speed_lp,
            continuous_speed: self.speed_lp,
        }
    }
}
