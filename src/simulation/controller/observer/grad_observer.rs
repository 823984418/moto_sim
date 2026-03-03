use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke, rotate};

#[derive(Debug, Default, Clone)]
pub struct GradObserver {
    pub last_current: [f64; 2],

    pub rs: f64,
    pub l0: f64,
    pub l1: f64,
    pub flux: f64,

    pub speed: f64,
    pub angle: f64,

    pub kr: f64,
    pub kl0: f64,
    pub kflux: f64,
    pub kl1_ds: f64,
    pub kl1_de: f64,
    pub kspeed_l1: f64,
    pub kspeed_flux: f64,
    pub kspeed_kangle: f64,
    pub kangle_l1_di: f64,
    pub kangle_l1_ds: f64,
    pub kangle_flux: f64,

    pub error: [f64; 2],
}

impl Observer<3> for GradObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let i = clarke(input.current);
        let di = [
            (i[0] - self.last_current[0]) / delta_time,
            (i[1] - self.last_current[1]) / delta_time,
        ];
        self.last_current = i;
        let v = clarke(input.voltage);

        self.angle += self.speed * delta_time;

        let ezj = rotate([1.0, 0.0], self.angle);
        let e2zjib = rotate([i[0], -i[1]], 2.0 * self.angle);
        let e2zjdib = rotate([di[0], -di[1]], 2.0 * self.angle);

        let ex = self.rs * i[0] + self.l0 * di[0]
            - self.flux * self.speed * ezj[1]
            - 2.0 * self.speed * self.l1 * e2zjib[1]
            + self.l1 * e2zjdib[0]
            - v[0];
        let ey = self.rs * i[1]
            + self.l0 * di[1]
            + self.flux * self.speed * ezj[0]
            + 2.0 * self.speed * self.l1 * e2zjib[0]
            + self.l1 * e2zjdib[1]
            - v[1];

        let dedr = self.kr * 2.0 * (ex * i[0] + ey * i[1]);
        let dedl0 = self.kl0 * 2.0 * (ex * di[0] + ey * di[1]);
        let dedf = self.kflux * 2.0 * (ey * ezj[0] - ex * ezj[1]);
        let dedl1 = self.kl1_ds * 4.0 * self.speed * (ey * e2zjib[0] - ex * e2zjib[1])
            + self.kl1_de * 2.0 * (ex * e2zjdib[0] + ey * e2zjdib[1]);
        let deds = self.kspeed_l1 * 4.0 * self.l1 * (ey * e2zjib[0] - ey * e2zjdib[1])
            + self.kspeed_flux * 2.0 * self.flux * (ey * ezj[0] - ex * ezj[1]);
        let dedz =
            -self.kangle_l1_ds * 8.0 * self.speed * self.l1 * (ex * e2zjib[0] + ey * e2zjib[0])
                + self.kangle_l1_di * 4.0 * self.l1 * (ey * e2zjdib[0] - ey * e2zjdib[1])
                - self.kangle_flux * 2.0 * self.flux * self.speed * (ex * ezj[0] + ey * ezj[1]);

        self.rs -= dedr * delta_time;
        self.l0 -= dedl0 * delta_time;
        self.flux -= dedf * delta_time;
        self.l1 -= dedl1 * delta_time;
        self.speed -= deds * delta_time + dedz * self.kspeed_kangle * delta_time;
        self.angle -= dedz * delta_time;

        self.rs = self.rs.max(0.0);
        self.l0 = self.l0.max(0.0);
        self.l1 = self.l1.min(0.0);
        self.flux = self.flux.max(0.0);

        self.error = [ex, ey];
        self.angle = angle_normal(self.angle);

        ObserverOutput {
            electrical_angle: self.angle,
            electrical_speed: self.speed,
        }
    }
}
