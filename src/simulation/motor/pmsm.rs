use crate::simulation::{clarke, inverse_clarke, rotate};
use crate::simulation::motor::{Motor, MotorInput, MotorOutput};
#[derive(Debug, Default, Clone)]
pub struct PermanentMagnetSynchronousMotor {
    /// 磁极对数
    pub pole_pairs: f64,

    /// 定子电阻
    pub rs: f64,

    /// 电感
    pub inductance_dq: [f64; 2],

    /// 磁链常数
    pub flux: f64,

    /// 转子坐标系电流
    pub current_dq: [f64; 2],
}

impl Motor<3> for PermanentMagnetSynchronousMotor {
    fn update(&mut self, delta_time: f64, input: &MotorInput<3>) -> MotorOutput<3> {
        let electrical_angle = input.angle * self.pole_pairs;
        let electrical_speed = input.speed * self.pole_pairs;
        let voltage_ab = clarke(input.voltage);
        let voltage_dq = rotate(voltage_ab, -electrical_angle);
        let last_current_dq = self.current_dq;

        let next_current_d = last_current_dq[0]
            + (voltage_dq[0] - last_current_dq[0] * self.rs
                + last_current_dq[1] * self.inductance_dq[1] * electrical_speed)
                / self.inductance_dq[0]
                * delta_time;

        let next_current_q = last_current_dq[1]
            + (voltage_dq[1]
                - last_current_dq[1] * self.rs
                - (self.flux + last_current_dq[0] * self.inductance_dq[0]) * electrical_speed)
                / self.inductance_dq[1]
                * delta_time;

        self.current_dq = [next_current_d, next_current_q];

        let torque = 1.5
            * self.pole_pairs
            * self.current_dq[1]
            * ((self.inductance_dq[0] - self.inductance_dq[1]) * self.current_dq[0] + self.flux);

        let current_ab = rotate(self.current_dq, electrical_angle);
        MotorOutput {
            torque,
            current: inverse_clarke(current_ab),
        }
    }
}
