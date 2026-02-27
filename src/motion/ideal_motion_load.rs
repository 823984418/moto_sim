use crate::motion::{MotionLoad, MotionLoadInput, MotionLoadOutput};

pub struct IdealMotionLoad {
    /// 转动惯量
    pub inertia: f64,

    /// 静摩擦扭矩
    pub static_friction_torque: f64,

    /// 动摩擦系数
    pub kinetic_friction_factor: f64,

    /// 当前机械角度
    pub angle: f64,

    /// 当前机械速度
    pub speed: f64,
}

impl MotionLoad for IdealMotionLoad {
    fn update(&mut self, delta_time: f64, input: &MotionLoadInput) -> MotionLoadOutput {
        let mut speed = self.speed;

        let delta_time_div_inertia = delta_time / self.inertia;
        speed += (input.torque - speed * self.kinetic_friction_factor) * delta_time_div_inertia;

        if speed > 0.0 {
            speed -= self.static_friction_torque * delta_time_div_inertia;
            speed = f64::max(speed, 0.0);
        } else {
            speed += self.static_friction_torque * delta_time_div_inertia;
            speed = f64::min(speed, 0.0);
        }

        self.speed = speed;
        self.angle += delta_time * speed;

        MotionLoadOutput {
            theta: self.angle,
            speed: self.speed,
        }
    }
}
