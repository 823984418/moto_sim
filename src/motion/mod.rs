pub mod ideal_motion_load;

pub struct MotionLoadInput {
    /// 提供的机械转矩 N*m
    pub torque: f64,
}

pub struct MotionLoadOutput {
    /// 所处的机械角度 rad
    pub theta: f64,

    /// 所处的机械角速度 rad/s
    pub speed: f64,
}

pub trait MotionLoad {
    fn update(&mut self, delta_time: f64, input: &MotionLoadInput) -> MotionLoadOutput;
}
