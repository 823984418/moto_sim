pub mod pmsm;

pub struct MotorUpdateInput<const P: usize> {
    /// 电机机械角度 rad
    /// 这个值由机械负载提供
    pub angle: f64,

    /// 电机机械转速 rad/s
    /// 这个值由机械负载提供
    pub speed: f64,

    /// 输入相线电压 V
    /// 这个值由功率模块提供
    pub voltage: [f64; P],
}

pub struct MotorUpdateOutput<const P: usize> {
    /// 电机机械转矩 N*m
    /// 这个值提供给机械负载
    pub torque: f64,

    /// 电机相线电流 A
    /// 这个值提供给功率模块
    pub current: [f64; P],
}

pub trait Motor<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &MotorUpdateInput<P>) -> MotorUpdateOutput<P>;
}
