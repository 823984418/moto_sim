pub mod pmsm;

#[derive(Debug, Clone)]
pub struct MotorInput<const P: usize> {
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

impl<const P: usize> Default for MotorInput<P> {
    fn default() -> Self {
        Self {
            angle: 0.0,
            speed: 0.0,
            voltage: [0.0; P],
        }
    }
}

#[derive(Debug, Clone)]
pub struct MotorOutput<const P: usize> {
    /// 电机机械转矩 N*m
    /// 这个值提供给机械负载
    pub torque: f64,

    /// 电机相线电流 A
    /// 这个值提供给功率模块
    pub current: [f64; P],
}

impl<const P: usize> Default for MotorOutput<P> {
    fn default() -> Self {
        Self {
            torque: 0.0,
            current: [0.0; P],
        }
    }
}

pub trait Motor<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &MotorInput<P>) -> MotorOutput<P>;
}
