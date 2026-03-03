use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::angle_normal;

/// 通过锁相环从外部位置传感器重建电角度和电角速度的观测器
#[derive(Default, Debug, Clone)]
pub struct SensorObserver {
    /// 传感器角度输入
    pub sensor_angle: f64,

    pub pll_kp: f64,
    pub pll_ki: f64,
    pub pll_angle: f64,
    pub pll_speed: f64,

    /// 磁极对数
    pub pole_pairs: f64,
}

impl<const P: usize> Observer<P> for SensorObserver {
    fn update(&mut self, delta_time: f64, _input: &ObserverInput<P>) -> ObserverOutput {
        let new_angle = self.pll_angle + self.pll_speed * delta_time;
        let error = angle_normal(self.sensor_angle - new_angle);
        self.pll_angle = new_angle + self.pll_kp * error * delta_time;
        self.pll_speed += self.pll_ki * error * delta_time;

        ObserverOutput {
            electrical_angle: angle_normal(self.pll_angle * self.pole_pairs),
            electrical_speed: self.pll_speed * self.pole_pairs,
        }
    }
}
