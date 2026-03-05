pub mod ds_observer;
pub mod flux_observer;
pub mod grad_observer;
pub mod mix_observer;
pub mod mp_observer;
pub mod sensor_observer;

#[derive(Debug, Clone)]
pub struct ObserverInput<const P: usize> {
    pub voltage: [f64; P],
    pub current: [f64; P],
}

impl<const P: usize> Default for ObserverInput<P> {
    fn default() -> Self {
        Self {
            voltage: [0.0; P],
            current: [0.0; P],
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ObserverOutput {
    /// 电角度估计
    pub electrical_angle: f64,
    /// 无静差速度
    pub electrical_speed: f64,
    /// 用于同步旋转电流环的速度
    pub continuous_speed: f64,
}

pub trait Observer<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<P>) -> ObserverOutput;
}
