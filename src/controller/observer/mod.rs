pub mod flux_observer;
pub mod sensor_observer;

pub struct ObserverInput<const P: usize> {
    pub voltage: [f64; P],
    pub current: [f64; P],
}

pub struct ObserverOutput {
    pub electrical_angle: f64,
    pub electrical_speed: f64,
}

pub trait Observer<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<P>) -> ObserverOutput;
}
