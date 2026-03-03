use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};

pub struct FluxObserver {
    pub rs: f64,

    pub inductance_dq: [f64; 2],

    pub flux: f64,
}

impl Observer<3> for FluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        todo!()
    }
}
