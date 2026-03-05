pub mod three_phase_pi_current_regulator;

#[derive(Debug, Clone)]
pub struct CurrentRegulatorInput<const P: usize> {
    pub electrical_speed: f64,
    pub electrical_angle: f64,
    pub continuous_speed: f64,
    pub current: [f64; P],
    pub command_current: [f64; P],
}

impl<const P: usize> Default for CurrentRegulatorInput<P> {
    fn default() -> Self {
        Self {
            electrical_speed: 0.0,
            electrical_angle: 0.0,
            continuous_speed: 0.0,
            current: [0.0; P],
            command_current: [0.0; P],
        }
    }
}

#[derive(Debug, Clone)]
pub struct CurrentRegulatorOutput<const P: usize> {
    pub command_voltage: [f64; P],
}

impl<const P: usize> Default for CurrentRegulatorOutput<P> {
    fn default() -> Self {
        Self {
            command_voltage: [0.0; P],
        }
    }
}

pub trait CurrentRegulator<const P: usize> {
    fn update(
        &mut self,
        delta_time: f64,
        input: &CurrentRegulatorInput<P>,
    ) -> CurrentRegulatorOutput<P>;
}
