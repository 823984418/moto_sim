pub mod three_phase_pi_current_regulator;

pub struct CurrentRegulatorInput<const P: usize> {
    pub electrical_speed: f64,
    pub electrical_angle: f64,
    pub current: [f64; P],
    pub command_current: [f64; P],
}

pub struct CurrentRegulatorOutput<const P: usize> {
    pub command_voltage: [f64; P],
}

pub trait CurrentRegulator<const P: usize> {
    fn update(
        &mut self,
        delta_time: f64,
        input: &CurrentRegulatorInput<P>,
    ) -> CurrentRegulatorOutput<P>;
}
