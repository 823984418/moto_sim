use crate::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use crate::simulation::{clarke, inverse_clarke, rotate};

#[derive(Debug, Default, Clone)]
pub struct ThreePhasePICurrentRegulator {
    pub kp: [f64; 2],
    pub ki: [f64; 2],
    pub integrator: [f64; 2],
}

impl CurrentRegulator<3> for ThreePhasePICurrentRegulator {
    fn update(
        &mut self,
        delta_time: f64,
        input: &CurrentRegulatorInput<3>,
    ) -> CurrentRegulatorOutput<3> {
        self.integrator = rotate(self.integrator, input.continuous_speed * delta_time);

        let error_current = rotate(
            clarke([
                input.command_current[0] - input.current[0],
                input.command_current[1] - input.current[1],
                input.command_current[2] - input.current[2],
            ]),
            -input.electrical_angle,
        );
        let value_p = rotate(
            [error_current[0] * self.kp[0], error_current[1] * self.kp[1]],
            input.electrical_angle,
        );
        let value_i = rotate(
            [error_current[0] * self.ki[0], error_current[1] * self.ki[1]],
            input.electrical_angle,
        );

        self.integrator[0] += value_i[0] * delta_time;
        self.integrator[1] += value_i[1] * delta_time;

        let output = [
            value_p[0] + self.integrator[0],
            value_p[1] + self.integrator[1],
        ];

        CurrentRegulatorOutput {
            command_voltage: inverse_clarke(output),
        }
    }
}
