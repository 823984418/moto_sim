use crate::simulation::controller::observer::ObserverInput;
use crate::simulation::{clarke, complex_div, complex_mode, nn, rotate};

/// 在线测量电阻
///
#[derive(Debug, Default, Clone)]
pub struct ResTune {
    pub tune_angle: f64,

    pub voltage_lp: [f64; 2],
    pub current_lp: [f64; 2],
    pub input_lp_factor: f64,
    pub output_lp_factor: f64,

    pub output_res: f64,
}

impl ResTune {
    pub fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) {
        let voltage = rotate(clarke(input.voltage), -self.tune_angle);
        let current = rotate(clarke(input.current), -self.tune_angle);

        self.voltage_lp[0] += (voltage[0] - self.voltage_lp[0]) * self.input_lp_factor * delta_time;
        self.voltage_lp[1] += (voltage[1] - self.voltage_lp[1]) * self.input_lp_factor * delta_time;

        self.current_lp[0] += (current[0] - self.current_lp[0]) * self.input_lp_factor * delta_time;
        self.current_lp[1] += (current[1] - self.current_lp[1]) * self.input_lp_factor * delta_time;

        let res = nn(self.voltage_lp[0] / self.current_lp[0]);
        self.output_res += (res - self.output_res) * self.output_lp_factor * delta_time;
    }
}

/// 在线测量平均电感
/// 使用高通滤波器取出输入信号
/// 然后将比值视为电感
/// 按照电流变化率加权滤波
#[derive(Debug, Default, Clone)]
pub struct InductanceTune {
    pub voltage_sample_lp: [f64; 2],
    pub current_sample_lp: [f64; 2],
    pub sample_lp_factor: f64,

    pub voltage_lp: [f64; 2],
    pub p_current_lp: [f64; 2],

    pub last_current: [f64; 2],
    pub input_lp_factor: f64,

    pub l0_power_lp: f64,
    pub power_lp: f64,
    pub output_lp_factor: f64,

    pub output_l0: f64,
}

impl InductanceTune {
    pub fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) {
        let voltage = clarke(input.voltage);
        let current = clarke(input.current);
        self.voltage_sample_lp[0] +=
            (voltage[0] - self.voltage_sample_lp[0]) * self.sample_lp_factor * delta_time;
        self.voltage_sample_lp[1] +=
            (voltage[1] - self.voltage_sample_lp[1]) * self.sample_lp_factor * delta_time;
        self.current_sample_lp[0] +=
            (current[0] - self.current_sample_lp[0]) * self.sample_lp_factor * delta_time;
        self.current_sample_lp[1] +=
            (current[1] - self.current_sample_lp[1]) * self.sample_lp_factor * delta_time;
        let voltage = self.voltage_sample_lp;
        let current = self.current_sample_lp;

        self.voltage_lp[0] += (voltage[0] - self.voltage_lp[0]) * self.input_lp_factor * delta_time;
        self.voltage_lp[1] += (voltage[1] - self.voltage_lp[1]) * self.input_lp_factor * delta_time;
        let p_current = [
            (current[0] - self.last_current[0]) / delta_time,
            (current[1] - self.last_current[1]) / delta_time,
        ];
        self.p_current_lp[0] +=
            (p_current[0] - self.p_current_lp[0]) * self.input_lp_factor * delta_time;
        self.p_current_lp[1] +=
            (p_current[1] - self.p_current_lp[1]) * self.input_lp_factor * delta_time;
        self.last_current = current;

        let voltage_input = [
            voltage[0] - self.voltage_lp[0],
            voltage[1] - self.voltage_lp[1],
        ];

        let p_current_input = [
            p_current[0] - self.p_current_lp[0],
            p_current[1] - self.p_current_lp[1],
        ];
        let p_current_input_len = complex_mode(p_current_input);
        let inductance = complex_div(voltage_input, p_current_input);
        self.l0_power_lp += (inductance[0] * p_current_input_len - self.l0_power_lp)
            * self.output_lp_factor
            * delta_time;

        self.power_lp += (p_current_input_len - self.power_lp) * self.output_lp_factor * delta_time;
        self.output_l0 = nn(self.l0_power_lp / self.power_lp);
    }
}
