use std::fmt::{Debug, Formatter};
use crate::simulation::nn;
use crate::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};
use crate::util::{BoxedFunction, Function};

/// 模拟真实的两电平功率桥
/// 具有一个整流桥对输入电压进行全波整流
/// 然后通过母线电容器维持母线电压
/// 随后具有指定死区特性的半桥输出
/// 输出电压被限制在正负母线电压的一半
/// 这意味着使用者需要自行调整共模电压
#[derive(Debug, Clone)]
pub struct TwoLevelPowerBridge<const P: usize> {
    /// 载波频率
    /// 并不会实际模拟载波
    /// 仅用于死区计算
    pub carry_freq: f64,
    /// 输入电压
    pub power_input: f64,

    /// 母线电容
    pub bus_capacitance: f64,

    pub braking_current: f64,

    /// 当前母线电压
    pub bus_voltage: f64,

    /// 输出在电机上的母线电流
    pub bus_current: f64,

    /// 死区特性
    /// 根据流入电机方向上的电流生成该相死区电压
    pub dead_mapping: BoxedFunction,

    pub dead_voltage: [f64; P],
}

impl<const P: usize> Default for TwoLevelPowerBridge<P> {
    fn default() -> Self {
        Self {
            carry_freq: 0.0,
            power_input: 0.0,
            bus_capacitance: 0.0,
            braking_current: 0.0,
            bus_voltage: 0.0,
            bus_current: 0.0,
            dead_mapping: Default::default(),
            dead_voltage: [0.0; P],
        }
    }
}

impl<const P: usize> PowerBridge<P> for TwoLevelPowerBridge<P> {
    fn update(&mut self, delta_time: f64, input: &PowerBridgeInput<P>) -> PowerBridgeOutput<P> {
        let power_input = self.power_input.abs();
        if self.bus_voltage < power_input {
            self.bus_voltage = power_input;
        }
        let half_bus = self.bus_voltage * 0.5;
        let mut bus_current = 0.0;

        let mut output_voltage = [0.0; P];
        for i in 0..P {
            self.dead_voltage[i] = self.dead_mapping.eval(input.output_current[i])
                * self.bus_voltage
                * self.carry_freq;
            output_voltage[i] =
                (input.output_duty[i] * half_bus + self.dead_voltage[i]).clamp(-half_bus, half_bus);
            bus_current += input.output_current[i] * output_voltage[i];
        }
        self.bus_current = nn(bus_current / half_bus);
        if self.bus_voltage > power_input {
            self.bus_voltage -=
                (self.bus_current + self.braking_current) / self.bus_capacitance * delta_time;
        }
        if self.bus_voltage < power_input {
            self.bus_voltage = power_input;
        }

        PowerBridgeOutput {
            output_voltage,
            bus_voltage: self.bus_voltage,
        }
    }
}
