use crate::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};

/// 理想的功率桥
/// 直接输出指令占空比对应电压
#[derive(Debug, Default, Clone)]
pub struct IdealPowerBridge {
    pub bus_voltage: f64,
}

impl<const P: usize> PowerBridge<P> for IdealPowerBridge {
    fn update(&mut self, _delta_time: f64, input: &PowerBridgeInput<P>) -> PowerBridgeOutput<P> {
        PowerBridgeOutput {
            output_voltage: input.output_duty.map(|i| i * 0.5 * self.bus_voltage),
            bus_voltage: self.bus_voltage,
        }
    }
}
