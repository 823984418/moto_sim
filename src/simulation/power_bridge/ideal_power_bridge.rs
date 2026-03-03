use crate::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};

/// 理想的功率桥
#[derive(Debug, Default, Clone)]
pub struct IdealPowerBridge {}

impl<const P: usize> PowerBridge<P> for IdealPowerBridge {
    fn update(&mut self, _delta_time: f64, input: &PowerBridgeInput<P>) -> PowerBridgeOutput<P> {
        PowerBridgeOutput {
            voltage: input.command_voltage,
        }
    }
}
