pub mod ideal_power_bridge;
pub mod two_level_power_bridge;

#[derive(Debug, Clone)]
pub struct PowerBridgeInput<const P: usize> {
    pub output_duty: [f64; P],
    pub output_current: [f64; P],
}

impl<const P: usize> Default for PowerBridgeInput<P> {
    fn default() -> Self {
        Self {
            output_duty: [0.0; P],
            output_current: [0.0; P],
        }
    }
}

#[derive(Debug, Clone)]
pub struct PowerBridgeOutput<const P: usize> {
    pub output_voltage: [f64; P],
    pub bus_voltage: f64,
}

impl<const P: usize> Default for PowerBridgeOutput<P> {
    fn default() -> Self {
        Self {
            output_voltage: [0.0; P],
            bus_voltage: 0.0,
        }
    }
}

pub trait PowerBridge<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &PowerBridgeInput<P>) -> PowerBridgeOutput<P>;
}
