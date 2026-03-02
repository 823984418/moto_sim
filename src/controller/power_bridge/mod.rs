pub mod ideal_power_bridge;
pub mod two_level_bridge;

#[derive(Debug, Clone)]
pub struct PowerBridgeInput<const P: usize> {
    pub command: [f64; P],
    pub current: [f64; P],
}

impl<const P: usize> Default for PowerBridgeInput<P> {
    fn default() -> Self {
        Self {
            command: [0.0; P],
            current: [0.0; P],
        }
    }
}

#[derive(Debug, Clone)]
pub struct PowerBridgeOutput<const P: usize> {
    pub voltage: [f64; P],
}

impl<const P: usize> Default for PowerBridgeOutput<P> {
    fn default() -> Self {
        Self { voltage: [0.0; P] }
    }
}

pub trait PowerBridge<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &PowerBridgeInput<P>) -> PowerBridgeOutput<P>;
}
