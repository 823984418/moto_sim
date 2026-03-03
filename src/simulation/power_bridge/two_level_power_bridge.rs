pub struct TwoLevelPowerBridge<const P: usize> {
    /// 母线电压
    pub bus_voltage: f64,

    /// 母线电容
    pub bus_capacitance: f64,
}
