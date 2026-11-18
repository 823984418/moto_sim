pub mod any_observer;
pub mod base_flux_observer;
pub mod direct_flux_observer;
pub mod ds_observer;
pub mod ex_flux_observer;
pub mod flux_circle_observer;
pub mod flux_observer;
pub mod grad_observer;
pub mod kasa_flux_observer;
pub mod mix_observer;
pub mod mp_observer;
pub mod sensor_observer;
pub mod ss2_observer;
pub mod ss_observer;
pub mod super_flux_observer;

#[derive(Debug, Clone)]
pub struct ObserverInput<const P: usize> {
    pub voltage: [f64; P],
    pub current: [f64; P],
    /// 零矢量（下桥臂全通）期间测得的电流斜率 di/dt，单位 A/s。
    ///
    /// 与 `current` 必须对应同一有效时刻（两次采样的中点），否则 `R·di/dt·δt`
    /// 的时序错配会被 `R` 放大约两个数量级（见 PAPER.md §3.1）。
    pub current_slope: [f64; P],
    /// 本次斜率测量实际可用的采样间隔 Δt（秒），即零矢量时长减去稳定延时。
    ///
    /// 它有两个用途：(1) 作为斜率噪声的尺度（σ_s ∝ 1/Δt）；(2) 作为圆拟合的
    /// 权重（w ∝ Δt²）。为 0 表示本拍没有可用的零矢量窗口，观测器应跳过。
    pub current_slope_dt: f64,
    pub target_speed: f64,
}

impl<const P: usize> Default for ObserverInput<P> {
    fn default() -> Self {
        Self {
            voltage: [0.0; P],
            current: [0.0; P],
            current_slope: [0.0; P],
            current_slope_dt: 0.0,
            target_speed: 0.0,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ObserverOutput {
    /// 电角度估计
    pub electrical_angle: f64,
    /// 无静差速度
    pub electrical_speed: f64,
    /// 用于同步旋转电流环的速度
    pub continuous_speed: f64,
}

pub trait Observer<const P: usize> {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<P>) -> ObserverOutput;
}
