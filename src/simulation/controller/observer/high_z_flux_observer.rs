//! 高阻（开路）输出下的磁链圆观测器。
//!
//! 硬件上高阻态不是一直开路：每个载波周期里约 **20%** 的时间导通下管，
//! 用一个零矢量给自举电容充电，同时采样电流斜率
//!
//! ```text
//!   s0 = di/dt | v=0 = -(Rs·i + ω×λ)/L
//! ```
//!
//! 其余时间三相高阻，电流被续流二极管经母线钳回 0。电流平均为 0，所以：
//!
//! * 不需要 `V = ∫i dt`，也不需要辨识 `k = Rs/L`（没有电阻项）；
//! * 磁链由斜率积分得到：`p = ∫s0 dt / unit_len`，在 αβ 平面是一个圆，
//!   圆心是积分常数，半径是 `ψf / (L·unit_len)`；
//! * 不需要 Newton / 变量投影 / 可观性加权；
//! * 圆拟合只求**圆心**，半径可用先验 `radius_prior`，或用拟合半径并夹在
//!   `[radius_min, radius_max]` 内；
//! * 斜率积分器带泄漏（`hp_rel·tau`，一阶高通），抑制斜率直流偏置造成的
//!   积分漂移；`T_hp` 远大于 `1/ω` 时对磁链圆本身几乎无影响。
//!
//! 角度约定与 `direct_flux_observer` 一致：`θ = atan2(-(p - c))`。
//!
//! 本模块与 `direct_flux_observer` 完全独立：不复用它的 36 矩、`Coef`、
//! `Tracker` 或 `V2`，只实现一个 9 个矩的二维 Kåsa 圆拟合。
//!
//! 仿真里斜率的噪声按 `σ_s = √2·σ_i / slope_dt` 注入，`slope_dt` 是 20%
//! 下管窗口扣掉稳定延时后的长度；电流本身被理想化为 0（脉冲的平均电流和
//! 转矩可以忽略）。

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke};

/// 标量类型。
pub type S = f64;

/// 平面二维向量（本模块独立的小类型）。
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: S,
    pub y: S,
}

impl V2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: S, y: S) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }

    #[inline]
    pub fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }

    #[inline]
    pub fn mul(self, s: S) -> Self {
        Self::new(self.x * s, self.y * s)
    }

    #[inline]
    pub fn norm2(self) -> S {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn norm(self) -> S {
        self.norm2().sqrt()
    }

    #[inline]
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

/// 二维代数圆拟合需要的 9 个加权矩。
///
/// 残差是 `α + D·u + E·v + F`，其中 `α = |p|²`；法方程是 3×3：
///
/// ```text
///   [Σu²  Σuv  Σu ] [D]   [-Σuα]
///   [Σuv  Σv²  Σv ] [E] = [-Σvα]
///   [Σu   Σv   Σw ] [F]   [-Σα ]
/// ```
#[derive(Clone, Copy, Debug, Default)]
struct Moments {
    /// Σw
    w: S,
    /// Σwu、Σwv
    ux: S,
    uy: S,
    /// Σwu²、Σwuv、Σwv²
    ux2: S,
    ux_uy: S,
    uy2: S,
    /// Σwα
    a: S,
    /// Σwuα、Σwvα
    a_ux: S,
    a_uy: S,
}

impl Moments {
    #[inline]
    fn decay(&mut self, rho: S) {
        self.w *= rho;
        self.ux *= rho;
        self.uy *= rho;
        self.ux2 *= rho;
        self.ux_uy *= rho;
        self.uy2 *= rho;
        self.a *= rho;
        self.a_ux *= rho;
        self.a_uy *= rho;
    }

    #[inline]
    fn add(&mut self, p: V2, q: S) {
        let a = p.norm2();
        self.w += q;
        self.ux += q * p.x;
        self.uy += q * p.y;
        self.ux2 += q * (p.x * p.x);
        self.ux_uy += q * (p.x * p.y);
        self.uy2 += q * (p.y * p.y);
        self.a += q * a;
        self.a_ux += q * (a * p.x);
        self.a_uy += q * (a * p.y);
    }

    /// 解 3×3 法方程，返回圆心 `-(D,E)/2` 和拟合半径 `sqrt(|c|² - F)`。
    fn fit(&self) -> Option<(V2, S)> {
        if !(self.w > 0.0) || !self.w.is_finite() {
            return None;
        }
        let (a00, a01, a02) = (self.ux2, self.ux_uy, self.ux);
        let (a11, a12) = (self.uy2, self.uy);
        let a22 = self.w;
        let det = a00 * (a11 * a22 - a12 * a12) - a01 * (a01 * a22 - a12 * a02)
            + a02 * (a01 * a12 - a11 * a02);
        // 相对量级：所有元素都随 w 缩放，用 w³ 做尺度。
        let scale = self.w * self.w * self.w;
        if !det.is_finite() || det.abs() <= 1.0e-14 * scale {
            return None;
        }
        let (b0, b1, b2) = (-self.a_ux, -self.a_uy, -self.a);
        let d = b0 * (a11 * a22 - a12 * a12) - a01 * (b1 * a22 - a12 * b2)
            + a02 * (b1 * a12 - a11 * b2);
        let e = a00 * (b1 * a22 - b2 * a12) - b0 * (a01 * a22 - a12 * a02)
            + a02 * (a01 * b2 - b1 * a02);
        let f = (a00 * (a11 * b2 - b1 * a12) - a01 * (a01 * b2 - b1 * a02)
            + b0 * (a01 * a12 - a11 * a02))
            / det;
        let c = V2::new(-0.5 * d / det, -0.5 * e / det);
        if !c.finite() {
            return None;
        }
        let r2 = c.norm2() - f;
        if !(r2 > 0.0) || !r2.is_finite() {
            return None;
        }
        Some((c, r2.sqrt()))
    }
}

/// 高阻输出下的磁链圆观测器。
///
/// 每个 `update` 把电流斜率积分进 `p`（快速环），每 `fit_period` 用一次
/// 二维 Kåsa 拟合更新圆心；半径可用先验或拟合范围，不做 Newton。
#[derive(Clone, Debug)]
pub struct HighZFluxObserver {
    /// 内部长度单位（A）。取 `ψf/L`（额定半径）时 `radius_prior = 1`。
    pub unit_len: S,
    /// 先验半径（内部单位）。
    pub radius_prior: S,
    /// `true`：半径固定用先验；`false`：用拟合半径并夹在 `[radius_min, radius_max]`。
    pub use_prior_radius: bool,
    /// `use_prior_radius = false` 时拟合半径的允许范围（内部单位）。
    pub radius_min: S,
    pub radius_max: S,
    /// 拟合周期（秒）。
    pub fit_period: S,
    /// 矩的指数遗忘时间常数（秒）；`<= 0` 表示不遗忘。
    pub tau: S,
    /// 测速低通时间常数（秒）。
    pub speed_tau: S,
    /// 斜率积分器（`p`）的泄漏时间常数系数：`T_hp = hp_rel·tau`。
    ///
    /// `p ← e^{-Δt/T_hp}·(p + s0·Δt/unit_len)`，等效一阶高通，防止斜率
    /// 直流偏置/积分漂移把圆心推走。`T_hp` 必须远大于 `1/ω`（否则会把磁链
    /// 圆本身衰减/移相）；`<= 0` 或 `tau <= 0` 表示不泄漏。
    pub hp_rel: S,
    /// 允许输出角度所需的最小 `Σw`。
    pub min_weight: S,

    /// 本拟合周期内的斜率积分（内部单位）。
    u: V2,
    /// 本拟合周期内有效斜率样本的权重和。
    q_acc: S,
    /// 上次拟合时刻。
    t_last: S,
    fit_accum: S,
    m: Moments,
    center: V2,
    /// 上一拍的输出角（测速用）。
    theta_prev: S,

    // ---- 输出 / 诊断 ----
    /// 拟合圆心相对**最新样本**的向量（内部单位）：`center - p`。
    ///
    /// 与 `direct_flux_observer` 的交班量 `c_local = c - p_local` 是同一个
    /// 定义；高阻没有快速环交班，但也保留这个量，供画图/上层使用。
    pub center_local: V2,
    pub radius: S,
    pub theta: S,
    pub omega: S,
    pub valid: bool,
    pub weight: S,
    pub updates: usize,
    pub fits: usize,
    last: ObserverOutput,
}

impl Default for HighZFluxObserver {
    fn default() -> Self {
        Self::new(1.0, 1.0)
    }
}

impl HighZFluxObserver {
    /// `unit_len`：内部长度单位（A）；`radius_prior`：先验半径（内部单位）。
    ///
    /// 例：`HighZFluxObserver::new(motor.flux / mean_inductance, 1.0)`。
    pub fn new(unit_len: S, radius_prior: S) -> Self {
        Self {
            unit_len: if unit_len.is_finite() && unit_len > 0.0 {
                unit_len
            } else {
                1.0
            },
            radius_prior,
            use_prior_radius: true,
            radius_min: 0.5 * radius_prior.abs(),
            radius_max: 1.5 * radius_prior.abs(),
            fit_period: 0.01,
            tau: 1.0,
            speed_tau: 0.02,
            hp_rel: 200.0,
            min_weight: 8.0,
            u: V2::ZERO,
            q_acc: 0.0,
            t_last: 0.0,
            fit_accum: 0.0,
            m: Moments::default(),
            center: V2::ZERO,
            theta_prev: 0.0,
            center_local: V2::ZERO,
            radius: radius_prior,
            theta: 0.0,
            omega: 0.0,
            valid: false,
            weight: 0.0,
            updates: 0,
            fits: 0,
            last: ObserverOutput::default(),
        }
    }

    /// 清空全部状态（切换到本观测器时调用）。
    ///
    /// 只保留配置（`unit_len`、`radius_prior`、`fit_period`、`tau`、
    /// `speed_tau`、`hp_rel`、`min_weight`、半径策略）。
    pub fn reset(&mut self) {
        self.u = V2::ZERO;
        self.q_acc = 0.0;
        self.t_last = 0.0;
        self.fit_accum = 0.0;
        self.m = Moments::default();
        self.center = V2::ZERO;
        self.center_local = V2::ZERO;
        self.theta_prev = 0.0;
        self.radius = self.radius_prior;
        self.theta = 0.0;
        self.omega = 0.0;
        self.valid = false;
        self.weight = 0.0;
        self.updates = 0;
        self.fits = 0;
        self.last = ObserverOutput::default();
    }

    /// 当前斜率积分（内部单位，`p = ∫s0 dt / unit_len`）。
    #[inline]
    pub fn flux(&self) -> V2 {
        self.u
    }

    /// 当前拟合圆心（内部单位）。
    #[inline]
    pub fn center(&self) -> V2 {
        self.center
    }

    /// 上一次报告。
    #[inline]
    pub fn last_output(&self) -> &ObserverOutput {
        &self.last
    }
}

impl Observer<3> for HighZFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        if !(delta_time > 0.0) || !delta_time.is_finite() {
            self.valid = false;
            return self.last.clone();
        }

        // 快速环：只在有有效零矢量斜率时积分。`current_slope` 是 v=0 时的
        // di/dt，高阻下 i≈0，所以它就是磁链的导数（差一个 -1/L 的符号）。
        let slope_ok = input.current_slope_dt > 0.0 && input.current_slope_dt.is_finite();
        if slope_ok {
            let s = clarke(input.current_slope);
            let s = V2::new(s[0], s[1]);
            if s.finite() {
                // 泄漏积分（一阶高通）：只保留 `p`，泄漏掉的分量不保存。
                let hp = if self.hp_rel > 0.0 && self.tau > 0.0 {
                    (-delta_time / (self.hp_rel * self.tau)).exp()
                } else {
                    1.0
                };
                self.u = self.u.add(s.mul(delta_time / self.unit_len)).mul(hp);
                // 斜率噪声 σ_s ∝ 1/slope_dt，所以按 slope_dt² 加权（和直接
                // 观测器的 q = (slope_dt/Δt)² 同一套）。
                let t = input.current_slope_dt / delta_time;
                self.q_acc += (t * t).clamp(1.0e-6, 1.0e3);
            }
        }
        self.updates += 1;

        // 低速环：每 fit_period 加一个矩样本、拟合一次圆心。
        self.fit_accum += delta_time;
        if self.fit_accum >= self.fit_period {
            self.fit_accum = (self.fit_accum - self.fit_period).max(0.0);
            let rho = if self.tau > 0.0 {
                (-self.fit_period / self.tau).exp()
            } else {
                1.0
            };
            self.m.decay(rho);
            if self.q_acc > 0.0 {
                self.m.add(self.u, self.q_acc);
                self.q_acc = 0.0;
            }
            self.fits += 1;
            if let Some((c, r_fit)) = self.m.fit() {
                self.center = c;
                self.radius = if self.use_prior_radius {
                    self.radius_prior
                } else {
                    r_fit.clamp(self.radius_min, self.radius_max)
                };
            }
            self.weight = self.m.w;
        }

        // 与最新样本的相对圆心：c - p（和 direct_flux 的 c_local 定义一致）。
        self.center_local = self.center.sub(self.u);

        // 角度：rel = p - center，θ = atan2(-rel.y, -rel.x)
        let rel = self.u.sub(self.center);
        let theta = (-rel.y).atan2(-rel.x);
        let dtheta = angle_normal(theta - self.theta_prev);
        self.theta_prev = theta;
        let alpha = if self.speed_tau > 0.0 {
            (delta_time / self.speed_tau).clamp(0.0, 1.0)
        } else {
            1.0
        };
        self.omega += alpha * (dtheta / delta_time - self.omega);

        self.theta = theta;
        self.valid = self.weight >= self.min_weight && theta.is_finite();
        let out = ObserverOutput {
            electrical_angle: theta,
            electrical_speed: self.omega,
            continuous_speed: self.omega,
        };
        self.last = out.clone();
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::inverse_clarke;

    /// 合成零矢量斜率：`s0 = -ω·ψf/L·[-sin, cos]`（i = 0），返回三相斜率。
    ///
    /// 这里 `radius = ψf/L`，所以 `p` 的圆半径就是 `radius`。
    fn synth_slope(omega: S, radius: S, theta0: S, duree: S, dt: S) -> Vec<[S; 3]> {
        let n = (duree / dt) as usize;
        let mut out = Vec::with_capacity(n);
        let mut theta = theta0;
        for _ in 0..n {
            let s_ab = [-omega * radius * -theta.sin(), -omega * radius * theta.cos()];
            out.push(inverse_clarke(s_ab));
            theta += omega * dt;
        }
        out
    }

    fn run(
        ob: &mut HighZFluxObserver,
        stream: &[[S; 3]],
        from: usize,
        to: usize,
        dt: S,
        theta0: S,
        omega: S,
    ) -> S {
        let mut theta = theta0 + omega * (from as S) * dt;
        for e in &stream[from..to] {
            let input = ObserverInput {
                current_slope: *e,
                current_slope_dt: 9.5e-6,
                ..Default::default()
            };
            ob.update(dt, &input);
            theta += omega * dt;
        }
        angle_normal(theta - omega * dt)
    }

    #[test]
    fn recovers_angle_and_radius_with_prior() {
        let (omega, radius, theta0, dt) = (100.0, 1.0, 0.7, 5.0e-5);
        let stream = synth_slope(omega, radius, theta0, 1.0, dt);
        let mut ob = HighZFluxObserver::new(radius, 1.0);
        ob.fit_period = 0.01;
        ob.tau = 0.5;
        ob.min_weight = 8.0;

        let theta_true = run(&mut ob, &stream, 0, stream.len(), dt, theta0, omega);
        eprintln!(
            "high-z: theta={:.4} true={:.4} radius={:.4} center={:?} w={:.0}",
            ob.theta, theta_true, ob.radius, ob.center, ob.weight
        );
        assert!(ob.valid, "observer should be valid");
        assert!(
            (ob.radius - 1.0).abs() < 1.0e-9,
            "半径应先验为 1，实际 {}",
            ob.radius
        );
        assert!(
            angle_normal(ob.theta - theta_true).abs() < 0.02,
            "angle error {}",
            angle_normal(ob.theta - theta_true)
        );
        // p 的圆半径应该 ≈ 1（内部单位）。
        let r = ob.flux().sub(ob.center()).norm();
        assert!((r - 1.0).abs() < 0.01, "|p - c| = {r}");
        // 相对圆心：center_local = c - p，长度同样 ≈ 1。
        assert!(
            ob.center_local.sub(ob.center().sub(ob.flux())).norm() < 1.0e-12,
            "center_local 定义不对"
        );
        assert!((ob.center_local.norm() - 1.0).abs() < 0.01);
    }

    #[test]
    fn reset_clears_state_and_reconverges() {
        let (omega, radius, theta0, dt) = (100.0, 1.0, 0.3, 5.0e-5);
        let stream = synth_slope(omega, radius, theta0, 1.0, dt);
        let mut ob = HighZFluxObserver::new(radius, 1.0);
        ob.tau = 0.5;
        let _ = run(&mut ob, &stream, 0, 4000, dt, theta0, omega);
        assert!(ob.valid);
        ob.reset();
        assert!(!ob.valid);
        assert_eq!(ob.flux(), V2::ZERO);
        assert_eq!(ob.weight, 0.0);
        assert_eq!(ob.fits, 0);
        let theta_end = run(&mut ob, &stream, 4000, 8000, dt, theta0, omega);
        eprintln!(
            "reset: theta={:.4} true={:.4} err={:.3}deg w={:.0}",
            ob.theta,
            theta_end,
            angle_normal(ob.theta - theta_end).to_degrees(),
            ob.weight
        );
        assert!(ob.valid);
        assert!(angle_normal(ob.theta - theta_end).abs() < 0.02);
    }

    #[test]
    fn no_slope_keeps_fit_finite() {
        let mut ob = HighZFluxObserver::new(1.0, 1.0);
        let input = ObserverInput::default(); // current_slope_dt = 0
        for _ in 0..1000 {
            let out = ob.update(5.0e-5, &input);
            assert!(out.electrical_angle.is_finite());
        }
        assert!(!ob.valid);
        assert_eq!(ob.flux(), V2::ZERO);
    }

    #[test]
    fn prior_radius_is_used_not_fitted() {
        // 先验给 0.5（与真实 1.0 不同）：输出半径必须严格等于先验。
        let (omega, radius, theta0, dt) = (100.0, 1.0, 0.0, 5.0e-5);
        let stream = synth_slope(omega, radius, theta0, 1.0, dt);
        let mut ob = HighZFluxObserver::new(radius, 0.5);
        ob.tau = 0.5;
        run(&mut ob, &stream, 0, stream.len(), dt, theta0, omega);
        assert_eq!(ob.radius, 0.5);
        assert!(ob.valid);

        // 同一批数据，改成“用拟合半径 + 范围”，半径应落在范围内。
        let mut ob = HighZFluxObserver::new(radius, 0.5);
        ob.tau = 0.5;
        ob.use_prior_radius = false;
        ob.radius_min = 0.9;
        ob.radius_max = 1.1;
        run(&mut ob, &stream, 0, stream.len(), dt, theta0, omega);
        eprintln!(
            "fit radius: {:.4} in [{}, {}]",
            ob.radius, ob.radius_min, ob.radius_max
        );
        assert!((0.9..=1.1).contains(&ob.radius));
        assert!((ob.radius - 1.0).abs() < 0.05, "fitted radius {}", ob.radius);
    }
}
