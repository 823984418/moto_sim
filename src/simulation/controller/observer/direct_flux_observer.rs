//! 直接磁链积分观测器：电流环频率上做反正切，低速环上做圆拟合。
//!
//! 与 [`KasaFluxObserver`](super::kasa_flux_observer::KasaFluxObserver) 的区别：
//!
//! * **没有 PLL**。角度就是快速环的
//!   `rel = c + ∫s0 + R·∫i`、`θ = atan2(-rel.y, -rel.x)`；
//!   转速只对 `Δθ/Δt` 做一阶低通，不构成闭环。
//! * **双速率**。`∫s0`、`∫i` 与测角在每次 `update`（电流环频率）；
//!   圆拟合每 `fit_period`（默认 10 ms）才取一次快速积分器的增量。
//! * **每次拟合恰好一次 Newton**。所有门限都做成**权重**，没有硬截断：
//!   可观性 `w_obs = clamp(|obs|/obs_min,0,1)²`、噪声可信度
//!   `w_sigma = clamp((σ_max/σ_rel)²,0,1)`、统计显著性
//!   `w_sig = δ²/(δ²+σ_k²)`，其中 `σ_k = sqrt(2P/((N_eff-3)P''))`、
//!   `N_eff = (Σw)²/Σw²`。`obs_min`/`σ_max` 是权重尺度，不是“过/不过”的门限。
//! * **每个拟合样本都把最新点搬到原点**（`U`、`V` 各一次精确平移），
//!   拟合圆心因此总是相对“最新点”。
//! * **加锁交班**：`Arc<Mutex<FastFlux>>` 内读并清零 `a`、`b`，同时
//!   `c += a + k_cur·b` 保持磁链连续；拟合完成后写三个目标
//!   `c_tgt = -c_local`、`k_tgt = k_fit`、`rad_tgt = radius_fit`，再让
//!   `c`、`k`、`rad` **都**按同一个增益 `sync_gain` 向各自目标滑动
//!   （可选再乘可观性权重，默认关，见 `sync_obs_weighted`）。
//!   这样积分器清零不会让角度跳变，三个量也不会一次写死。

use super::kasa_flux_observer::{wrap_pi, FitConfig, Tracker, V2, S};
use super::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::clarke;
use std::sync::{Arc, Mutex};

/// 快速环（电流环频率）状态：两个积分器 + 当前相对圆心/半径/R + 低通测速。
#[derive(Clone, Copy, Debug, Default)]
struct FastFlux {
    /// `∫ s0 dt`（内部单位），每个拟合周期清零。
    a: V2,
    /// `∫ i dt`（内部单位），每个拟合周期清零。
    b: V2,
    /// 相对圆心：磁链相对圆心的偏移为 `c + a + k b`。
    c: V2,
    /// 当前 `R = Rs/L`（向拟合值滑动）。
    k: S,
    /// 当前半径（内部单位，向拟合值滑动）。
    rad: S,
    /// 上一拍的反正切角（测速用）。
    theta_prev: S,
    /// 低通后的电角速度。
    omega_lp: S,
    /// 本拟合周期内累积的 `Σ(slope_dt/Δt)²`。
    q_acc: S,
}

/// 直接磁链积分观测器。
#[derive(Clone, Debug)]
pub struct DirectFluxObserver {
    pub fit: FitConfig,
    /// 圆拟合周期（秒）。
    ///
    /// 10 ms 是刻意取慢的：快速环仍有 20 kHz 的角度，拟合只负责 `R` 与圆心；
    /// 每个周期内 `a`、`b` 的增量由快速环精确积分，离散化误差只在矩更新。
    pub fit_period: S,
    /// 测速低通时间常数（秒）。
    pub speed_tau: S,
    /// 圆心、半径、`R` 每次拟合向拟合值移动的比例。
    ///
    /// 三者共用同一个增益，保证交班是同一个机制；`0.05` 对应约 20 个拟合
    /// 周期（200 ms）收敛。拟合本身的 Newton 步已经用可观性加权，所以这里
    /// 默认不再乘 `obs`：否则 `obs→0` 时快速环会停在**过时的** `R`/圆心，
    /// 与拟合的圆不一致（实测 1.25× 初值会卡在 +10% 并造成 π 角度偏置）。
    pub sync_gain: S,
    /// 交班是否再乘可观性权重（默认关；打开做 A/B 对照）。
    pub sync_obs_weighted: bool,
    /// 半径范围的相对尺度（相对 `set_initial` 的 `radius`）。
    pub radius_scale: (S, S),
    /// `k` 范围的相对尺度（相对 `set_initial` 的 `r`）。
    pub k_scale: (S, S),
    /// `|rel|/radius` 的可信区间。
    pub mag_lo: S,
    pub mag_hi: S,

    // ---- 输出 / 诊断 ----
    /// 快速环当前使用的 `R`（滑动后的值）。
    pub r: S,
    /// 快速环当前使用的半径（滑动后的值，用户单位）。
    pub radius: S,
    /// 本次拟合给出的 `R` 目标值。
    pub r_fit: S,
    /// 本次拟合给出的半径目标值（用户单位）。
    pub radius_fit: S,
    pub theta: S,
    pub omega: S,
    pub valid: bool,
    pub theta_meas: S,
    pub flux_mag: S,
    pub obs: S,
    pub sigma_rel: S,
    /// `k` 的标准误（诊断/调参用）。
    pub sigma_k: S,
    pub steps: u32,
    pub weight: S,
    pub data_arc: S,
    /// 拟合圆心（绝对坐标，诊断用）。
    pub center: V2,
    /// 拟合圆心（内部单位，相对最新样本；交班目标就是它的相反数）。
    pub center_local: V2,
    pub r_nominal: S,
    pub radius_nominal: S,
    pub radius_clamped: bool,
    pub fit_updates: usize,
    pub updates: usize,

    tracker: Option<Tracker>,
    unit_len: S,
    fit_accum: S,
    total_time: S,
    fast: Arc<Mutex<FastFlux>>,
    last: ObserverOutput,
}

impl Default for DirectFluxObserver {
    fn default() -> Self {
        Self {
            // 直接观测器不需要随机游走先验：步长只由可观性加权。
            fit: FitConfig {
                k_prior_rel: 0.0,
                ..FitConfig::default()
            },
            fit_period: 0.01,
            speed_tau: 0.02,
            sync_gain: 0.05,
            sync_obs_weighted: false,
            radius_scale: (0.1, 10.0),
            k_scale: (0.1, 10.0),
            mag_lo: 0.3,
            mag_hi: 3.0,
            r: 0.0,
            radius: 0.0,
            r_fit: 0.0,
            radius_fit: 0.0,
            theta: 0.0,
            omega: 0.0,
            valid: false,
            theta_meas: 0.0,
            flux_mag: 0.0,
            obs: 0.0,
            sigma_rel: 0.0,
            sigma_k: 0.0,
            steps: 0,
            weight: 0.0,
            data_arc: 0.0,
            center: V2::ZERO,
            center_local: V2::ZERO,
            r_nominal: 0.0,
            radius_nominal: 0.0,
            radius_clamped: false,
            fit_updates: 0,
            updates: 0,
            tracker: None,
            unit_len: 1.0,
            fit_accum: 0.0,
            total_time: 0.0,
            fast: Arc::new(Mutex::new(FastFlux::default())),
            last: ObserverOutput::default(),
        }
    }
}

impl DirectFluxObserver {
    pub fn new() -> Self {
        Self::default()
    }

    /// 写入启动（ALIGN / 交班）时刻的状态。
    ///
    /// `t = 0` 时 `p = 0`、圆心为 `radius·n(theta0)`，所以相对圆心初值是
    /// `c = -radius·n(theta0)`（内部单位）。
    pub fn set_initial(&mut self, theta: S, omega: S, r: S, radius: S) {
        self.theta = wrap_pi(theta);
        self.r = r;
        self.radius = radius;
        self.r_nominal = r;
        self.radius_nominal = radius;
        self.r_fit = r;
        self.radius_fit = radius;

        let mut cfg = self.fit;
        cfg.r_min = self.radius_scale.0 * radius;
        cfg.r_max = self.radius_scale.1 * radius;
        cfg.k_min = self.k_scale.0 * r;
        cfg.k_max = self.k_scale.1 * r;
        self.fit = cfg;
        self.unit_len = 0.5 * (cfg.r_min + cfg.r_max);
        self.tracker = Some(Tracker::new(cfg, r));

        let c0 = V2::new(self.theta.cos(), self.theta.sin()).mul(-radius / self.unit_len);
        if let Ok(mut f) = self.fast.lock() {
            *f = FastFlux {
                c: c0,
                k: self.r,
                rad: self.radius / self.unit_len,
                theta_prev: self.theta,
                omega_lp: omega,
                ..Default::default()
            };
        }

        self.fit_accum = 0.0;
        self.total_time = 0.0;
        self.updates = 0;
        self.fit_updates = 0;
        self.valid = false;
        self.obs = 0.0;
        self.sigma_rel = 0.0;
        self.sigma_k = 0.0;
        self.steps = 0;
        self.weight = 0.0;
        self.data_arc = 0.0;
        self.center = V2::new(self.theta.cos(), self.theta.sin()).mul(radius);
        self.center_local = V2::ZERO;
        self.theta_meas = self.theta;
        self.omega = omega;
        self.flux_mag = radius;
        self.last = ObserverOutput {
            electrical_angle: self.theta,
            electrical_speed: omega,
            continuous_speed: omega,
        };
    }
}

impl Observer<3> for DirectFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        if !(delta_time > 0.0) || !delta_time.is_finite() {
            self.valid = false;
            return self.last.clone();
        }
        self.total_time += delta_time;
        self.updates += 1;
        // 让运行中的 `fit` 改动（例如开/关显著性硬门限）传到 Tracker。
        if let Some(tr) = self.tracker.as_mut() {
            tr.sync_fit(&self.fit);
        }

        let i = clarke(input.current);
        let s = clarke(input.current_slope);
        let i = V2::new(i[0], i[1]);
        let s = V2::new(s[0], s[1]);
        if !i.finite() || !s.finite() {
            self.valid = false;
            return self.last.clone();
        }

        let slope_dt = input.current_slope_dt;
        let slope_ok = slope_dt > 0.0 && slope_dt.is_finite();
        let inv = 1.0 / self.unit_len;

        // ---- 快速环：积分 + 到点则“取走并清零”，同一把锁里保持磁链连续 ----
        let mut take: Option<(V2, V2, S)> = None;
        {
            let mut f = match self.fast.lock() {
                Ok(f) => f,
                Err(_) => {
                    self.valid = false;
                    return self.last.clone();
                }
            };
            if slope_ok {
                f.a = f.a.add(s.mul(delta_time * inv));
                f.b = f.b.add(i.mul(delta_time * inv));
                let q = (slope_dt / delta_time).powi(2).clamp(1.0e-6, 1.0e3);
                f.q_acc += q;
            }

            self.fit_accum += delta_time;
            if self.fit_accum >= self.fit_period {
                self.fit_accum = (self.fit_accum - self.fit_period).max(0.0);
                let a_inc = f.a;
                let b_inc = f.b;
                let q_inc = f.q_acc;
                take = Some((a_inc, b_inc, q_inc));
                f.a = V2::ZERO;
                f.b = V2::ZERO;
                f.q_acc = 0.0;
                // 清零前先把积分值吸收进相对圆心：磁链 = c + a + k·b 不变
                let k_cur = f.k;
                f.c = f.c.add(a_inc).add(b_inc.mul(k_cur));
            }
        }

        // ---- 低速环：一次矩更新 + 一次 Newton；完成后加权交班 ----
        if let Some((a, b, q)) = take {
            if let Some(tr) = self.tracker.as_mut() {
                // 快速环的 a、b 已经是内部单位；`step_once` 吃用户单位。
                let l = self.unit_len;
                if let Some((rep, c_local)) = tr.step_once(a.mul(l), b.mul(l), self.total_time, q)
                {
                    let k_fit = tr.k();
                    let rad_fit = rep.radius / l;
                    let w_obs = (rep.obs / self.fit.obs_min.max(1.0e-12)).clamp(0.0, 1.0);
                    let w = self.sync_gain * if self.sync_obs_weighted { w_obs } else { 1.0 };
                    if let Ok(mut f) = self.fast.lock() {
                        // 目标圆心：让最新样本落在拟合圆上，即 c_tgt = -c_local。
                        // 圆心、半径、R 三者用同一个权重一起滑动。
                        let tgt = c_local.mul(-1.0);
                        f.c = f.c.add(tgt.sub(f.c).mul(w));
                        f.k += (k_fit - f.k) * w;
                        f.rad += (rad_fit - f.rad) * w;
                    }
                    self.r_fit = k_fit;
                    self.radius_fit = rep.radius;
                    self.obs = rep.obs;
                    self.sigma_rel = rep.sigma_rel;
                    self.sigma_k = rep.sigma_k;
                    self.steps = rep.steps;
                    self.weight = rep.weight;
                    self.center = rep.center;
                    self.center_local = c_local;
                    self.radius_clamped = rep.clamped;
                    self.data_arc = if self.radius_nominal > 0.0 {
                        tr.path() * tr.unit_len() / self.radius_nominal
                    } else {
                        0.0
                    };
                    self.fit_updates += 1;
                }
            }
        }

        // ---- 快速环：反正切测角 + 低通差分测速 ----
        let (theta, omega, mag, k_now, rad_now) = {
            let mut f = match self.fast.lock() {
                Ok(f) => f,
                Err(_) => {
                    self.valid = false;
                    return self.last.clone();
                }
            };
            let rel = f.c.add(f.a).add(f.b.mul(f.k));
            let theta = (-rel.y).atan2(-rel.x);
            let dtheta = wrap_pi(theta - f.theta_prev);
            f.theta_prev = theta;
            let alpha = if self.speed_tau > 0.0 {
                (delta_time / self.speed_tau).clamp(0.0, 1.0)
            } else {
                1.0
            };
            f.omega_lp += alpha * (dtheta / delta_time - f.omega_lp);
            (theta, f.omega_lp, rel.norm(), f.k, f.rad)
        };

        self.theta = theta;
        self.theta_meas = theta;
        self.omega = omega;
        self.r = k_now;
        self.radius = rad_now * self.unit_len;
        self.flux_mag = mag * self.unit_len;
        let r_i = rad_now;
        let valid = slope_ok
            && r_i > 0.0
            && mag > self.mag_lo * r_i
            && mag < self.mag_hi * r_i
            && theta.is_finite();
        self.valid = valid;

        let out = ObserverOutput {
            electrical_angle: theta,
            electrical_speed: omega,
            continuous_speed: omega,
        };
        self.last = out.clone();
        out
    }
}

// ===========================================================================
// 测试
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::inverse_clarke;

    /// 与 kasa 的 `synth_stream` 同构，但 `omega` 可以随时间变化；返回真实电角度。
    fn synth_stream_profile(
        r: S,
        radius: S,
        omega: impl Fn(S) -> S,
        iq: impl Fn(S) -> S,
        dur: S,
        dt: S,
    ) -> Vec<(S, V2, V2)> {
        let n = (dur / dt) as usize;
        let (mut a, mut b) = (V2::ZERO, V2::ZERO);
        let mut th: S = 0.0;
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as S * dt;
            let w = omega(t);
            let (sin, cos) = th.sin_cos();
            let jn = V2::new(-sin, cos);
            let i = jn.mul(iq(t));
            let s = i.mul(r).add(jn.mul(w * radius)).mul(-1.0);
            let du = s.mul(dt);
            let dv = i.mul(dt);
            a = a.add(du);
            b = b.add(dv);
            out.push((th, du, dv));
            th += w * dt;
        }
        out
    }

    fn direct_input(du: V2, dv: V2, dt: S) -> ObserverInput<3> {
        ObserverInput {
            voltage: [0.0; 3],
            current: inverse_clarke([dv.x / dt, dv.y / dt]),
            current_slope: inverse_clarke([du.x / dt, du.y / dt]),
            current_slope_dt: dt,
            target_speed: 0.0,
        }
    }

    /// 加速→减速后仍要跟得上，且交班不能让角度跳变。
    #[test]
    fn direct_flux_tracks_speed_profile() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let (lo, hi, ramp) = (20.0, 100.0, 1000.0);
        let omega = move |t: S| {
            let up = (hi - lo) / ramp;
            if t < 1.0 {
                lo
            } else if t < 1.0 + up {
                lo + ramp * (t - 1.0)
            } else if t < 3.0 {
                hi
            } else if t < 3.0 + up {
                hi - ramp * (t - 3.0)
            } else {
                lo
            }
        };
        let stream = synth_stream_profile(r, radius, omega, |_| 3.0, 6.0, dt);

        let mut ob = DirectFluxObserver::new();
        ob.set_initial(0.0, 0.0, r, radius);
        let mut worst_tail: S = 0.0;
        let mut worst_jump: S = 0.0;
        let mut prev: S = 0.0;
        let mut t = 0.0;
        for (th, du, dv) in stream {
            let out = ob.update(dt, &direct_input(du, dv, dt));
            if t > 5.0 {
                worst_tail = worst_tail.max(wrap_pi(out.electrical_angle - th).abs());
            }
            if t > 0.2 {
                worst_jump = worst_jump.max(wrap_pi(out.electrical_angle - prev).abs());
            }
            prev = out.electrical_angle;
            t += dt;
        }
        eprintln!(
            "direct: tail={worst_tail:.4} jump={worst_jump:.4} r={:.3} r_fit={:.3} radius={:.4} obs={:.2e}",
            ob.r, ob.r_fit, ob.radius, ob.obs
        );
        assert!(worst_tail < 0.05, "tail error {worst_tail} rad");
        assert!(worst_jump < 0.15, "angle jump {worst_jump} rad");
    }

    /// R 初值 +25%：只能靠可观性加权 Newton 慢慢拉回；拉回后角度要恢复。
    #[test]
    fn direct_flux_recovers_r_with_weighted_newton() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let omega = move |t: S| {
            if t < 0.02 {
                0.0
            } else if t < 0.12 {
                100.0 * (t - 0.02) / 0.1
            } else {
                100.0
            }
        };
        // 启动瞬态 + 持续的电流幅值变化给 R 提供非相似变换的激励
        let iq = move |t: S| 3.0 + 1.5 * (t * 40.0).sin() * (-t / 2.0).exp();
        let stream = synth_stream_profile(r, radius, omega, iq, 4.0, dt);

        let mut ob = DirectFluxObserver::new();
        // 这个合成流的激励比真实电机弱，把门限调低来验证“可观性加权 Newton”
        // 本身；真实仿真的门限仍是 5e-3。
        ob.fit.obs_min = 1.0e-5;
        ob.set_initial(0.0, 0.0, 1.05 * r, radius);
        let mut t = 0.0;
        let mut tail: S = 0.0;
        for (th, du, dv) in stream {
            let out = ob.update(dt, &direct_input(du, dv, dt));
            if t > 3.0 {
                tail = tail.max(wrap_pi(out.electrical_angle - th).abs());
            }
            t += dt;
        }
        let rel = (ob.r - r).abs() / r;
        eprintln!(
            "direct R: r={:.3} (true {r:.1}) rel={rel:.4} tail={tail:.4} radius={:.4} obs={:.2e} w={:.0}",
            ob.r, ob.radius, ob.obs, ob.weight
        );
        assert!(rel < 0.03, "R rel error {rel}");
        assert!(tail < 0.1, "tail angle error {tail}");
    }

    /// 显著性硬门限可以开关；开启时软权重仍乘在步长上（不是二选一）。
    #[test]
    fn k_sigma_gate_can_be_switched() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let omega = move |t: S| {
            if t < 0.02 {
                0.0
            } else if t < 0.12 {
                100.0 * (t - 0.02) / 0.1
            } else {
                100.0
            }
        };
        let iq = move |t: S| 3.0 + 1.5 * (t * 40.0).sin() * (-t / 2.0).exp();
        let stream = synth_stream_profile(r, radius, omega, iq, 4.0, dt);

        let run = |k_sigma: S| {
            let mut ob = DirectFluxObserver::new();
            ob.fit.obs_min = 1.0e-5;
            ob.fit.k_sigma = k_sigma;
            ob.set_initial(0.0, 0.0, 1.05 * r, radius);
            for (_, du, dv) in &stream {
                ob.update(dt, &direct_input(*du, *dv, dt));
            }
            ob
        };
        // 门限关：只靠软权重
        let soft = run(0.0);
        // 门限开且极大：任何有限步长都会被硬门限挡住
        let gated = run(1.0e9);
        let rel_soft = (soft.r - r).abs() / r;
        let rel_gated = (gated.r - 1.05 * r).abs() / r;
        eprintln!(
            "gate: soft r={:.3} (rel {rel_soft:.4}) | gated r={:.3} (rel {rel_gated:.4}) steps={}",
            soft.r, gated.r, gated.steps
        );
        assert!(rel_soft < 0.02, "soft R rel {rel_soft}");
        assert!(rel_gated < 0.005, "gated R moved: rel {rel_gated}");
        assert_eq!(gated.steps, 0, "gate should block the step");
    }

    /// 最新样本必须位于拟合局部坐标原点（交班的前置条件）。
    #[test]
    fn step_once_puts_latest_sample_at_origin() {
        let (r, radius, dt) = (384.0, 4.04, 5.0e-5);
        let stream = synth_stream_profile(r, radius, |_| 20.0, |_| 3.0, 0.2, dt);
        let mut cfg = FitConfig::default();
        cfg.r_min = 0.3 * radius;
        cfg.r_max = 3.0 * radius;
        cfg.k_min = 0.1 * r;
        cfg.k_max = 10.0 * r;
        cfg.tau = 10.0;
        let mut tr = Tracker::new(cfg, r);
        let (mut a, mut b) = (V2::ZERO, V2::ZERO);
        let (mut t, mut n, mut last) = (0.0, 0, None);
        for (_th, du, dv) in stream {
            a = a.add(du);
            b = b.add(dv);
            t += dt;
            n += 1;
            if n == 200 {
                last = tr.step_once(a, b, t, 200.0);
                a = V2::ZERO;
                b = V2::ZERO;
                n = 0;
            }
        }
        let (rep, c) = last.expect("report");
        eprintln!("origin: p_local={:?} c={:?} radius={:.4}", tr.p_local(), c, rep.radius);
        assert!(tr.p_local().norm() < 1.0e-12, "latest sample not at origin");
        assert!((rep.radius - radius).abs() < 0.01);
    }
}
