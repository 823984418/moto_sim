//! 基于零矢量电流斜率积分的磁链 / 电阻观测器。
//!
//! 这是 `pmsm_flux`（见 `PAPER.md`）在 moto_sim 里的实现。核心恒等式为
//!
//! ```text
//!     A(t) = ∫ s0 dt ,  B(t) = ∫ i dt
//!     u(t) = A + R*B = -(λ(t) - λ(0))/L
//! ```
//!
//! 其中 `s0 = di/dt |(v=0)` 是零矢量期间的电流斜率，`R = Rs/L`。
//! 因此 `u` 落在以 `c = λ(0)/L` 为圆心、`radius = ψf/L` 为半径的圆上，
//! 圆上相位就是电角度：`θ = atan2(-(u-c).y, -(u-c).x)`。
//!
//! 本模块只负责“给定零矢量斜率”之后的估计：
//!
//! 1. [`fit`]：加窗圆拟合，同时估计 `R`、圆心、圆心漂移与半径；
//! 2. [`FluxCircleObserver`]：逐拍积分 + 仿射中心 + 二阶 PLL，输出角度与角速度。
//!
//! 斜率本身由 `ObserverInput::current_slope` / `current_slope_dt` 传入。
//! 注意：本仓库的功率桥是平均模型，没有真实载波与开关状态，因此
//! `examples/example1.rs` 里的斜率是用“零矢量 ODE”合成的传感器模型，
//! 不是从真实 PWM 波形双采样得到的。这一点在评估结果时必须记住。

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke};

/// 标量类型。与 PAPER.md 一致，桌面验证用 `f64`。
pub type S = f64;

/// 把角度归一化到 `[-π, π)`。
#[inline]
pub fn wrap_pi(a: S) -> S {
    if !a.is_finite() {
        return 0.0;
    }
    angle_normal(a)
}

// ===========================================================================
// 二维向量
// ===========================================================================

/// 平面二维向量 `(x, y)`。
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
    pub fn dot(self, o: Self) -> S {
        self.x * o.x + self.y * o.y
    }

    #[inline]
    pub fn norm(self) -> S {
        self.dot(self).sqrt()
    }

    #[inline]
    pub fn scaled(self, k: S) -> Self {
        Self::new(self.x * k, self.y * k)
    }

    #[inline]
    pub fn unit(self) -> Self {
        let n = self.norm();
        if n > 1.0e-12 {
            self.scaled(1.0 / n)
        } else {
            Self::new(1.0, 0.0)
        }
    }

    #[inline]
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl std::ops::Add for V2 {
    type Output = Self;
    #[inline]
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }
}

impl std::ops::Sub for V2 {
    type Output = Self;
    #[inline]
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }
}

impl std::ops::Neg for V2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl std::ops::Mul<S> for V2 {
    type Output = Self;
    #[inline]
    fn mul(self, k: S) -> Self {
        Self::new(self.x * k, self.y * k)
    }
}

// ===========================================================================
// 加窗圆拟合
// ===========================================================================

/// 窗口内累积后的一个观测。
///
/// `a`、`b` 都是**相对窗起点**的积分，而不是从开机起的绝对积分：
/// `u = a + R*b` 是两个 1e3 量级数之差、结果只有 1e1 量级，绝对积分会把
/// 有效数字吃光。
#[derive(Clone, Copy, Debug)]
pub struct Obs {
    /// `∫ s0 dt`，从窗起点算起。
    pub a: V2,
    /// `∫ i dt`，从窗起点算起。
    pub b: V2,
    /// 距窗起点的时间（秒）。
    pub t: S,
    /// 拟合权重，正比于 `Δt_zero²`（长零矢量样本更可信）。
    pub w: S,
}

/// 拟合器配置。
#[derive(Clone, Copy, Debug)]
pub struct FitCfg {
    /// Huber 阈值（单位：鲁棒尺度 `sigma`）。
    pub huber: S,
    /// 退火步数：前这么多次迭代用纯最小二乘。
    pub warmup: usize,
    pub iters: usize,
    pub min_obs: usize,
    pub r_max: S,
    pub eps: S,
    /// 是否把圆心漂移 `v = du/dt` 作为冗余参数一起估计。
    pub fit_drift: bool,
}

impl Default for FitCfg {
    fn default() -> Self {
        Self {
            huber: 1.345,
            warmup: 3,
            iters: 25,
            min_obs: 12,
            r_max: 1.0e6,
            eps: 1.0e-13,
            fit_drift: true,
        }
    }
}

/// 一次圆拟合的结果。
#[derive(Clone, Copy, Debug)]
pub struct Fit {
    /// 辨识出的 `R = Rs/L`（单位 s^-1）。
    pub r: S,
    /// 圆心，取值时刻为窗口时间加权均值 `t0`。
    pub center: V2,
    /// 圆心漂移速度 `du/dt`。
    pub drift: V2,
    /// 圆半径 `rho = ψf/L`（单位 A）。
    pub radius: S,
    /// 径向残差的鲁棒尺度。
    pub sigma: S,
    /// `r` 的边际方差估计；不可辨识时为 `+inf`。
    pub var_r: S,
    /// 消去 4 个冗余参数后留给 `r` 的 Schur 补信息量。
    pub info_r: S,
    /// 可观测性指数 `γ = I_R / Σ w ||B - Bbar||²`，无量纲，`[0,1]`。
    ///
    /// 恒定工况下 `γ -> 0`，表示当前窗口不能定 `R`；只有工况变化时才更新 `R`。
    pub gamma: S,
    /// 窗口内轨迹覆盖的累计弧长（弧度）。
    pub arc: S,
    /// 圆心参考时刻（窗口时间加权均值）。
    pub t0: S,
    /// 被 Huber 接受（未降权）的权重占比。
    pub inlier_frac: S,
    /// 参与拟合的观测数。
    pub n_obs: usize,
}

/// 带部分主元的高斯消元，求解 `A x = b`。
fn solve<const N: usize>(mut a: [[S; N]; N], mut b: [S; N]) -> Option<[S; N]> {
    for c in 0..N {
        let mut p = c;
        for r in (c + 1)..N {
            if a[r][c].abs() > a[p][c].abs() {
                p = r;
            }
        }
        if a[p][c].abs() < 1.0e-14 {
            return None;
        }
        if p != c {
            a.swap(p, c);
            b.swap(p, c);
        }
        let d = a[c][c];
        for r in (c + 1)..N {
            let f = a[r][c] / d;
            for k in c..N {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0.0; N];
    for i in (0..N).rev() {
        let mut s = b[i];
        for j in (i + 1)..N {
            s -= a[i][j] * x[j];
        }
        x[i] = s / a[i][i];
    }
    Some(x)
}

/// 消去 4 个冗余参数后留给 `R` 的信息，即 Schur 补。
fn schur_r(h: &[[S; 5]; 5]) -> S {
    let mut a = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            a[i][j] = h[i + 1][j + 1];
        }
    }
    let tr = (a[0][0] + a[1][1] + a[2][2] + a[3][3]).abs();
    for i in 0..4 {
        a[i][i] += 1.0e-13 * (tr + 1.0);
    }
    let b = [h[1][0], h[2][0], h[3][0], h[4][0]];
    let x = match solve::<4>(a, b) {
        Some(v) => v,
        None => return 0.0,
    };
    let mut s = h[0][0];
    for i in 0..4 {
        s -= h[0][i + 1] * x[i];
    }
    s.max(0.0)
}

/// 固定 `R` 时的代数（Kåsa）拟合，返回 `(代价, 圆心)`。
///
/// 代数代价在真 `R` 处恒等于零，因此只把它当作“盆地探测器”：
/// 在 `R` 的对数网格上取最小者作为几何拟合的初值，避开半径塌缩的伪圆。
fn kasa(obs: &[Obs], r: S) -> Option<(S, V2)> {
    let mut m = [[0.0; 3]; 3];
    let mut v = [0.0; 3];
    let mut syy = 0.0;
    for o in obs {
        let u = o.a + o.b.scaled(r);
        let q = u.dot(u);
        let phi = [u.x, u.y, 1.0];
        let y = -q;
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += o.w * phi[i] * phi[j];
            }
            v[i] += o.w * phi[i] * y;
        }
        syy += o.w * y * y;
    }
    let t = solve::<3>(m, v)?;
    let mut c = syy;
    for i in 0..3 {
        c -= 2.0 * t[i] * v[i];
        for j in 0..3 {
            c += t[i] * m[i][j] * t[j];
        }
    }
    let center = V2::new(-0.5 * t[0], -0.5 * t[1]);
    Some((c.max(0.0), center))
}

/// 给定参数时剖面化半径后的径向代价，返回 `(残差平方和, rho, sigma)`。
fn cost_at(obs: &[Obs], r: S, c: V2, v: V2, t0: S) -> (S, S, S) {
    let mut sw = 0.0;
    let mut sd = 0.0;
    for o in obs {
        let u = o.a + o.b.scaled(r) - c - v.scaled(o.t - t0);
        sd += o.w * u.norm();
        sw += o.w;
    }
    let rho = sd / sw;
    let mut ss = 0.0;
    for o in obs {
        let u = o.a + o.b.scaled(r) - c - v.scaled(o.t - t0);
        let e = u.norm() - rho;
        ss += o.w * e * e;
    }
    (ss, rho, (ss / sw).sqrt())
}

/// 轨迹绕拟合中心覆盖的累计弧长（对相邻点相位差做 wrap 后累加的范围）。
fn arc_at(obs: &[Obs], r: S, c: V2, v: V2, t0: S) -> S {
    let mut prev: Option<S> = None;
    let mut cum = 0.0;
    let mut lo: S = 0.0;
    let mut hi: S = 0.0;
    for o in obs {
        let u = o.a + o.b.scaled(r) - c - v.scaled(o.t - t0);
        let th = (-u.y).atan2(-u.x);
        if let Some(p) = prev {
            cum += wrap_pi(th - p);
            lo = lo.min(cum);
            hi = hi.max(cum);
        }
        prev = Some(th);
    }
    hi - lo
}

/// 从初值 `r0` 出发做一次局部几何圆拟合。
///
/// `freeze_r = true` 时把 `R` 固定在 `r0`，只估计 `(c, v, rho)`。
/// 这用于“当前窗口 `R` 不可辨识”的场合：此时自由拟合给出的 `R` 是任意的，
/// 它的圆心/半径对应的是那个任意 `R`；若只冻结 `R` 却沿用自由拟合的几何，
/// 两者会在 `u = A + R*B` 里差出 `(R_free - R)*B`，把角度带偏。
fn fit_local(
    obs: &[Obs],
    r0: S,
    c_init: Option<V2>,
    cfg: &FitCfg,
    t0: S,
    freeze_r: bool,
) -> Option<(Fit, S)> {
    if obs.len() < cfg.min_obs {
        return None;
    }
    let mut wt = 0.0;
    for o in obs {
        wt += o.w;
    }
    if !(wt > 0.0) || !r0.is_finite() {
        return None;
    }

    // B 的加权均值与总方差：γ 的分母。
    let mut bbar = V2::ZERO;
    for o in obs {
        bbar = bbar + o.b * o.w;
    }
    bbar = bbar.scaled(1.0 / wt);
    let mut b_var = 0.0;
    for o in obs {
        b_var += o.w * (o.b - bbar).dot(o.b - bbar);
    }

    let mut r = r0.clamp(-cfg.r_max, cfg.r_max);
    let mut c = match c_init {
        Some(c0) if c0.finite() => c0,
        _ => {
            let mut s = V2::ZERO;
            for o in obs {
                s = s + (o.a + o.b.scaled(r)) * o.w;
            }
            s.scaled(1.0 / wt)
        }
    };
    let mut v = V2::ZERO;

    let mut h = [[0.0; 5]; 5];
    let mut sigma = 1.0;
    let mut inlier = 0.0;
    let mut ok = false;

    for it in 0..cfg.iters {
        let (_, rho, sg) = cost_at(obs, r, c, v, t0);
        sigma = sg.max(1.0e-12);

        let mut ns: Vec<V2> = Vec::with_capacity(obs.len());
        for o in obs {
            let u = o.a + o.b.scaled(r) - c - v.scaled(o.t - t0);
            let d = u.norm().max(1.0e-12);
            ns.push(u.scaled(1.0 / d));
        }

        let mut jr: Vec<[S; 5]> = Vec::with_capacity(obs.len());
        let mut mean = [0.0; 5];
        for (k, o) in obs.iter().enumerate() {
            let tau = o.t - t0;
            let n = ns[k];
            let j = [
                if freeze_r { 0.0 } else { n.dot(o.b) },
                -n.x,
                -n.y,
                -tau * n.x,
                -tau * n.y,
            ];
            for i in 0..5 {
                mean[i] += o.w * j[i];
            }
            jr.push(j);
        }
        for i in 0..5 {
            mean[i] /= wt;
        }

        for row in h.iter_mut() {
            for e in row.iter_mut() {
                *e = 0.0;
            }
        }
        let mut g = [0.0; 5];
        inlier = 0.0;
        let robust = it >= cfg.warmup;
        let hk = cfg.huber * sigma;
        for (k, o) in obs.iter().enumerate() {
            let tau = o.t - t0;
            let u = o.a + o.b.scaled(r) - c - v.scaled(tau);
            let d = u.norm().max(1.0e-12);
            let e = d - rho;
            let rw = if !robust || e.abs() <= hk {
                1.0
            } else {
                hk / e.abs()
            };
            let wk = o.w * rw;
            inlier += wk;
            let j = [
                jr[k][0] - mean[0],
                jr[k][1] - mean[1],
                jr[k][2] - mean[2],
                jr[k][3] - mean[3],
                jr[k][4] - mean[4],
            ];
            for i in 0..5 {
                g[i] += wk * j[i] * e;
                for jj in 0..5 {
                    h[i][jj] += wk * j[i] * j[jj];
                }
            }
        }

        if !cfg.fit_drift {
            let pin = 1.0e9 * (h[0][0] + 1.0);
            h[3][3] += pin;
            h[4][4] += pin;
        }
        for i in 0..5 {
            h[i][i] += 1.0e-9 * (h[i][i].abs() + 1.0);
        }

        let rhs = [-g[0], -g[1], -g[2], -g[3], -g[4]];
        let dx = match solve::<5>(h, rhs) {
            Some(x) => x,
            None => break,
        };

        let scale_r = 0.5 * r.abs() + 1.0;
        let scale_c = 0.5 * sigma.max(1.0e-9) + 1.0e-9;
        let mut s0: S = 1.0;
        if dx[0].abs() > scale_r {
            s0 = s0.min(scale_r / dx[0].abs());
        }
        let dn = (dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
        if dn > scale_c {
            s0 = s0.min(scale_c / dn);
        }

        let before = cost_at(obs, r, c, v, t0).0;
        let r_new = (r + s0 * dx[0]).clamp(-cfg.r_max, cfg.r_max);
        let c_new = c + V2::new(s0 * dx[1], s0 * dx[2]);
        let v_new = v + V2::new(s0 * dx[3], s0 * dx[4]);
        let after = cost_at(obs, r_new, c_new, v_new, t0).0;
        if after <= before || !robust {
            r = r_new;
            c = c_new;
            v = v_new;
            ok = true;
        } else {
            let s1 = 0.5 * s0;
            let r2 = (r + s1 * dx[0]).clamp(-cfg.r_max, cfg.r_max);
            let c2 = c + V2::new(s1 * dx[1], s1 * dx[2]);
            let v2 = v + V2::new(s1 * dx[3], s1 * dx[4]);
            if cost_at(obs, r2, c2, v2, t0).0 <= before {
                r = r2;
                c = c2;
                v = v2;
                ok = true;
            } else {
                break;
            }
        }

        let step = (s0 * dx[0]).abs()
            + (s0 * dx[1]).abs()
            + (s0 * dx[2]).abs()
            + (s0 * dx[3]).abs()
            + (s0 * dx[4]).abs();
        if step < cfg.eps {
            break;
        }
    }
    if !ok {
        return None;
    }

    let info_r = schur_r(&h);
    let (cost_f, rho_f, sigma_f) = cost_at(obs, r, c, v, t0);
    let var_r = if info_r > 0.0 {
        sigma_f * sigma_f / info_r
    } else {
        S::INFINITY
    };
    let gamma = if info_r > 0.0 && b_var > 0.0 {
        (info_r / b_var).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let arc = arc_at(obs, r, c, v, t0);
    Some((
        Fit {
            r,
            center: c,
            drift: v,
            radius: rho_f,
            sigma: sigma_f,
            var_r,
            info_r,
            gamma,
            arc,
            t0,
            inlier_frac: inlier / wt,
            n_obs: obs.len(),
        },
        cost_f,
    ))
}

/// 全局拟合：先代数扫描锁定盆地，再局部几何拟合，并与初值局部解比较取优。
pub fn fit(obs: &[Obs], r0: S, cfg: &FitCfg) -> Option<Fit> {
    if obs.len() < cfg.min_obs {
        return None;
    }
    let mut wt = 0.0;
    let mut tw = 0.0;
    for o in obs {
        wt += o.w;
        tw += o.w * o.t;
    }
    if !(wt > 0.0) || !r0.is_finite() {
        return None;
    }
    let t0 = tw / wt;

    let mut best_r = r0;
    let mut best_cost = S::INFINITY;
    let mut best_center = None;
    let lo = (0.3 * r0.abs()).max(1.0e-6);
    let hi = (3.0 * r0.abs()).max(3.0e-6).min(cfg.r_max);
    let n = 60;
    for i in 0..=n {
        let f = i as S / n as S;
        let rr = lo * (hi / lo).powf(f);
        if let Some((cst, ctr)) = kasa(obs, rr) {
            if cst < best_cost {
                best_cost = cst;
                best_r = rr;
                best_center = Some(ctr);
            }
        }
    }

    let (a, ca) = fit_local(obs, best_r, best_center, cfg, t0, false)?;
    if let Some((b, cb)) = fit_local(obs, r0, None, cfg, t0, false) {
        if cb < ca {
            return Some(b);
        }
    }
    Some(a)
}

/// 固定 `R` 的几何拟合：只更新圆心、漂移与半径。
///
/// 当自由拟合的可观测性指数 `γ` 低于门限时，`R` 不应更新；此时必须用这个
/// 函数重新求与当前 `R` 一致的几何，不能沿用自由拟合的 `center/radius`。
pub fn fit_fixed_r(obs: &[Obs], r: S, cfg: &FitCfg) -> Option<Fit> {
    if obs.len() < cfg.min_obs || !r.is_finite() {
        return None;
    }
    let mut wt = 0.0;
    let mut tw = 0.0;
    for o in obs {
        wt += o.w;
        tw += o.w * o.t;
    }
    if !(wt > 0.0) {
        return None;
    }
    let t0 = tw / wt;
    let c_init = kasa(obs, r).map(|(_, c)| c);
    fit_local(obs, r, c_init, cfg, t0, true).map(|(f, _)| f)
}

// ===========================================================================
// 磁链观测器（角度环）
// ===========================================================================

/// 磁链 / 电阻观测器。
#[derive(Clone, Debug)]
pub struct FluxCircleObserver {
    // ---- 拟合配置 ----
    pub fit_cfg: FitCfg,
    /// 是否在线跑圆拟合。关闭后只用 `set_initial` 给的参数，便于对照。
    pub fit_enabled: bool,
    /// PLL 自然频率相对 `|omega|` 的比例。
    pub pll_wn_rel: S,
    /// PLL 自然频率下限（rad/s）。启动捕获阶段需要足够大。
    pub pll_wn_min: S,
    /// 第一次成功拟合之后切换到的 PLL 频率下限。
    ///
    /// 启动阶段用宽环路跟住加速；圆心/半径建立后收窄，否则会把斜率噪声
    /// 直接搬进角度（PAPER.md §9.1）。
    pub pll_wn_min_run: S,
    /// 不依赖拟合成功、按绝对时间收窄 PLL 的时刻（秒）。
    ///
    /// 高噪声下拟合可能一直不通过 `sigma` 门限；此时仍应在启动完成后收窄，
    /// 否则宽环路会把噪声全部搬进角度。
    pub pll_narrow_time: S,
    pub pll_zeta: S,
    /// `|u-c|/radius` 的可信区间。
    pub mag_lo: S,
    pub mag_hi: S,
    /// 触发一次拟合所需的最小累计弧长（rad）。
    pub min_arc: S,
    /// 最短拟合窗口（秒）。
    pub min_window: S,
    /// 最长窗口（秒）；超过且弧长仍不足则只重锚点、不拟合。
    pub max_window: S,
    /// 拟合窗口的降采样倍数（控制周期 → 拟合样本）。
    pub decimation: usize,
    /// 自 `set_initial` 起的绝对时间（秒）。
    total_time: S,
    narrowed: bool,
    /// 只有 `γ >= gamma_min` 才接受新的 `R`。
    pub gamma_min: S,
    /// 拟合可接受的最大相对径向残差 `sigma / radius`。
    ///
    /// 这是“圆模型是否成立”的硬门限：斜率噪声大、器件压降谐波、凸极都会
    /// 抬高 `sigma`。残差过大时宁可保留旧圆心/半径，也不要把拟合结果灌进 PLL。
    pub max_sigma_rel: S,

    // ---- 当前参数 ----
    /// `R = Rs/L`（s^-1）。
    pub r: S,
    /// `rho = ψf/L`（A）。
    pub radius: S,
    /// 电角度估计。
    pub theta: S,
    /// 电角速度估计。
    pub omega: S,

    // ---- 积分 / 拟合状态 ----
    /// 锚点时刻的圆心（此后按 `drift` 线性漂移）。
    center: V2,
    /// 圆心漂移速度 `du/dt`。
    drift: V2,
    /// 自锚点起的 `∫ s0 dt`。
    a: V2,
    /// 自锚点起的 `∫ i dt`。
    b: V2,
    /// 自锚点起的时间（秒）。
    t: S,
    /// `∫ |omega| dt`，用作弧覆盖的近似。
    arc: S,
    window: Vec<Obs>,
    sample_index: usize,
    last: ObserverOutput,

    // ---- 诊断 ----
    /// 最近一次输出的模长 / 角度是否可信。
    pub valid: bool,
    /// 最近一次由 `atan2(-(u-c))` 得到的测量角（未过 PLL）。
    pub theta_meas: S,
    /// 最近一次 `|u-c|`。
    pub flux_mag: S,
    /// 最近一次拟合的可观测性指数。
    pub gamma: S,
    /// 最近一次成功拟合。
    pub fit: Option<Fit>,
    pub fit_count: usize,
    pub fit_fail: usize,
    /// 初始 `R` 与半径，用作拟合结果的有效性闸门。
    pub r_nominal: S,
    pub radius_nominal: S,
}

impl Default for FluxCircleObserver {
    fn default() -> Self {
        Self {
            fit_cfg: FitCfg::default(),
            fit_enabled: true,
            pll_wn_rel: 0.2,
            pll_wn_min: 50.0,
            pll_wn_min_run: 1.0,
            pll_narrow_time: 2.0,
            pll_zeta: 0.9,
            mag_lo: 0.3,
            mag_hi: 3.0,
            // 拟合半径 + 圆心 + 漂移共 5 个参数，半圈数据会让“巨大半径”也能
            // 以极小的几何残差通过（小弧病态）。至少覆盖一整圈再拟合。
            min_arc: std::f64::consts::TAU,
            min_window: 1.0,
            max_window: 5.0,
            decimation: 4,
            total_time: 0.0,
            narrowed: false,
            gamma_min: 0.05,
            max_sigma_rel: 0.02,
            r: 0.0,
            radius: 0.0,
            theta: 0.0,
            omega: 0.0,
            center: V2::ZERO,
            drift: V2::ZERO,
            a: V2::ZERO,
            b: V2::ZERO,
            t: 0.0,
            arc: 0.0,
            window: Vec::new(),
            sample_index: 0,
            last: ObserverOutput::default(),
            valid: false,
            theta_meas: 0.0,
            flux_mag: 0.0,
            gamma: 0.0,
            fit: None,
            fit_count: 0,
            fit_fail: 0,
            r_nominal: 0.0,
            radius_nominal: 0.0,
        }
    }
}

impl FluxCircleObserver {
    pub fn new() -> Self {
        Self::default()
    }

    /// 写入启动（ALIGN / 交班）时刻的状态。
    ///
    /// `theta` 是已知的初始电角度，`r`/`radius` 是初始参数（允许偏离真值）。
    /// 圆心按 `u = 0 => c = radius * n(theta)` 初始化。
    pub fn set_initial(&mut self, theta: S, omega: S, r: S, radius: S) {
        self.theta = wrap_pi(theta);
        self.omega = omega;
        self.r = r;
        self.radius = radius;
        self.r_nominal = r;
        self.radius_nominal = radius;
        self.center = V2::new(self.theta.cos(), self.theta.sin()).scaled(radius);
        self.drift = V2::ZERO;
        self.a = V2::ZERO;
        self.b = V2::ZERO;
        self.t = 0.0;
        self.arc = 0.0;
        self.window.clear();
        self.sample_index = 0;
        self.total_time = 0.0;
        self.narrowed = false;
        self.gamma = 0.0;
        self.fit = None;
        self.fit_count = 0;
        self.fit_fail = 0;
        self.valid = false;
        self.last = ObserverOutput {
            electrical_angle: self.theta,
            electrical_speed: self.omega,
            continuous_speed: self.omega,
        };
    }

    /// 诊断：返回 `(A, B, u, 当前圆心)`。
    pub fn debug_state(&self) -> (V2, V2, V2, V2) {
        let c = self.center + self.drift.scaled(self.t);
        let u = self.a + self.b.scaled(self.r);
        (self.a, self.b, u, c)
    }

    /// 重锚点但保持当前 `f = u - c` 不变，只把积分器清零。
    fn reanchor_keep(&mut self) {
        let c_now = self.center + self.drift.scaled(self.t);
        let u_now = self.a + self.b.scaled(self.r);
        self.center = c_now - u_now;
        self.reset_window();
    }

    fn reset_window(&mut self) {
        self.a = V2::ZERO;
        self.b = V2::ZERO;
        self.t = 0.0;
        self.arc = 0.0;
        self.window.clear();
        self.sample_index = 0;
    }

    fn accept_fit(&self, f: &Fit) -> bool {
        f.radius.is_finite()
            && f.radius > 0.0
            && f.radius > 0.3 * self.radius_nominal
            && f.radius < 3.0 * self.radius_nominal
            && f.sigma.is_finite()
            && f.sigma <= self.max_sigma_rel * f.radius
            && f.center.finite()
            && f.drift.finite()
            && f.info_r.is_finite()
    }

    fn apply_fit(&mut self, f: Fit) {
        // 拟合坐标与观测器坐标相同（窗口从锚点开始，a=b=0），因此可以直接
        // 把圆心从 t0 外推到当前时刻。
        let c_now = f.center + f.drift.scaled(self.t - f.t0);
        let u_fit = self.a + self.b.scaled(f.r);
        let use_r = f.gamma >= self.gamma_min
            && f.r.is_finite()
            && f.r > 0.1 * self.r_nominal
            && f.r < 10.0 * self.r_nominal;

        // 重锚点后 u'=0，要保持 f 不变就必须把圆心同步平移。
        self.center = c_now - u_fit;
        self.drift = f.drift;
        self.radius = f.radius;
        if use_r {
            self.r = f.r;
        }
        self.gamma = f.gamma;
        self.fit = Some(f);
        self.reset_window();
    }

    fn maybe_fit(&mut self) {
        if !self.fit_enabled {
            return;
        }
        if self.arc < self.min_arc {
            if self.t >= self.max_window {
                self.reanchor_keep();
            }
            return;
        }
        if self.t < self.min_window {
            return;
        }
        if self.window.len() < self.fit_cfg.min_obs {
            self.reanchor_keep();
            return;
        }
        // 先自由拟合：拿到 γ，判断当前窗口能否定 R。
        let free = fit(&self.window, self.r, &self.fit_cfg);
        let free_gamma = free.as_ref().map(|f| f.gamma).unwrap_or(0.0);
        let accepted = match free {
            Some(f) if self.accept_fit(&f) && f.gamma >= self.gamma_min => {
                self.apply_fit(f);
                true
            }
            // R 不可辨识，或自由拟合的几何不可接受：固定当前 R 重新求几何。
            // 绝不能沿用自由拟合的 center/radius，否则 (R_free-R)*B 会进角度。
            _ => match fit_fixed_r(&self.window, self.r, &self.fit_cfg) {
                Some(g) if self.accept_fit(&g) => {
                    self.apply_fit(g);
                    self.gamma = free_gamma;
                    true
                }
                _ => false,
            },
        };
        if accepted {
            self.fit_count += 1;
            if self.fit_count == 2 {
                // 前两次拟合让圆心/半径/漂移建立起来，再收窄环路抑制斜率噪声。
                // 太早收窄会在速度仍在爬升时留下大相位滞后。
                self.pll_wn_min = self.pll_wn_min_run;
                self.narrowed = true;
            }
        } else {
            self.fit_fail += 1;
            self.reanchor_keep();
        }
    }
}

impl Observer<3> for FluxCircleObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let slope_dt = input.current_slope_dt;
        if !(slope_dt > 0.0) || !(delta_time > 0.0) || !delta_time.is_finite() {
            // 本拍没有可用的零矢量窗口（高调制比 / 采样非法）。
            self.valid = false;
            return self.last.clone();
        }

        let i = clarke(input.current);
        let s = clarke(input.current_slope);
        let i = V2::new(i[0], i[1]);
        let s = V2::new(s[0], s[1]);
        if !i.finite() || !s.finite() {
            self.valid = false;
            return self.last.clone();
        }

        self.t += delta_time;
        self.total_time += delta_time;
        self.a = self.a + s.scaled(delta_time);
        self.b = self.b + i.scaled(delta_time);

        let c = self.center + self.drift.scaled(self.t);
        let u = self.a + self.b.scaled(self.r);
        let f = u - c;
        let mag = f.norm();
        let theta_meas = (-f.y).atan2(-f.x);
        self.theta_meas = theta_meas;
        self.flux_mag = mag;

        // 二阶 PLL：ω̂ += ki*e*dt, θ̂ += (ω̂ + kp*e)*dt
        if !self.narrowed && self.total_time >= self.pll_narrow_time {
            // 启动完成：即使拟合因噪声不通过，也应收窄环路。
            self.pll_wn_min = self.pll_wn_min_run;
            self.narrowed = true;
        }
        let wn = (self.pll_wn_rel * self.omega.abs()).max(self.pll_wn_min);
        let kp = 2.0 * self.pll_zeta * wn;
        let ki = wn * wn;
        let e = wrap_pi(theta_meas - self.theta);
        self.omega += ki * e * delta_time;
        self.theta = wrap_pi(self.theta + (self.omega + kp * e) * delta_time);
        self.arc += self.omega.abs() * delta_time;

        let valid = self.radius > 0.0
            && mag > self.mag_lo * self.radius
            && mag < self.mag_hi * self.radius
            && theta_meas.is_finite();
        self.valid = valid;

        let out = ObserverOutput {
            electrical_angle: self.theta,
            electrical_speed: self.omega,
            continuous_speed: self.omega,
        };
        self.last = out.clone();

        self.sample_index += 1;
        if self.fit_enabled && self.sample_index % self.decimation.max(1) == 0 {
            let w = (slope_dt / delta_time).powi(2).clamp(1.0e-6, 1.0e3);
            self.window.push(Obs {
                a: self.a,
                b: self.b,
                t: self.t,
                w,
            });
        }
        self.maybe_fit();
        out
    }
}

// ===========================================================================
// 合成数据验证
// ===========================================================================

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;
    use crate::simulation::{inverse_clarke, rotate};

    const RS: S = 0.5;
    const L: S = 2.0e-3;
    const PSI: S = 0.03;

    fn synth(omega: S, id: impl Fn(S) -> S, iq: impl Fn(S) -> S, dur: S, dt: S) -> Vec<Obs> {
        let n = (dur / dt) as usize;
        let mut a = V2::ZERO;
        let mut b = V2::ZERO;
        let mut a0 = V2::ZERO;
        let mut b0 = V2::ZERO;
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as S * dt;
            let th = omega * t;
            let nn = V2::new(th.cos(), th.sin());
            let jn = V2::new(-th.sin(), th.cos());
            let i = nn.scaled(id(t)) + jn.scaled(iq(t));
            let e = jn.scaled(omega * PSI);
            let s0 = -(i.scaled(RS) + e).scaled(1.0 / L);
            a = a + s0.scaled(dt);
            b = b + i.scaled(dt);
            if k == 0 {
                a0 = a;
                b0 = b;
            }
            out.push(Obs {
                a: a - a0,
                b: b - b0,
                t,
                w: 1.0,
            });
        }
        out
    }

    #[test]
    fn fit_recovers_r_with_excitation() {
        let obs = synth(
            TAU,
            |_| 1.0,
            |t| 3.0 + 1.2 * (TAU * 0.7 * t).cos(),
            3.0,
            1.0e-4,
        );
        let r_true = RS / L;
        let f = fit(&obs, 1.25 * r_true, &FitCfg::default()).expect("fit");
        let rel = (f.r - r_true).abs() / r_true;
        assert!(
            rel < 0.02,
            "R rel err = {rel}, r = {}, rho = {}, gamma = {}",
            f.r,
            f.radius,
            f.gamma
        );
        assert!((f.radius - PSI / L).abs() / (PSI / L) < 0.05);
        assert!(f.gamma > 0.05, "gamma = {}", f.gamma);
    }

    #[test]
    fn r_is_unidentifiable_without_excitation() {
        let flat = synth(TAU, |_| 1.0, |_| 3.0, 3.0, 1.0e-4);
        let excit = synth(
            TAU,
            |_| 1.0,
            |t| 3.0 + 1.2 * (TAU * 0.7 * t).cos(),
            3.0,
            1.0e-4,
        );
        let r_true = RS / L;
        let f_flat = fit(&flat, r_true, &FitCfg::default()).expect("flat fit");
        let f_exc = fit(&excit, r_true, &FitCfg::default()).expect("excit fit");
        assert!(
            f_flat.info_r / f_exc.info_r < 0.05,
            "flat/excited info ratio = {}",
            f_flat.info_r / f_exc.info_r
        );
        assert!(
            f_flat.gamma < f_exc.gamma,
            "flat gamma {} should be < excited gamma {}",
            f_flat.gamma,
            f_exc.gamma
        );
    }

    #[test]
    fn observer_tracks_angle_with_ideal_zero_vector_slope() {
        let omega = TAU * 1.0;
        let r_true = RS / L;
        let radius = PSI / L;
        let dt = 50.0e-6;
        let mut ob = FluxCircleObserver::new();
        ob.set_initial(0.0, omega, r_true, radius);
        ob.pll_wn_min = 50.0;
        ob.min_window = 1.0;
        ob.min_arc = TAU;

        let n = (3.0 / dt) as usize;
        let mut worst: S = 0.0;
        for k in 0..n {
            let t = k as S * dt;
            let th = omega * t;
            let nn = V2::new(th.cos(), th.sin());
            let jn = V2::new(-th.sin(), th.cos());
            let i = nn.scaled(1.0) + jn.scaled(3.0);
            let e = jn.scaled(omega * PSI);
            let s = -(i.scaled(RS) + e).scaled(1.0 / L);
            let i3 = inverse_clarke([i.x, i.y]);
            let s3 = inverse_clarke([s.x, s.y]);
            let input = ObserverInput {
                voltage: [0.0; 3],
                current: i3,
                current_slope: s3,
                current_slope_dt: 20.0e-6,
                target_speed: 0.0,
            };
            let out = ob.update(dt, &input);
            if k > (1.0 / dt) as usize {
                worst = worst.max(wrap_pi(out.electrical_angle - th).abs());
            }
        }
        assert!(
            worst < 0.05,
            "worst angle error = {worst} rad, r = {}, radius = {}, gamma = {}, fits = {}",
            ob.r,
            ob.radius,
            ob.gamma,
            ob.fit_count
        );
    }

    #[test]
    fn observer_reads_three_phase_input() {
        // 说明：ObserverInput 的斜率是三相的，这里确认 clarke/inverse_clarke 往返
        // 没有把零序分量带进观测器。
        let v = V2::new(1.0, -2.0);
        let i3 = inverse_clarke([v.x, v.y]);
        let back = clarke(i3);
        assert!((back[0] - v.x).abs() < 1.0e-12);
        assert!((back[1] - v.y).abs() < 1.0e-12);
        let _ = rotate([1.0, 0.0], 0.0);
    }
}
