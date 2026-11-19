//! 直接磁链积分观测器 + 流式 Kåsa 拟合核心。
//!
//! 本模块只服务于 `DirectFluxObserver`，包含两部分：
//!
//! 1. **拟合核心**（`S`、`V2`、`Moments`、`Coef`、`FitConfig`、`Tracker`、
//!    `Report` 及全部辅助函数）。它是从旧的 `kasa_flux_observer.rs` 搬过来的
//!    36 矩 / 变量投影实现；旧 `KasaFluxObserver`（带 PLL）已删除。
//! 2. **直接观测器**（`FastFlux` / `DirectFluxObserver`）。
//!
//! # 1. 直接观测器
//!
//! * **没有 PLL**。角度就是快速环的 `rel = c + ∫s0 + R·∫i`、
//!   `θ = atan2(-rel.y, -rel.x)`；转速只对 `Δθ/Δt` 做一阶低通。上一版把拟合
//!   相位送进二阶 PLL，加速/减速后 `ω̇/wn²` 的稳态滞后会把角度拖出几 rad，
//!   减速到停机甚至丢锁；直接反正切没有这个滞后。
//! * **双速率**。`∫s0`、`∫i` 与测角在每次 `update`（电流环频率，20 kHz）；
//!   圆拟合每 `fit_period`（默认 10 ms）才取一次快速积分器的增量。
//! * **每次拟合恰好一次 Newton**。门限都做成权重，不硬冻结：
//!   可观性 `w_obs = clamp(|obs|/obs_min,0,1)²`、噪声可信度
//!   `w_sigma = clamp((σ_max/σ_rel)²,0,1)`、统计显著性
//!   `w_sig = δ²/(δ²+σ_k²)`，其中 `σ_k = sqrt(2P/((N_eff-3)P''))`、
//!   `N_eff = (Σw)²/Σw²`。`k_sigma` / `gate_sigma_rel` 是可选的硬门限，
//!   默认关。
//! * **每个拟合样本都把最新点搬到原点**（`U`、`V` 各一次精确平移），
//!   拟合圆心因此总是相对“最新点”。
//! * **加锁交班**：`Arc<Mutex<FastFlux>>` 内读并清零 `a`、`b`，同时
//!   `c += a + k_cur·b` 保持磁链连续；拟合完成后写三个目标
//!   `c_tgt = -c_local`、`k_tgt = k_fit`、`rad_tgt = radius_fit`，再让
//!   `c`、`k`、`rad` **都**按同一个增益 `sync_gain` 向各自目标滑动。
//!   这样积分器清零不会让角度跳变，三个量也不会一次写死。
//!
//! # 2. 拟合核心
//!
//! 零矢量期间 `s0 = di/dt|v=0`，令 `A = ∫s0 dt`、`B = ∫i dt`、`k = R = Rs/L`，
//! 则 `p(k) = A + k B = -(λ(t)-λ(0))/L` 落在圆心 `c = λ(0)/L`、半径 `ψf/L`
//! 的圆上。窗口被精确压缩成 36 个与 `k` 无关的加权矩，每拍一次变量投影
//! Newton：
//!
//! ```text
//!   J(k,θ) = θᵀ A(k) θ + 2 θᵀ b(k) + c(k)
//!   A(k) = A0 + k A1 + k² A2
//!   b(k) = b0 + k b1 + k² b2 + k³ b3
//!   c(k) = c0 + k c1 + k² c2 + k³ c3 + k⁴ c4
//! ```
//!
//! 数值措施：
//!
//! * 内部单位 + U 浮动原点 + `V` 原点管理（物理 `V = v' + v_off`）把矩挡在
//!   `|p| ~ R`；补偿求和用于从 0 开始的长积分。
//! * `FitConfig::shear` 可以把历史矩剪切到当前 `k` 的参考系（永远在 `κ = 0`
//!   求值）。直接观测器每个拟合周期都重锚，默认 `shear = false`，改为在
//!   `κ = k` 处把 `b` 展开到三次、`c` 到四次（用的仍是已有矩
//!   `g_vx/g_vy/b_g/g2`）。实测 0.5~100 rad/s、1~10 mA、拟合周期放大到
//!   0.2 s 都与剪切逐位一致，理由见 `direct_flux/REPORT.md`。
//! * 半径越界时把线性化伪观测 `μ(gᵀθ-z)²` 加进 `A0/b0/c0`，于是 `P`、`P'`、
//!   `P''` 一起变成带罚项剖面；Newton 步、线搜索、扫描与输出都用同一个目标，
//!   报告的损失仍用**无罚项**残差，罚项常数不会污染 `sigma_rel`。
//!
//! # 3. 与旧实现的关系
//!
//! `flux_circle_observer`（窗口批量几何拟合 + PLL）与 `KasaFluxObserver`
//! （流式拟合 + PLL）都已删除；本文件是唯一的磁链圆观测器实现，也是唯一
//! 使用这 36 个矩的地方。


use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke};
use std::sync::{Arc, Mutex};

/// 标量类型。
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
    pub fn norm2(self) -> S {
        self.dot(self)
    }

    #[inline]
    pub fn norm(self) -> S {
        self.norm2().sqrt()
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
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

type M3 = [[S; 3]; 3];

#[inline]
fn dot3(a: &[S; 3], b: &[S; 3]) -> S {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn mv3(m: &M3, v: &[S; 3]) -> [S; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

#[inline]
fn msc3(m: &M3, s: S) -> M3 {
    let mut o = *m;
    for r in o.iter_mut() {
        for v in r.iter_mut() {
            *v *= s;
        }
    }
    o
}

/// `a + s·b`（3×3，逐元素）。
#[inline]
fn madd3(a: &M3, b: &M3, s: S) -> M3 {
    let mut o = *a;
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] += s * b[i][j];
        }
    }
    o
}

/// `a + s·b`（三维向量，逐元素）。
#[inline]
fn vadd3(a: &[S; 3], b: &[S; 3], s: S) -> [S; 3] {
    [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]]
}

/// Cholesky 分解 `A = L Lᵀ`。非数值正定时返回 `None`。
fn chol3(a: &M3) -> Option<M3> {
    let mut l = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let mut s = a[i][j];
            for p in 0..j {
                s -= l[i][p] * l[j][p];
            }
            if i == j {
                let scale = a[i][i].abs().max(1.0);
                if !(s > 1.0e-14 * scale) {
                    return None;
                }
                l[i][i] = s.sqrt();
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    Some(l)
}

#[inline]
fn chol_solve(l: &M3, b: &[S; 3]) -> [S; 3] {
    let mut y = [0.0; 3];
    for i in 0..3 {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i][k] * y[k];
        }
        y[i] = s / l[i][i];
    }
    let mut x = [0.0; 3];
    for i in (0..3).rev() {
        let mut s = y[i];
        for k in (i + 1)..3 {
            s -= l[k][i] * x[k];
        }
        x[i] = s / l[i][i];
    }
    x
}

// ===========================================================================
// 补偿求和：输入是增量，位置是长和，不能丢小增量
// ===========================================================================

#[derive(Clone, Copy, Debug, Default)]
struct Acc {
    s: S,
    c: S,
}

impl Acc {
    #[inline]
    fn add(&mut self, x: S) {
        let t = self.s + x;
        if self.s.abs() >= x.abs() {
            self.c += (self.s - t) + x;
        } else {
            self.c += (x - t) + self.s;
        }
        self.s = t;
    }

    #[inline]
    fn val(&self) -> S {
        self.s + self.c
    }

    /// 坐标系平移：`value -= d`。
    #[inline]
    fn shift(&mut self, d: S) {
        self.s -= d;
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Vacc {
    x: Acc,
    y: Acc,
}

impl Vacc {
    #[inline]
    fn add(&mut self, v: V2) {
        self.x.add(v.x);
        self.y.add(v.y);
    }

    #[inline]
    fn val(&self) -> V2 {
        V2::new(self.x.val(), self.y.val())
    }

    #[inline]
    fn shift(&mut self, d: V2) {
        self.x.shift(d.x);
        self.y.shift(d.y);
    }
}

// ===========================================================================
// 充分统计量（36 个矩）
// ===========================================================================

/// `(U, V)` 的加权矩，所有求和都是 `Σ w`。
///
/// 记 `α = U·U`、`β = U·V`、`γ = V·V`，Kåsa 法方程为
///
/// ```text
///   J(k,θ) = θᵀ A(k) θ + 2 θᵀ b(k) + c(k)
///   A(k) = A0 + k A1 + k² A2
///   b(k) = b0 + k b1 + k² b2 + k³ b3
///   c(k) = c0 + k c1 + k² c2 + k³ c3 + k⁴ c4
/// ```
///
/// 每个系数都是下面这些矩的固定线性组合。注意 `ux_vx/ux_vy/uy_vx/uy_vy`
/// 存的是完整的非对称 `ΣwUVᵀ`：平移变换里 `(ΣUVᵀ)s` 与 `(ΣVUᵀ)s` 不同。
#[derive(Clone, Copy, Debug, Default)]
struct Moments {
    // ---- 论文 §4.5 的 36 个矩；字段名与论文表格（以及论文 §9.1 的代码）逐行一致 ----
    /// Σw
    w: S,
    /// ΣwU_x、ΣwU_y
    ux: S,
    uy: S,
    /// ΣwU_x²、ΣwU_xU_y、ΣwU_y²
    ux2: S,
    ux_uy: S,
    uy2: S,
    /// ΣwV_x、ΣwV_y
    vx: S,
    vy: S,
    /// ΣwV_x²、ΣwV_xV_y、ΣwV_y²
    vx2: S,
    vx_vy: S,
    vy2: S,
    /// ΣwUVᵀ 的四个元素（非对称：U_xV_y 与 U_yV_x 是两个不同的和）
    ux_vx: S,
    ux_vy: S,
    uy_vx: S,
    uy_vy: S,
    /// Σwα·(1, U_x, U_y, V_x, V_y)，α = |U|²
    a: S,
    a_ux: S,
    a_uy: S,
    a_vx: S,
    a_vy: S,
    /// Σwβ·(1, U_x, U_y, V_x, V_y)，β = U·V
    b: S,
    b_ux: S,
    b_uy: S,
    b_vx: S,
    b_vy: S,
    /// Σwγ·(1, U_x, U_y, V_x, V_y)，γ = |V|²
    g: S,
    g_ux: S,
    g_uy: S,
    g_vx: S,
    g_vy: S,
    /// Σw(α², αβ, β², αγ, βγ, γ²)
    a2: S,
    a_b: S,
    b2: S,
    a_g: S,
    b_g: S,
    g2: S,
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
        self.vx *= rho;
        self.vy *= rho;
        self.vx2 *= rho;
        self.vx_vy *= rho;
        self.vy2 *= rho;
        self.ux_vx *= rho;
        self.ux_vy *= rho;
        self.uy_vx *= rho;
        self.uy_vy *= rho;
        self.a *= rho;
        self.a_ux *= rho;
        self.a_uy *= rho;
        self.a_vx *= rho;
        self.a_vy *= rho;
        self.b *= rho;
        self.b_ux *= rho;
        self.b_uy *= rho;
        self.b_vx *= rho;
        self.b_vy *= rho;
        self.g *= rho;
        self.g_ux *= rho;
        self.g_uy *= rho;
        self.g_vx *= rho;
        self.g_vy *= rho;
        self.a2 *= rho;
        self.a_b *= rho;
        self.b2 *= rho;
        self.a_g *= rho;
        self.b_g *= rho;
        self.g2 *= rho;
    }

    /// 加入一个权重为 `q` 的样本；36 个字段逐行对应论文 §4.5 的表。
    #[inline]
    fn add(&mut self, u: V2, v: V2, q: S) {
        let alpha = u.x * u.x + u.y * u.y;
        let beta = u.x * v.x + u.y * v.y;
        let gamma = v.x * v.x + v.y * v.y;

        self.w += q;
        self.ux += q * u.x;
        self.uy += q * u.y;
        self.ux2 += q * (u.x * u.x);
        self.ux_uy += q * (u.x * u.y);
        self.uy2 += q * (u.y * u.y);
        self.vx += q * v.x;
        self.vy += q * v.y;
        self.vx2 += q * (v.x * v.x);
        self.vx_vy += q * (v.x * v.y);
        self.vy2 += q * (v.y * v.y);
        self.ux_vx += q * (u.x * v.x);
        self.ux_vy += q * (u.x * v.y);
        self.uy_vx += q * (u.y * v.x);
        self.uy_vy += q * (u.y * v.y);

        self.a += q * alpha;
        self.a_ux += q * (alpha * u.x);
        self.a_uy += q * (alpha * u.y);
        self.a_vx += q * (alpha * v.x);
        self.a_vy += q * (alpha * v.y);

        self.b += q * beta;
        self.b_ux += q * (beta * u.x);
        self.b_uy += q * (beta * u.y);
        self.b_vx += q * (beta * v.x);
        self.b_vy += q * (beta * v.y);

        self.g += q * gamma;
        self.g_ux += q * (gamma * u.x);
        self.g_uy += q * (gamma * u.y);
        self.g_vx += q * (gamma * v.x);
        self.g_vy += q * (gamma * v.y);

        self.a2 += q * (alpha * alpha);
        self.a_b += q * (alpha * beta);
        self.b2 += q * (beta * beta);
        self.a_g += q * (alpha * gamma);
        self.b_g += q * (beta * gamma);
        self.g2 += q * (gamma * gamma);
    }

    /// 剪切 `U -> U + lam V`：精确重投影，`p(κ)` 与剖面不变。
    ///
    /// 论文附录 A.1；`g_ux/g_uy` 必须跟着动（γ 不变但 U 变了）。
    fn shear(&mut self, lam: S) {
        if lam == 0.0 {
            return;
        }
        let m = *self;
        let l2 = lam * lam;
        let l3 = l2 * lam;
        let l4 = l2 * l2;

        // α' = α + 2λη + λ²γ, β' = β + λγ, γ' = γ
        self.ux = m.ux + lam * m.vx;
        self.uy = m.uy + lam * m.vy;
        self.ux2 = m.ux2 + 2.0 * lam * m.ux_vx + l2 * m.vx2;
        self.ux_uy = m.ux_uy + lam * (m.ux_vy + m.uy_vx) + l2 * m.vx_vy;
        self.uy2 = m.uy2 + 2.0 * lam * m.uy_vy + l2 * m.vy2;
        self.ux_vx = m.ux_vx + lam * m.vx2;
        self.ux_vy = m.ux_vy + lam * m.vx_vy;
        self.uy_vx = m.uy_vx + lam * m.vx_vy;
        self.uy_vy = m.uy_vy + lam * m.vy2;

        self.a = m.a + 2.0 * lam * m.b + l2 * m.g;
        self.a_ux = m.a_ux
            + lam * m.a_vx
            + 2.0 * lam * m.b_ux
            + 2.0 * l2 * m.b_vx
            + l2 * m.g_ux
            + l3 * m.g_vx;
        self.a_uy = m.a_uy
            + lam * m.a_vy
            + 2.0 * lam * m.b_uy
            + 2.0 * l2 * m.b_vy
            + l2 * m.g_uy
            + l3 * m.g_vy;
        self.a_vx = m.a_vx + 2.0 * lam * m.b_vx + l2 * m.g_vx;
        self.a_vy = m.a_vy + 2.0 * lam * m.b_vy + l2 * m.g_vy;

        self.b = m.b + lam * m.g;
        self.b_ux = m.b_ux + lam * m.b_vx + lam * m.g_ux + l2 * m.g_vx;
        self.b_uy = m.b_uy + lam * m.b_vy + lam * m.g_uy + l2 * m.g_vy;
        self.b_vx = m.b_vx + lam * m.g_vx;
        self.b_vy = m.b_vy + lam * m.g_vy;

        // γ 不变，但 U 变了
        self.g_ux = m.g_ux + lam * m.g_vx;
        self.g_uy = m.g_uy + lam * m.g_vy;

        self.a2 = m.a2 + 4.0 * lam * m.a_b + 2.0 * l2 * m.a_g + 4.0 * l2 * m.b2
            + 4.0 * l3 * m.b_g
            + l4 * m.g2;
        self.a_b = m.a_b + lam * m.a_g + 2.0 * lam * m.b2 + 3.0 * l2 * m.b_g + l3 * m.g2;
        self.b2 = m.b2 + 2.0 * lam * m.b_g + l2 * m.g2;
        self.a_g = m.a_g + 2.0 * lam * m.b_g + l2 * m.g2;
        self.b_g = m.b_g + lam * m.g2;
        // w、V 那一族、g、g_vx、g_vy、g2 在剪切下不变
    }

    /// 平移 `U -> U - s`（`V` 不动）：精确；Kåsa 评分不变，圆心读数加回 `s`。
    ///
    /// 论文附录 A.2。
    fn translate_u(&mut self, s: V2) {
        if s.norm2() == 0.0 {
            return;
        }
        let m = *self;
        let (sx, sy) = (s.x, s.y);
        let s2 = sx * sx + sy * sy;
        let su = sx * m.ux + sy * m.uy;
        let sv = sx * m.vx + sy * m.vy;
        let uu_s_x = m.ux2 * sx + m.ux_uy * sy;
        let uu_s_y = m.ux_uy * sx + m.uy2 * sy;
        let uv_s_x = m.ux_vx * sx + m.ux_vy * sy;
        let uv_s_y = m.uy_vx * sx + m.uy_vy * sy;
        let vu_s_x = m.ux_vx * sx + m.uy_vx * sy;
        let vu_s_y = m.ux_vy * sx + m.uy_vy * sy;
        let vv_s_x = m.vx2 * sx + m.vx_vy * sy;
        let vv_s_y = m.vx_vy * sx + m.vy2 * sy;

        self.ux = m.ux - sx * m.w;
        self.uy = m.uy - sy * m.w;
        self.ux2 = m.ux2 - 2.0 * sx * m.ux + m.w * sx * sx;
        self.ux_uy = m.ux_uy - sx * m.uy - sy * m.ux + m.w * sx * sy;
        self.uy2 = m.uy2 - 2.0 * sy * m.uy + m.w * sy * sy;
        self.ux_vx = m.ux_vx - sx * m.vx;
        self.ux_vy = m.ux_vy - sx * m.vy;
        self.uy_vx = m.uy_vx - sy * m.vx;
        self.uy_vy = m.uy_vy - sy * m.vy;

        self.a = m.a - 2.0 * su + s2 * m.w;
        self.a_ux = m.a_ux
            - sx * m.a
            - 2.0 * uu_s_x
            + m.ux * s2
            + sx * (2.0 * su)
            - sx * (s2 * m.w);
        self.a_uy = m.a_uy
            - sy * m.a
            - 2.0 * uu_s_y
            + m.uy * s2
            + sy * (2.0 * su)
            - sy * (s2 * m.w);
        self.a_vx = m.a_vx - 2.0 * vu_s_x + m.vx * s2;
        self.a_vy = m.a_vy - 2.0 * vu_s_y + m.vy * s2;

        self.b = m.b - sv;
        self.b_ux = m.b_ux - sx * m.b - uv_s_x + sx * sv;
        self.b_uy = m.b_uy - sy * m.b - uv_s_y + sy * sv;
        self.b_vx = m.b_vx - vv_s_x;
        self.b_vy = m.b_vy - vv_s_y;

        self.g_ux = m.g_ux - sx * m.g;
        self.g_uy = m.g_uy - sy * m.g;

        let s_au = sx * m.a_ux + sy * m.a_uy;
        let s_av = sx * m.a_vx + sy * m.a_vy;
        let s_bu = sx * m.b_ux + sy * m.b_uy;
        let s_bv = sx * m.b_vx + sy * m.b_vy;
        let s_gu = sx * m.g_ux + sy * m.g_uy;
        let s_gv = sx * m.g_vx + sy * m.g_vy;
        let s_uu_s = sx * uu_s_x + sy * uu_s_y;
        let s_uv_s = sx * uv_s_x + sy * uv_s_y;
        let s_vv_s = sx * vv_s_x + sy * vv_s_y;

        self.a2 = m.a2 - 4.0 * s_au + 2.0 * s2 * m.a + 4.0 * s_uu_s - 4.0 * s2 * su
            + s2 * s2 * m.w;
        self.a_b = m.a_b - s_av - 2.0 * s_bu + s2 * m.b + 2.0 * s_uv_s - s2 * sv;
        self.b2 = m.b2 - 2.0 * s_bv + s_vv_s;
        self.a_g = m.a_g - 2.0 * s_gu + s2 * m.g;
        self.b_g = m.b_g - s_gv;
        // w、V 那一族、g、g_vx、g_vy、g2 在 U 平移下不变
    }

    /// 平移 `V -> V - s`（`U` 不动）：纯规范，`P`、`P'`、`P''` 不变。
    ///
    /// 论文附录 A.3；物理 `V = V' + v_off`，剪切时要补 `U_s += dk·v_off`。
    fn translate_v(&mut self, s: V2) {
        if s.norm2() == 0.0 {
            return;
        }
        let m = *self;
        let (sx, sy) = (s.x, s.y);
        let s2 = sx * sx + sy * sy;
        let su = sx * m.ux + sy * m.uy;
        let sv = sx * m.vx + sy * m.vy;
        let uu_s_x = m.ux2 * sx + m.ux_uy * sy;
        let uu_s_y = m.ux_uy * sx + m.uy2 * sy;
        let uv_s_x = m.ux_vx * sx + m.ux_vy * sy;
        let uv_s_y = m.uy_vx * sx + m.uy_vy * sy;
        let vu_s_x = m.ux_vx * sx + m.uy_vx * sy;
        let vu_s_y = m.ux_vy * sx + m.uy_vy * sy;
        let vv_s_x = m.vx2 * sx + m.vx_vy * sy;
        let vv_s_y = m.vx_vy * sx + m.vy2 * sy;

        self.vx = m.vx - sx * m.w;
        self.vy = m.vy - sy * m.w;
        self.vx2 = m.vx2 - 2.0 * sx * m.vx + m.w * sx * sx;
        self.vx_vy = m.vx_vy - sx * m.vy - sy * m.vx + m.w * sx * sy;
        self.vy2 = m.vy2 - 2.0 * sy * m.vy + m.w * sy * sy;
        self.ux_vx = m.ux_vx - m.ux * sx;
        self.ux_vy = m.ux_vy - m.ux * sy;
        self.uy_vx = m.uy_vx - m.uy * sx;
        self.uy_vy = m.uy_vy - m.uy * sy;

        self.a_vx = m.a_vx - sx * m.a;
        self.a_vy = m.a_vy - sy * m.a;
        self.b = m.b - su;
        self.b_ux = m.b_ux - uu_s_x;
        self.b_uy = m.b_uy - uu_s_y;
        self.b_vx = m.b_vx - sx * m.b - vu_s_x + sx * su;
        self.b_vy = m.b_vy - sy * m.b - vu_s_y + sy * su;

        self.g = m.g - 2.0 * sv + s2 * m.w;
        self.g_ux = m.g_ux - 2.0 * uv_s_x + m.ux * s2;
        self.g_uy = m.g_uy - 2.0 * uv_s_y + m.uy * s2;
        self.g_vx = m.g_vx
            - sx * m.g
            - 2.0 * vv_s_x
            + m.vx * s2
            + sx * (2.0 * sv)
            - sx * (s2 * m.w);
        self.g_vy = m.g_vy
            - sy * m.g
            - 2.0 * vv_s_y
            + m.vy * s2
            + sy * (2.0 * sv)
            - sy * (s2 * m.w);

        let s_au = sx * m.a_ux + sy * m.a_uy;
        let s_av = sx * m.a_vx + sy * m.a_vy;
        let s_bu = sx * m.b_ux + sy * m.b_uy;
        let s_bv = sx * m.b_vx + sy * m.b_vy;
        let s_gu = sx * m.g_ux + sy * m.g_uy;
        let s_gv = sx * m.g_vx + sy * m.g_vy;
        let s_uu_s = sx * uu_s_x + sy * uu_s_y;
        let s_uv_s = sx * uv_s_x + sy * uv_s_y;
        let s_vv_s = sx * vv_s_x + sy * vv_s_y;

        self.a_b = m.a_b - s_au;
        self.b2 = m.b2 - 2.0 * s_bu + s_uu_s;
        self.a_g = m.a_g - 2.0 * s_av + s2 * m.a;
        self.b_g = m.b_g - 2.0 * s_bv - s_gu + 2.0 * s_uv_s + s2 * m.b - s2 * su;
        self.g2 = m.g2 - 4.0 * s_gv + 2.0 * s2 * m.g + 4.0 * s_vv_s - 4.0 * s2 * sv
            + s2 * s2 * m.w;
        // w、U 那一族、a、a_ux、a_uy、a2 在 V 平移下不变
    }

    #[inline]
    fn a_mat(&self) -> M3 {
        [
            [self.ux2, self.ux_uy, self.ux],
            [self.ux_uy, self.uy2, self.uy],
            [self.ux, self.uy, self.w],
        ]
    }

    #[inline]
    fn b_vec(&self) -> [S; 3] {
        [self.a_ux, self.a_uy, self.a]
    }

    /// 当前参考 `k`（`κ = 0`）处的剖面 Taylor 系数。
    #[inline]
    fn coef(&self) -> Coef {
        self.coef_at(0.0)
    }

    /// 在 `κ = kap` 处展开的剖面 Taylor 系数。
    ///
    /// 剪切模式下矩被保持在当前 `k` 的参考系里，所有调用都是 `kap = 0`。
    /// `shear = false` 时矩里存的是原始 `U`、`V'`（`k_ref = 0`），求值点在
    /// `κ = k`，需要把
    ///
    /// ```text
    ///   A(κ) = A0 + κA1 + κ²A2
    ///   b(κ) = b0 + κb1 + κ²b2 + κ³b3
    ///   c(κ) = c0 + κc1 + κ²c2 + κ³c3 + κ⁴c4
    /// ```
    ///
    /// 展开到 `κ`。返回的 `Coef` 仍是“以 `κ` 为原点”的 Taylor 系数
    /// （`a0 = A(κ)`、`a1 = A'(κ)`、`a2 = A''(κ)/2`，`b`、`c` 同理），
    /// 所以 `fit_from_coef` 完全不用改。
    fn coef_at(&self, kap: S) -> Coef {
        let a0 = self.a_mat();
        let a1 = [
            [
                self.ux_vx * 2.0,
                self.ux_vy + self.uy_vx,
                self.vx,
            ],
            [
                self.ux_vy + self.uy_vx,
                self.uy_vy * 2.0,
                self.vy,
            ],
            [self.vx, self.vy, 0.0],
        ];
        let a2 = [
            [self.vx2, self.vx_vy, 0.0],
            [self.vx_vy, self.vy2, 0.0],
            [0.0, 0.0, 0.0],
        ];
        let b0 = self.b_vec();
        let b1 = [
            self.a_vx + 2.0 * self.b_ux,
            self.a_vy + 2.0 * self.b_uy,
            2.0 * self.b,
        ];
        let b2 = [
            self.g_ux + 2.0 * self.b_vx,
            self.g_uy + 2.0 * self.b_vy,
            self.g,
        ];
        let c0 = self.a2;
        let c1 = 4.0 * self.a_b;
        let c2 = 4.0 * self.b2 + 2.0 * self.a_g;
        if kap == 0.0 {
            return Coef {
                a0,
                a1,
                a2,
                b0,
                b1,
                b2,
                c0,
                c1,
                c2,
            };
        }
        // 只有 `b` 的三次项和 `c` 的三、四次项需要额外的高阶矩。
        let k2 = kap * kap;
        let b3 = [self.g_vx, self.g_vy, 0.0];
        let c3 = 4.0 * self.b_g;
        let c4 = self.g2;
        Coef {
            a0: madd3(&madd3(&a0, &a1, kap), &a2, k2),
            a1: madd3(&a1, &a2, 2.0 * kap),
            a2,
            b0: vadd3(&vadd3(&vadd3(&b0, &b1, kap), &b2, k2), &b3, k2 * kap),
            b1: vadd3(&vadd3(&b1, &b2, 2.0 * kap), &b3, 3.0 * k2),
            b2: vadd3(&b2, &b3, 3.0 * kap),
            c0: c0 + kap * (c1 + kap * (c2 + kap * (c3 + kap * c4))),
            c1: c1 + kap * (2.0 * c2 + kap * (3.0 * c3 + kap * 4.0 * c4)),
            c2: c2 + kap * (3.0 * c3 + kap * 6.0 * c4),
        }
    }

    /// 在当前参考 `k`（`κ = 0`）处做变量投影（测试用）。
    #[cfg(test)]
    fn fit(&self) -> Option<Profile> {
        if !(self.w > 0.0) {
            return None;
        }
        fit_from_coef(&self.coef())
    }

    /// 给定 `θ` 在**无罚项**法方程上的残差平方和
    /// `θᵀA₀θ + 2θᵀb₀ + c₀`（测试用）。
    #[cfg(test)]
    fn loss_at(&self, theta: &[S; 3]) -> S {
        loss_from_coef(&self.coef(), theta)
    }
}

/// 给定 Taylor 系数，`θ` 在**无罚项**法方程上的残差平方和
/// `θᵀa₀θ + 2θᵀb₀ + c₀`。
fn loss_from_coef(co: &Coef, theta: &[S; 3]) -> S {
    co.c0 + 2.0 * dot3(theta, &co.b0) + dot3(theta, &mv3(&co.a0, theta))
}

#[derive(Clone, Copy)]
struct Coef {
    a0: M3,
    a1: M3,
    a2: M3,
    b0: [S; 3],
    b1: [S; 3],
    b2: [S; 3],
    c0: S,
    c1: S,
    c2: S,
}

/// 当前参考 `k` 处的一次剖面求值。
#[derive(Clone, Copy, Debug)]
struct Profile {
    theta: [S; 3],
    /// 半径（内部单位）。
    radius: S,
    p: S,
    p1: S,
    p2: S,
}

/// 半径伪观测 `μ (gᵀθ - z)²` 的线性化：在无约束解处取 `g`、`z`，在一个
/// Newton 步（含线搜索）内保持不变，下一步再用新的解重新线性化。
#[derive(Clone, Copy, Debug)]
struct RadiusPenalty {
    /// `R²(θ)` 的梯度。
    g: [S; 3],
    /// 目标约束 `gᵀθ = z`。
    z: S,
    /// 权重 `μ = r_kappa·Σw`。
    mu: S,
}

/// 由剖面 Taylor 系数解出 `θ*`、`P`、`P'`、`P''`（`κ = 0` 处）。
fn fit_from_coef(co: &Coef) -> Option<Profile> {
    let chol = chol3(&co.a0)?;
    let u = chol_solve(&chol, &co.b0); // u = A⁻¹ b, θ* = -u
    let au = mv3(&co.a1, &u); // A' u
    let s_bp = chol_solve(&chol, &co.b1); // A⁻¹ b'
    let s_au = chol_solve(&chol, &au); // A⁻¹ A' u

    let p = co.c0 - dot3(&co.b0, &u);
    let p1 = co.c1 - 2.0 * dot3(&u, &co.b1) + dot3(&u, &au);

    let bpp = [2.0 * co.b2[0], 2.0 * co.b2[1], 2.0 * co.b2[2]];
    let app_u = mv3(&msc3(&co.a2, 2.0), &u);
    let p2 = 2.0 * co.c2 - 2.0 * dot3(&co.b1, &s_bp) + 4.0 * dot3(&s_bp, &au)
        - 2.0 * dot3(&u, &bpp)
        - 2.0 * dot3(&au, &s_au)
        + dot3(&u, &app_u);

    if !p.is_finite() || !p1.is_finite() || !p2.is_finite() {
        return None;
    }
    let theta = [-u[0], -u[1], -u[2]];
    let center = V2::new(-0.5 * theta[0], -0.5 * theta[1]);
    let r2 = center.norm2() - theta[2];
    let radius = if r2 > 0.0 && r2.is_finite() {
        r2.sqrt()
    } else {
        S::NAN
    };
    Some(Profile {
        theta,
        radius,
        p,
        p1,
        p2,
    })
}

/// 在 `m` 的 `κ = 0` 处求带罚项剖面（测试用）。
#[cfg(test)]
fn fit_with_penalty(m: &Moments, pen: Option<RadiusPenalty>) -> Option<Profile> {
    fit_with_penalty_coef(&m.coef(), pen)
}

/// 在目标 `κ` 处求剖面（系数由 [`Tracker::coef_at`] 给出）；`pen` 非空时把
/// 线性化半径罚项加进法方程：`A₀ += μggᵀ`、`b₀ -= μzg`、`c₀ += μz²`。
///
/// `g`、`z` 固定，所以 `P'`、`P''` 也是同一个带罚项剖面的导数，可以直接
/// 拿去做 Newton 步和线搜索。
fn fit_with_penalty_coef(co: &Coef, pen: Option<RadiusPenalty>) -> Option<Profile> {
    let Some(p) = pen else {
        return fit_from_coef(co);
    };
    let mut co = *co;
    for i in 0..3 {
        for j in 0..3 {
            co.a0[i][j] += p.mu * p.g[i] * p.g[j];
        }
        co.b0[i] -= p.mu * p.z * p.g[i];
    }
    co.c0 += p.mu * p.z * p.z;
    fit_from_coef(&co)
}

/// 物理 `V = V' + v_off` 的加权二阶矩 `Σw|V|² = g0 + 2 v_off·ΣwV' + w|v_off|²`。
///
/// 它与 `V` 的原点规范无关（`v_off` 怎么拆分都不变），所以 `recenter_v` 不会
/// 让可观性/先验的尺度跳变。注意：这里**不用**中心化方差
/// `Σw|V - V̄|²`——那会把 `V` 的积分偏移也当成“没有信息”去掉，1 mA 稳态下
/// `R` 会被噪声推到 +17%（实测），因为它同时拿掉了先验里对漂移的阻尼。
#[inline]
fn v_second(m: &Moments, v_off: V2) -> S {
    m.g + 2.0 * (v_off.x * m.vx + v_off.y * m.vy) + m.w * v_off.norm2()
}

// ===========================================================================
// 流式拟合器
// ===========================================================================

/// 流式 Kåsa 拟合配置。
#[derive(Clone, Copy, Debug)]
pub struct FitConfig {
    /// 指数遗忘时间常数（秒）。`<= 0` 表示不遗忘。
    pub tau: S,
    /// 半径有效范围（用户单位，A）。
    pub r_min: S,
    pub r_max: S,
    /// `k = Rs/L` 的硬范围（s⁻¹）。
    pub k_min: S,
    pub k_max: S,
    /// 每样本允许的相对步长：`|δk| <= k_step_rel·(1 + |k|)`。
    pub k_step_rel: S,
    /// 随机游走先验强度，相对于“满激励曲率” `4 R² Σw|V|²`。
    ///
    /// 它给 Newton 分母加阻尼：激励越小，单步越保守。
    pub k_prior_rel: S,
    /// 可观性门限：`P'' / (4 R² Σw|V|²)` 低于它时冻结 `k`。
    pub obs_min: S,
    /// 初始化网格扫描接受候选点时要求的可观性门限。
    ///
    /// 它通常比 `obs_min` 低：扫描是一次性的全局盆地搜索（还要求损失至少
    /// 减半），允许在激励开始衰减时仍把 `k` 放到正确盆地；随后的逐拍 Newton
    /// 用更严的 `obs_min` 防止噪声随机游走。
    pub scan_obs_min: S,
    /// 每样本 Newton 步数。
    pub inner_iters: usize,
    /// 是否把历史矩剪切到当前 `k` 的参考系（论文附录 A.1）。
    ///
    /// * `true`：存 `U_s = U + k_ref·V`，求值永远在 `κ = 0`；
    /// * `false`：直接存 `U`、`V'`，每次在 `κ = k` 处把剖面展开到三/四次
    ///   （`b` 到 `κ³`、`c` 到 `κ⁴`，这些矩本来就有，不需要额外统计量）。
    ///
    /// 两者**数学上严格等价**，差别只在浮点相消：`P = σ(κ) − q(κ)ᵀM(κ)⁻¹q(κ)`
    /// 在 `κ = k` 处两项都是 `(k|V|)⁴` 量级，而 `P` 本身是 `(2R)²·σ_rel²·Σw`
    /// 量级。但这个上界只在 `|V|` 本身很大时才有意义：直接观测器每个拟合周期
    /// 都把最新样本搬到原点、`V` 的均值又搬进 `v_off`，矩里的 `|V|` 只有
    /// 一个周期的增量，实测 `κ = k` 与剪切逐位一致（见模块文档 §4）。这里默认
    /// `true` 保持通用；[`DirectFluxObserver`](super::direct_flux_observer)
    /// 显式设为 `false`。
    ///
    /// 这是 A/B 实验开关；运行时切换由 [`Tracker::set_shear`] 做一次精确重投影。
    pub shear: bool,
    /// 浮动原点阈值（内部单位）。
    pub recenter_eps: S,
    /// `V` 原点管理的触发阈值：存储 `V'` 的加权均值超过 `RMS(V')` 的这个倍数时
    /// 把它搬进 `v_off`。
    ///
    /// 论文对 U、V 共用一个长度阈值 `recenter_eps`；但 `V` 是“长度 / k”的积分，
    /// 量纲与 `U` 不同，绝对阈值会随工况放大到几乎不可达（本仿真 8 s 时
    /// `|V̄| ≈ 0.011`，而 `recenter_eps = 0.25`）。改用相对 RMS 的判据后：
    /// 纯旋转的 `V`（`|V̄|/RMS ~ 1/(ωτ)`）不触发，只有直流偏置/启动偏移把均值
    /// 推高时才触发。
    pub recenter_v_rel: S,
    /// `P''` 的绝对下限。
    pub p2_eps: S,
    /// 半径伪观测权重（相对 `Σw`）。
    ///
    /// 半径越界时它是“把 `R²` 拉回 `[r_min², r_max²]`”的罚项权重；罚项加进
    /// `M₀/b₀/c₀`，所以同时影响 `P`、`P'`、`P''`（Newton 步与线搜索）。
    pub r_kappa: S,
    /// 允许更新 `k` 所需的最小有效样本数（`Σw`）。
    pub min_weight: S,
    /// 圆模型可信度门限：径向残差 RMS / 半径 超过它时冻结 `k`。
    ///
    /// 高噪声下圆模型本身不成立，剖面曲率会被噪声抬高，`k` 的更新不可信。
    pub max_sigma_rel: S,
    /// 统计显著性**可开关硬门限**（以 `σ_k` 为单位）；`0` = 关闭，只用软权重。
    ///
    /// 开启时：`|δk| < k_sigma·σ_k` 直接跳过本拍；同时软权重
    /// `w_sig = δ²/(δ²+σ_k²)` 仍然相乘（门限与加权同时生效，不是二选一）。
    pub k_sigma: S,
    /// 是否对噪声可信度 `sigma_rel` 加硬门限（与软权重 `w_sigma` 同时生效）。
    pub gate_sigma_rel: bool,
    /// 初始化网格扫描的相对范围：在 `[k0/scan_rel, k0·scan_rel]` 上取对数网格。
    pub scan_rel: S,
    /// 扫描接受条件：候选点的剖面损失必须低于 `scan_gain · 当前损失`。
    pub scan_gain: S,
    /// 初始化网格扫描的点数。
    pub scan_points: usize,
}

impl Default for FitConfig {
    fn default() -> Self {
        Self {
            tau: 1.0,
            r_min: 1.0,
            r_max: 3.0,
            k_min: -1.0e6,
            k_max: 1.0e6,
            k_step_rel: 0.02,
            k_prior_rel: 0.02,
            obs_min: 5.0e-3,
            scan_obs_min: 1.0e-4,
            inner_iters: 2,
            shear: true,
            recenter_eps: 0.25,
            recenter_v_rel: 0.25,
            p2_eps: 1.0e-18,
            r_kappa: 50.0,
            min_weight: 8.0,
            max_sigma_rel: 0.02,
            k_sigma: 0.0,
            gate_sigma_rel: false,
            scan_rel: 2.0,
            scan_gain: 0.5,
            scan_points: 96,
        }
    }
}

/// 一次样本更新后返回的拟合报告（拟合不可用时为 `None`）。
#[derive(Clone, Copy, Debug)]
pub struct Report {
    /// `k = Rs/L`（s⁻¹）。
    pub k: S,
    /// 圆半径（用户单位，A）。
    pub radius: S,
    /// 圆心（用户单位，绝对坐标）。
    pub center: V2,
    /// 最新样本相对圆心的位置（用户单位，与坐标系无关）。
    pub p_rel: V2,
    /// `|p_rel|`。
    pub dist: S,
    /// `|p_rel| - radius`。
    pub radial_err: S,
    /// `P / Σw`：平均代数残差。
    pub loss: S,
    /// 径向残差 RMS / 半径：圆模型可信度。
    pub sigma_rel: S,
    /// `k` 的标准误 `sqrt(2P/((N_eff-3)P''))`，单位同 `k`；`P''≤0` 时为 `∞`。
    pub sigma_k: S,
    /// 有效样本数 `Σw`。
    pub weight: S,
    /// 剖面一阶导 `P'`。
    pub p1: S,
    /// 剖面二阶导 `P''`。
    pub p2: S,
    /// 无量纲可观性指数 `P'' / (4R²Σw|V|²)`。
    pub obs: S,
    /// 本拍实际走的 Newton 步数。
    pub steps: u32,
    /// 半径范围约束是否起作用。
    pub clamped: bool,
}

/// 指数遗忘的流式 Kåsa 拟合器。
///
/// 剪切开（`shear = true`）时，存储的 `(U, V)` 是“当前参考 `k`”下的量：
/// `U_s = U + k_ref V`，每拍在 `κ = 0` 处求值。剪切断（`shear = false`）时存
/// 原始 `U`、`V'`，在 `κ = k` 处展开；两种写法共用同一套矩和 `fit_from_coef`。
///
/// `V` 另外有一个原点偏移：物理 `V = v + v_off`。`v` 存的是搬掉加权均值后
/// 的 `V'`，所以 `U_s` 仍是物理量，剪切时要补 `dk·v_off`。
#[derive(Clone, Debug)]
pub struct Tracker {
    cfg: FitConfig,
    m: Moments,
    /// 内部长度单位 `(r_min + r_max)/2`。
    unit_len: S,
    rmin_i: S,
    rmax_i: S,
    /// 当前估计，也是矩的参考 `k`。
    k: S,
    /// 存储矩的 `U_s` 所处的参考 `k`：`U_s = U + k_ref·V`。
    ///
    /// 剪切模式在每次 `k` 变化后由 [`Tracker::reproject`] 把 `k_ref` 推到
    /// 新的 `k`；非剪切模式恒为 `0`，求值点在 `κ = k - k_ref`。
    k_ref: S,
    /// `set_initial` 给的初始 `k`，网格扫描围绕它展开。
    k_initial: S,
    /// 当前样本在局部参考系中的位置 `U_s`。
    u: Vacc,
    /// 当前样本的 `V'`（已搬掉原点；物理 `V = V' + v_off`）。
    v: Vacc,
    /// 局部原点在绝对（用户）坐标系中的位置。
    off: V2,
    /// `V` 的原点偏移：物理 `V = v + v_off`。
    v_off: V2,
    t_last: S,
    started: bool,
    /// 已经累积的路径长度 `Σ|Δp|`（内部单位）。它与 `k` 无关，用作“数据
    /// 实际覆盖了多长弧”的度量；拿 `|ω|dt` 会在 `k` 错时严重低估。
    path: S,
    /// `Σw²`：只用于算有效样本数 `N_eff = (Σw)²/Σw²`（Kish），
    /// 进而估计 `k` 的标准误 `σ_k = sqrt(2P/((N_eff-3)P''))`。
    w2: S,
    obs: S,
    steps: u32,
}

impl Tracker {
    pub fn new(cfg: FitConfig, k0: S) -> Self {
        let unit_len = 0.5 * (cfg.r_min + cfg.r_max);
        let unit_len = if unit_len.is_finite() && unit_len > 0.0 {
            unit_len
        } else {
            1.0
        };
        let k = if k0.is_finite() {
            k0.clamp(cfg.k_min, cfg.k_max)
        } else {
            0.0
        };
        Self {
            rmin_i: cfg.r_min / unit_len,
            rmax_i: cfg.r_max / unit_len,
            cfg,
            m: Moments::default(),
            unit_len,
            k,
            k_ref: if cfg.shear { k } else { 0.0 },
            k_initial: k,
            u: Vacc::default(),
            v: Vacc::default(),
            off: V2::ZERO,
            v_off: V2::ZERO,
            t_last: 0.0,
            started: false,
            path: 0.0,
            w2: 0.0,
            obs: 0.0,
            steps: 0,
        }
    }

    #[inline]
    pub fn k(&self) -> S {
        self.k
    }

    #[inline]
    pub fn weight(&self) -> S {
        self.m.w
    }

    #[inline]
    pub fn obs(&self) -> S {
        self.obs
    }

    #[inline]
    pub fn unit_len(&self) -> S {
        self.unit_len
    }

    #[inline]
    pub fn off(&self) -> V2 {
        self.off
    }

    /// 物理 `V` 相对存储 `V'` 的原点偏移（内部单位）：`V = V' + v_off`。
    #[inline]
    pub fn v_off(&self) -> V2 {
        self.v_off
    }

    /// 物理 `V` 的加权二阶矩 `Σw|V|²`（对 `recenter_v` 不变）。
    #[inline]
    fn v_second(&self) -> S {
        v_second(&self.m, self.v_off)
    }

    /// 当前样本在局部参考系中的位置（内部单位）。
    #[inline]
    pub fn p_local(&self) -> V2 {
        self.u.val()
    }

    /// 已累积的路径长度 `Σ|Δp|`（内部单位）。
    #[inline]
    pub fn path(&self) -> S {
        self.path
    }

    /// 喂入一对增量 `(ΔU, ΔV)`。
    ///
    /// * `t`：样本时刻（秒），只用于指数遗忘；
    /// * `weight`：该样本的拟合权重；
    /// * `newton_iters`：本拍允许的 Newton 步数（0 表示冻结 `k`）。
    pub fn update(
        &mut self,
        delta_u: V2,
        delta_v: V2,
        t: S,
        weight: S,
        newton_iters: usize,
    ) -> Option<Report> {
        // 1. 遗忘
        let rho = if self.cfg.tau > 0.0 {
            if self.started {
                (-(t - self.t_last).max(0.0) / self.cfg.tau).exp()
            } else {
                1.0
            }
        } else {
            1.0
        };
        self.t_last = t;
        self.started = true;
        self.m.decay(rho);
        self.w2 *= rho * rho;

        // 2. 积分增量（内部单位）。剪切模式存的是 U_s = U + k V，所以增量是
        //    ΔU + k ΔV；非剪切模式存原始 U，只加 ΔU。路径长度始终用物理
        //    Δp = ΔU + k ΔV，与存储方式无关。
        let inv = 1.0 / self.unit_len;
        let du = delta_u.mul(inv);
        let dv = delta_v.mul(inv);
        let du_p = du.add(dv.mul(self.k));
        let du_s = if self.cfg.shear { du_p } else { du };
        self.u.add(du_s);
        self.v.add(dv);
        self.path += du_p.norm();

        // 3. 累积矩
        let (ui, vi) = (self.u.val(), self.v.val());
        let q = if weight.is_finite() && weight > 0.0 {
            weight
        } else {
            1.0
        };
        self.m.add(ui, vi, q);
        self.w2 += q * q;

        // 4. 浮动原点：`U` 的质心归零、`V` 的加权均值归零
        self.recenter();
        self.recenter_v();

        // 5. 变量投影 Newton（由调用方用 newton_iters 控制是否允许更新 k）
        self.steps = if newton_iters > 0 {
            self.newton(newton_iters)
        } else {
            0
        };
        self.recenter();
        self.recenter_v();

        self.report()
    }

    /// 立刻重新求一次报告（不喂新样本）。
    pub fn report_now(&self) -> Option<Report> {
        self.report()
    }

    /// 只同步**可在线调整**的字段（不改 `unit_len`、半径范围、`k` 范围）。
    /// 用于运行中开关显著性/噪声硬门限、调权重尺度。
    pub fn sync_fit(&mut self, cfg: &FitConfig) {
        self.cfg.tau = cfg.tau;
        self.cfg.obs_min = cfg.obs_min;
        self.cfg.k_prior_rel = cfg.k_prior_rel;
        self.cfg.k_step_rel = cfg.k_step_rel;
        self.cfg.max_sigma_rel = cfg.max_sigma_rel;
        self.cfg.k_sigma = cfg.k_sigma;
        self.cfg.gate_sigma_rel = cfg.gate_sigma_rel;
        self.cfg.p2_eps = cfg.p2_eps;
        self.cfg.r_kappa = cfg.r_kappa;
        self.cfg.recenter_v_rel = cfg.recenter_v_rel;
        if self.cfg.shear != cfg.shear {
            self.set_shear(cfg.shear);
        }
    }

    /// 打开/关闭剪切。切换本身是一次**精确重投影**，不改变剖面。
    fn set_shear(&mut self, on: bool) {
        if on == self.cfg.shear {
            return;
        }
        let dk = if on {
            self.k - self.k_ref
        } else {
            -self.k_ref
        };
        self.reproject(dk);
        self.k_ref = if on { self.k } else { 0.0 };
        self.cfg.shear = on;
    }

    /// 把存储矩从参考 `k_ref` 重投影到 `k_ref + dk`：
    /// `U_s -> U_s + dk·V`（物理 `V = V' + v_off`，历史与当前样本都要补）。
    fn reproject(&mut self, dk: S) {
        if dk == 0.0 {
            return;
        }
        self.m.shear(dk);
        self.u.add(self.v.val().mul(dk));
        if self.v_off.norm2() > 0.0 {
            self.u.add(self.v_off.mul(dk));
            self.m.translate_u(self.v_off.mul(-dk));
        }
        self.k_ref += dk;
    }

    /// 在存储矩 `m` 的 `κ = kap` 处组装剖面系数。
    ///
    /// 物理 `V = V' + v_off`，所以 `κ` 处等价于先把 `U` 平移 `κ·v_off`。
    /// 剪切模式下 `kap` 恒为 0，这里就是 `m.coef()`，没有任何额外开销。
    fn coef_at(&self, m: &Moments, kap: S) -> Coef {
        if kap == 0.0 {
            return m.coef();
        }
        let mut mm = *m;
        if self.v_off.norm2() > 0.0 {
            mm.translate_u(self.v_off.mul(-kap));
        }
        mm.coef_at(kap)
    }

    /// 把**最新样本**搬到局部原点：`U_s -> U_s - u`、`V' -> V' - v`。
    ///
    /// 两个平移都是精确变换；`off` / `v_off` 累加被搬走的偏移，绝对坐标诊断
    /// 不受影响。之后 `p_local() == 0`，下一个周期的增量从 0 起算。
    fn recenter_latest(&mut self) {
        let (ui, vi) = (self.u.val(), self.v.val());
        if !ui.finite() || !vi.finite() || (ui.norm2() == 0.0 && vi.norm2() == 0.0) {
            return;
        }
        self.m.translate_u(ui);
        self.m.translate_v(vi);
        self.u.shift(ui);
        self.v.shift(vi);
        self.off = self.off.add(ui);
        self.v_off = self.v_off.add(vi);
    }

    /// 一个降采样周期：积分增量入矩 → 最新样本搬原点 → **一次**带可观性权重
    /// 的 Newton → 再搬一次原点。返回 `(报告, 局部圆心)`。
    ///
    /// `delta_u` / `delta_v` 是快速环自上个周期以来对 `s0` 与 `i` 的积分（用户
    /// 单位），`weight` 是这段时间内 `Σ(slope_dt/Δt)²`。
    pub fn step_once(
        &mut self,
        delta_u: V2,
        delta_v: V2,
        t: S,
        weight: S,
    ) -> Option<(Report, V2)> {
        // 1. 遗忘（按降采样时间推进）
        let rho = if self.cfg.tau > 0.0 {
            if self.started {
                (-(t - self.t_last).max(0.0) / self.cfg.tau).exp()
            } else {
                1.0
            }
        } else {
            1.0
        };
        self.t_last = t;
        self.started = true;
        self.m.decay(rho);
        self.w2 *= rho * rho;
        let inv = 1.0 / self.unit_len;
        let du = delta_u.mul(inv);
        let dv = delta_v.mul(inv);
        let du_p = du.add(dv.mul(self.k));
        let du_s = if self.cfg.shear { du_p } else { du };
        self.u.add(du_s);
        self.v.add(dv);
        self.path += du_p.norm();

        let (ui, vi) = (self.u.val(), self.v.val());
        let q = if weight.is_finite() && weight > 0.0 {
            weight
        } else {
            1.0
        };
        self.m.add(ui, vi, q);
        self.w2 += q * q;

        // 3. 最新样本搬到原点（U 与 V 各一次精确平移）
        self.recenter_latest();

        // 4. 恰好一次 Newton；可观性只缩放步长，不硬冻结
        self.steps = self.newton_weighted();

        // 5. Newton 可能改了 `k`（剪切模式下矩也被重投影）；再搬一次原点，
        //    保证下一周期的增量从 0 开始。
        self.recenter_latest();

        self.report_pair()
    }

    /// 初始化网格扫描：在 `[k0/scan_rel, k0·scan_rel]` 上取对数网格，选剖面
    /// 损失最小且曲率为正的点，把 `k` 移过去（剪切模式下同时精确重投影矩），
    /// 然后跑几步 Newton。
    ///
    /// 只在窗口足够长（`Σw` 与弧长由调用方把关）时调用。稳态数据上
    /// `P(k) ≡ 0`，扫描找不到显著更优的点，于是保持初始 `k` 不动并返回
    /// `false`——调用方可以稍后重试（等真正的瞬态到来）。
    pub fn initialize(&mut self) -> bool {
        let kappa = self.k - self.k_ref;
        let Some((_, f_cur, _)) = self.fit_hinge(&self.m, kappa) else {
            return false;
        };
        let lo = (self.k_initial / self.cfg.scan_rel.max(1.0)).max(self.cfg.k_min);
        let hi = (self.k_initial * self.cfg.scan_rel.max(1.0)).min(self.cfg.k_max);
        if !(hi > lo) || !lo.is_finite() || !hi.is_finite() {
            return false;
        }
        let n = self.cfg.scan_points.max(2);
        let mut best: Option<(S, S)> = None; // (P, k)
        for i in 0..n {
            let f = i as S / (n - 1) as S;
            let k = lo * (hi / lo).powf(f);
            if k == self.k {
                continue;
            }
            // 扫描目标是**带半径罚项**的剖面：半径越界时不能去追一个
            // 数据上更圆、但半径更越界的盆地。
            if let Some((_, fit, _)) = self.fit_hinge_at(k) {
                if fit.p.is_finite() && fit.p >= 0.0 && best.map_or(true, |(bp, _)| fit.p < bp) {
                    best = Some((fit.p, k));
                }
            }
        }
        let Some((p_best, k_best)) = best else {
            return false;
        };

        // 只有“显著更优 + 该点曲率为正（确实是一个极小点）”才接受，
        // 否则稳态下的噪声会把 k 随机跳走。
        if !(p_best < self.cfg.scan_gain * f_cur.p) {
            return false;
        }
        // 候选点的评估：剪切模式沿用原来的“先剪切到候选 k，再在 κ=0 求值”；
        // 非剪切模式直接在 κ=k_best 展开（矩保持原始 `U`，不能再剪切一次）。
        let (m_best, kappa_best) = if self.cfg.shear {
            (self.moments_at(k_best), 0.0)
        } else {
            (self.m, k_best - self.k_ref)
        };
        let Some((_, _, f0_best)) = self.fit_hinge(&m_best, kappa_best) else {
            return false;
        };
        // 可辨识性与噪声门限只看**无约束**剖面：半径罚项是先验，不能替数据
        // 证明“这个 k 是数据支持的”。
        let v2 = v_second(&m_best, self.v_off);
        let scale = 4.0 * f0_best.radius * f0_best.radius * v2;
        let obs = if scale > 0.0 && scale.is_finite() {
            f0_best.p2 / scale
        } else {
            0.0
        };
        if !obs.is_finite() || obs < self.cfg.scan_obs_min || !(v2 > 0.0) {
            return false;
        }
        // 圆模型可信度：高噪声下“更优的盆地”只是噪声，不能把 k 带跑。
        let sigma_rel = (f0_best.p.max(0.0) / m_best.w.max(1.0e-300)).sqrt()
            / (2.0 * f0_best.radius.max(1.0e-12));
        if !sigma_rel.is_finite() || sigma_rel > self.cfg.max_sigma_rel {
            return false;
        }

        let dk = k_best - self.k;
        if self.cfg.shear {
            self.reproject(dk);
        }
        self.k = k_best;
        self.obs = obs;
        // 用较低的扫描门限把网格点抛光到局部极小；之后逐拍 Newton 仍用
        // `obs_min` 的严门限，避免稳态噪声随机游走。
        let obs_min = self.cfg.obs_min;
        self.cfg.obs_min = self.cfg.scan_obs_min;
        self.newton(8);
        self.cfg.obs_min = obs_min;
        true
    }

    fn recenter(&mut self) {
        let w = self.m.w;
        if !(w > 0.0) {
            return;
        }
        let s = V2::new(self.m.ux, self.m.uy).mul(1.0 / w);
        if !s.finite() || s.norm() <= self.cfg.recenter_eps {
            return;
        }
        self.m.translate_u(s);
        self.u.shift(s);
        self.off = self.off.add(s);
    }

    /// `V` 的原点管理：存储 `V'` 的加权均值超过它自身 RMS 的
    /// `recenter_v_rel` 倍时，把均值搬进 `v_off`（物理 `V = V' + v_off`）。
    ///
    /// 这是纯规范：`P(k)`、`k`、圆心、半径都不变；作用只是不让 `V = ∫i dt` 的
    /// 积分漂移把 `M₁/M₂/q₁..q₃` 的量级撑大。阈值取相对量，纯旋转的 `V` 不会
    /// 触发，只有积分漂移才会。
    fn recenter_v(&mut self) {
        let w = self.m.w;
        if !(w > 0.0) {
            return;
        }
        let c = V2::new(self.m.vx, self.m.vy).mul(1.0 / w);
        let rms = (self.m.g / w).sqrt();
        if !c.finite() || !rms.is_finite() || c.norm() <= self.cfg.recenter_v_rel * rms {
            return;
        }
        self.m.translate_v(c);
        self.v.shift(c);
        self.v_off = self.v_off.add(c);
    }

    /// 把存储矩精确重投影到候选 `k`（剪切模式），供扫描/线搜索在 `κ = 0`
    /// 处求值，保持原有的数值行为。
    fn moments_at(&self, k: S) -> Moments {
        let dk = k - self.k_ref;
        let mut m = self.m;
        if dk != 0.0 {
            m.shear(dk);
            if self.v_off.norm2() > 0.0 {
                m.translate_u(self.v_off.mul(-dk));
            }
        }
        m
    }

    /// 在 `m` 上先做无约束拟合；半径越界时把线性化罚项加进法方程再解一次。
    ///
    /// `kappa = k - k_ref` 是求值点：剪切模式下恒为 0（`m` 已被剪切到当前
    /// `k`），非剪切模式下是当前 `k` 的绝对值。返回 `(罚项, 带罚项解, 无约束解)`：
    /// 可辨识性 / 噪声门限用无约束解，Newton 步与线搜索用带罚项解。
    fn fit_hinge(
        &self,
        m: &Moments,
        kappa: S,
    ) -> Option<(Option<RadiusPenalty>, Profile, Profile)> {
        let co = self.coef_at(m, kappa);
        let f0 = fit_from_coef(&co)?;
        match self.radius_penalty(&f0, m.w) {
            None => Some((None, f0, f0)),
            Some(pen) => {
                let f = fit_with_penalty_coef(&co, Some(pen))?;
                Some((Some(pen), f, f0))
            }
        }
    }

    /// 半径范围写成对 `θ` 的线性化伪观测 `gᵀθ = z`；`R²` 在范围内时返回 `None`。
    ///
    /// 只在越界时触发（hinge）：正常情况下完全不改变解。
    fn radius_penalty(&self, f: &Profile, w: S) -> Option<RadiusPenalty> {
        let r2 = 0.25 * (f.theta[0] * f.theta[0] + f.theta[1] * f.theta[1]) - f.theta[2];
        let r2min = self.rmin_i * self.rmin_i;
        let r2max = self.rmax_i * self.rmax_i;
        if !r2.is_finite() || (r2 >= r2min && r2 <= r2max) {
            return None;
        }
        let target = if r2 < r2min { r2min } else { r2max };
        let g = [0.5 * f.theta[0], 0.5 * f.theta[1], -1.0];
        let z = target - r2 + dot3(&g, &f.theta);
        Some(RadiusPenalty {
            g,
            z,
            mu: self.cfg.r_kappa * w.max(1.0e-300),
        })
    }

    /// 任意 `k` 处的 `fit_hinge`。剪切模式把矩剪切到该点、在 `κ = 0` 求值；
    /// 非剪切模式直接在 `κ = k` 展开。
    fn fit_hinge_at(&self, k: S) -> Option<(Option<RadiusPenalty>, Profile, Profile)> {
        if self.cfg.shear {
            self.fit_hinge(&self.moments_at(k), 0.0)
        } else {
            self.fit_hinge(&self.m, k - self.k_ref)
        }
    }

    /// 固定罚项的剖面损失。线搜索必须与 Newton 步用同一个目标，否则
    /// “`P` 下降”与“带罚项 `P` 下降”会互相打架。
    fn p_at_with_penalty(&self, k: S, pen: Option<RadiusPenalty>) -> Option<S> {
        let co = if self.cfg.shear {
            self.moments_at(k).coef()
        } else {
            self.coef_at(&self.m, k - self.k_ref)
        };
        fit_with_penalty_coef(&co, pen).map(|f| f.p)
    }

    fn newton(&mut self, iters: usize) -> u32 {
        self.newton_impl(iters, false)
    }

    /// 降采样周期的单次 Newton：`obs` 只做**步长系数**
    /// `w = clamp(|obs|/obs_min,0,1)²`，不设任何基于 `obs` 或 `sigma` 的冻结门限。
    ///
    /// 稳态 `obs→0` 时步长→0（近似慢冻结），瞬态 `obs≥obs_min` 时全步。
    /// 保留 `newton()` 的旧行为（硬门限）做对照。
    fn newton_weighted(&mut self) -> u32 {
        self.newton_impl(1, true)
    }

    fn newton_impl(&mut self, iters: usize, weighted: bool) -> u32 {
        // 3 个有效样本必共圆，`k` 没有信息。
        if self.m.w < self.cfg.min_weight {
            return 0;
        }
        let k_start = self.k;
        let mut taken = 0;
        for _ in 0..iters {
            // 无约束解决定门限，带罚项解决定步长：半径越界时 Newton 会把 k
            // 一起拉向可行域，而不是只把输出半径按到边界上。
            let kappa = self.k - self.k_ref;
            let (pen, f, f0) = match self.fit_hinge(&self.m, kappa) {
                Some(x) => x,
                None => break,
            };
            // 满激励时 `P'' = 2Σw(∂r/∂k)² + ...`，而 `|∂r/∂k| <= 2R|V|`，
            // 用 `4R²Σw|V|²` 作尺度得到无量纲可观性指数。门限一律用**无约束**
            // 剖面：半径罚项只会增加曲率，不能拿它冒充“k 被数据辨识出来了”。
            let scale = 4.0 * f0.radius * f0.radius * self.v_second();
            self.obs = if scale > 0.0 && scale.is_finite() {
                f0.p2 / scale
            } else {
                0.0
            };
            let w_obs = if weighted {
                if !self.obs.is_finite() {
                    break;
                }
                // `obs < 0` 表示当前点剖面是凹的；先验项会把分母拉正，等价于
                // 一个下坡梯度步，再由线搜索兜底。
                //
                // 用平方而不是线性：线性权重在 1 mA 稳态（obs≈4e-5、obs_min=5e-3）
                // 仍会累积 +1.6%/8 s 的 R 漂移；平方后压到 <0.1%，而瞬态
                // （obs≥obs_min）仍然是全步。线性版本保留在这个注释里供对比：
                // `let w = (obs/obs_min).clamp(0,1);`
                let w = (self.obs.abs() / self.cfg.obs_min.max(1.0e-12)).clamp(0.0, 1.0);
                w * w
            } else {
                if !self.obs.is_finite() || self.obs < self.cfg.obs_min {
                    break;
                }
                1.0
            };
            if w_obs <= 0.0 {
                break;
            }
            // 噪声可信度软权重：`w_sigma = clamp((σ_max/σ)²,0,1)`。
            // 可选的硬门限 `gate_sigma_rel` 与它**同时**生效（不是二选一）。
            let sigma_rel = (f0.p.max(0.0) / self.m.w.max(1.0e-300)).sqrt()
                / (2.0 * f0.radius.max(1.0e-12));
            let w_sigma = if weighted {
                (self.cfg.max_sigma_rel / sigma_rel.max(1.0e-12))
                    .max(0.0)
                    .powi(2)
                    .clamp(0.0, 1.0)
            } else if !sigma_rel.is_finite() || sigma_rel > self.cfg.max_sigma_rel {
                break;
            } else {
                1.0
            };
            if w_sigma <= 0.0 {
                break;
            }
            if weighted && self.cfg.gate_sigma_rel && sigma_rel > self.cfg.max_sigma_rel {
                break;
            }

            // 随机游走先验：等价于给法方程加阻尼。以本拍起点为参考，
            // 激励小时步长自动收缩。
            let prior = self.cfg.k_prior_rel.max(0.0) * scale;
            let denom = f.p2 + 2.0 * prior;
            if !denom.is_finite() || denom <= self.cfg.p2_eps {
                break;
            }
            let raw_step = -(f.p1 + 2.0 * prior * (self.k - k_start)) / denom;
            if !raw_step.is_finite() {
                break;
            }
            // 统计显著性：
            //   σ_k = sqrt(2P / ((N_eff-3)·P''))，N_eff = (Σw)²/Σw²
            //   w_sig = δ²/(δ²+σ_k²)   （δ 是原始 Newton 步长）
            // 权重始终生效（1σ→0.5、2σ→0.8、10σ→0.99）；`k_sigma>0` 时再叠加
            // 一层可选硬门限 `|δ| < k_sigma·σ_k`。两者同时使用。
            let mut sigma_k = S::INFINITY;
            let mut w_sig = 1.0;
            if weighted && f0.p2 > self.cfg.p2_eps {
                let n_eff = if self.w2 > 0.0 {
                    self.m.w * self.m.w / self.w2
                } else {
                    0.0
                };
                let dof = (n_eff - 3.0).max(1.0);
                sigma_k = (2.0 * f0.p.max(0.0) / (dof * f0.p2)).sqrt();
                if sigma_k.is_finite() {
                    if sigma_k > 0.0 {
                        let r = raw_step / sigma_k;
                        w_sig = r * r / (1.0 + r * r);
                    }
                } else {
                    // σ_k 溢出（P 极大/曲率极小）：权重取 0，避免无效大步
                    w_sig = 0.0;
                    sigma_k = S::INFINITY;
                }
            }
            if weighted
                && self.cfg.k_sigma > 0.0
                && sigma_k.is_finite()
                && sigma_k > 0.0
                && raw_step.abs() < self.cfg.k_sigma * sigma_k
            {
                break;
            }
            let mut step = raw_step * w_obs * w_sig * w_sigma;
            if !step.is_finite() {
                break;
            }
            let lim = self.cfg.k_step_rel * (1.0 + self.k.abs());
            step = step.clamp(-lim, lim);
            if step.abs() <= 1.0e-14 * (1.0 + self.k.abs()) {
                break;
            }

            // 回溯线搜索：只接受让**同一个（带罚项）目标**下降的步长。
            let p_cur = f.p;
            let mut s = step;
            let mut ok = false;
            for _ in 0..10 {
                let k_try = (self.k + s).clamp(self.cfg.k_min, self.cfg.k_max);
                if k_try == self.k {
                    break;
                }
                match self.p_at_with_penalty(k_try, pen) {
                    Some(pt) if pt.is_finite() && pt <= p_cur => {
                        s = k_try - self.k;
                        ok = true;
                        break;
                    }
                    _ => s *= 0.5,
                }
            }
            if !ok {
                break;
            }

            let k_new = self.k + s;
            let dk = k_new - self.k;
            if dk == 0.0 {
                break;
            }
            // 精确地把历史重新投影到新的 k（非剪切模式下矩本来就存原始 U，
            // 不需要动，只是在 κ=k 处求值）。
            if self.cfg.shear {
                self.reproject(dk);
            }
            self.k = k_new;
            taken += 1;
            if dk.abs() < 1e-14 * (1.0 + self.k.abs()) {
                break;
            }
        }
        taken
    }

    fn report(&self) -> Option<Report> {
        self.report_pair().map(|(r, _)| r)
    }

    /// 报告 + **局部圆心**：后者是当前参考 `k` 下、以最新样本为原点（内部
    /// 单位）的拟合圆心，供快速环做“目标相对圆心”。
    ///
    /// 不剪切时 `recenter_latest` 把最新样本的局部坐标清成 `(0, 0)`，但它的
    /// 物理位置是 `κ·v_off`，所以返回 `c - κ·v_off`；剪切模式 `κ = 0` 时就是 `c`。
    fn report_pair(&self) -> Option<(Report, V2)> {
        let kappa = self.k - self.k_ref;
        let co = self.coef_at(&self.m, kappa);
        let f0 = fit_from_coef(&co)?;
        let scale = 4.0 * f0.radius * f0.radius * v_second(&self.m, self.v_off);
        let obs = if scale > 0.0 && scale.is_finite() {
            f0.p2 / scale
        } else {
            0.0
        };
        let pen = self.radius_penalty(&f0, self.m.w);
        let (theta, clamped) = match pen {
            None => (f0.theta, false),
            Some(p) => match fit_with_penalty_coef(&co, Some(p)) {
                Some(f) => (f.theta, true),
                // 罚项方程奇异时退回无约束解：宁可输出一个越界半径，
                // 也不要丢掉这一拍的圆心 / 角度。
                None => (f0.theta, false),
            },
        };
        let c = V2::new(-0.5 * theta[0], -0.5 * theta[1]);
        let r2 = c.norm2() - theta[2];
        if !(r2 > 0.0) || !r2.is_finite() {
            return None;
        }
        let r = r2.sqrt();
        // 物理 `p_local = U_s + κ(V' + v_off)`：剪切模式 `κ = 0` 时就是存储
        // 的 `U_s`；非剪切模式 `κ = k`，要补上 `k·V`。
        let p_loc = if kappa == 0.0 {
            self.u.val()
        } else {
            self.u
                .val()
                .add(self.v.val().mul(kappa))
                .add(self.v_off.mul(kappa))
        };
        let rel = p_loc.sub(c);
        let dist = rel.norm();
        let l = self.unit_len;
        let w = self.m.w.max(1.0e-300);
        // 损失与圆模型可信度都取**无罚项**残差（在约束后的 θ 上求值）：
        // 罚项常数 μz² 不能混进 P/Σw，否则 `sigma_rel` 门限会失真。
        let loss = loss_from_coef(&co, &theta);
        let sigma_rel = (loss.max(0.0) / w).sqrt() / (2.0 * r.max(1.0e-12));
        // 与 Newton 门限同一个量：k 的标准误（诊断/调参用）。
        let n_eff = if self.w2 > 0.0 {
            self.m.w * self.m.w / self.w2
        } else {
            0.0
        };
        let dof = (n_eff - 3.0).max(1.0);
        let sigma_k = if f0.p2 > self.cfg.p2_eps {
            (2.0 * f0.p.max(0.0) / (dof * f0.p2)).sqrt()
        } else {
            S::INFINITY
        };

        Some((Report {
            k: self.k,
            radius: r * l,
            center: c.add(self.off).mul(l),
            p_rel: rel.mul(l),
            dist: dist * l,
            radial_err: (dist - r) * l,
            loss: loss / w,
            sigma_rel,
            sigma_k,
            weight: self.m.w,
            p1: f0.p1,
            p2: f0.p2,
            obs,
            steps: self.steps,
            clamped,
        }, c.sub(self.v_off.mul(kappa))))
    }
}

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
    /// 剖面一阶/二阶导（诊断：无剪切时 `P'` 的绝对值会因相消而明显偏大）。
    pub p1: S,
    pub p2: S,
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
            //
            // 也不再需要剪切：每个拟合周期都把最新样本搬到原点（`U`、`V`
            // 各一次精确平移），矩里的 `|U|`、`|V|` 始终是“一个拟合周期内的
            // 增量”量级，`κ = k` 处的三/四次展开没有相消问题（实测 0.5~100
            // rad/s、1~10 mA、拟合周期放大到 0.2 s 都与剪切逐位一致）。
            // 保留 `shear = true` 作为 A/B 开关。
            fit: FitConfig {
                k_prior_rel: 0.0,
                shear: false,
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
            p1: 0.0,
            p2: 0.0,
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
        self.p1 = 0.0;
        self.p2 = 0.0;
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
                    self.p1 = rep.p1;
                    self.p2 = rep.p2;
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

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::*;
    use crate::simulation::inverse_clarke;

    /// 合成“磁链圆”流：返回每个样本的 `(p_ref, ΔU, ΔV)`。
    ///
    /// `s0 = -(R i + ω ρ j n)`，因此 `A + R B = -(λ - λ0)/L` 严格成立。
    fn synth_stream(
        r: S,
        radius: S,
        omega: S,
        iq: impl Fn(S) -> S,
        dur: S,
        dt: S,
    ) -> Vec<(V2, V2, V2)> {
        let n = (dur / dt) as usize;
        let mut a = V2::ZERO;
        let mut b = V2::ZERO;
        let mut out = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as S * dt;
            let th = omega * t;
            let (sin, cos) = th.sin_cos();
            let jn = V2::new(-sin, cos);
            let i = jn.mul(iq(t));
            let s = i.mul(r).add(jn.mul(omega * radius)).mul(-1.0);
            let du = s.mul(dt);
            let dv = i.mul(dt);
            a = a.add(du);
            b = b.add(dv);
            out.push((a.add(b.mul(r)), du, dv));
        }
        out
    }

    fn tracker_for(r: S, radius: S, k0: S) -> Tracker {
        let mut cfg = FitConfig::default();
        cfg.tau = 2.0;
        cfg.r_min = 0.3 * radius;
        cfg.r_max = 3.0 * radius;
        cfg.k_min = 0.1 * r;
        cfg.k_max = 10.0 * r;
        cfg.obs_min = 1.0e-4;
        cfg.min_weight = 8.0;
        Tracker::new(cfg, k0)
    }

    #[test]
    fn shear_matches_direct() {
        let (mut a, mut samples) = (Moments::default(), Vec::new());
        for k in 0..40u32 {
            let t = k as S * 0.3;
            let u = V2::new(t.cos() * 1.5 + 0.1 * t.sin(), t.sin() * 1.5);
            let v = V2::new(0.2 * t.cos(), -0.3 * t.sin());
            a.add(u, v, 1.0);
            samples.push((u, v));
        }
        let lam = 0.37;
        let before = a.clone();
        a.shear(lam);
        let mut b = Moments::default();
        for (u, v) in &samples {
            b.add(u.add(v.mul(lam)), *v, 1.0);
        }
        assert_close(&a, &b, "shear", 1e-12);
        assert!((before.w - a.w).abs() < 1e-12);
        assert!((before.g - a.g).abs() < 1e-9);
    }

    #[test]
    fn translate_matches_direct() {
        let (mut a, mut samples) = (Moments::default(), Vec::new());
        for k in 0..40u32 {
            let t = k as S * 0.3;
            let u = V2::new(t.cos() * 1.5 + 0.7, t.sin() * 1.5 - 0.4);
            let v = V2::new(0.2 * t.cos(), -0.3 * t.sin());
            a.add(u, v, 1.0);
            samples.push((u, v));
        }
        let s = V2::new(0.83, -1.21);
        a.translate_u(s);
        let mut b = Moments::default();
        for (u, v) in &samples {
            b.add(u.sub(s), *v, 1.0);
        }
        assert_close(&a, &b, "translate", 1e-12);
    }

    #[test]
    fn profile_derivatives_match_finite_difference() {
        let mut a = Moments::default();
        for k in 0..60u32 {
            let t = k as S * 0.27;
            let u = V2::new(1.3 * t.cos(), 1.3 * t.sin());
            let v = V2::new(0.4 * t.cos() + 0.2 * t.sin(), 0.4 * t.sin() - 0.2 * t.cos());
            a.add(u, v, 1.0);
        }
        let p_at = |k: S| {
            let mut c = a.clone();
            c.shear(k);
            c.fit().unwrap().p
        };
        for &k0 in &[0.2, 0.5, 0.8] {
            let mut c = a.clone();
            c.shear(k0);
            let f = c.fit().unwrap();
            let h = 1e-4;
            let pm = p_at(k0 - h);
            let pp = p_at(k0 + h);
            let fd1 = (pp - pm) / (2.0 * h);
            let fd2 = (pp - 2.0 * f.p + pm) / (h * h);
            let s1 = (f.p1 - fd1).abs() / (1.0 + fd1.abs());
            let s2 = (f.p2 - fd2).abs() / (1.0 + fd2.abs());
            assert!(s1 < 1e-5, "k0={k0} p1={} fd={fd1}", f.p1);
            assert!(s2 < 1e-4, "k0={k0} p2={} fd={fd2}", f.p2);
        }
    }

    #[test]
    fn tracker_recovers_r_with_excitation() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let stream = synth_stream(r, radius, 100.0, |t| 3.0 + 1.5 * (TAU * 0.7 * t).cos(), 3.0, dt);
        let mut tr = tracker_for(r, radius, 1.25 * r);
        let mut scan_done = false;
        for (k, (_, du, dv)) in stream.iter().enumerate() {
            let t = k as S * dt;
            let iters = if t >= 1.0 { 2 } else { 0 };
            tr.update(*du, *dv, t, 1.0, iters);
            if !scan_done && t >= 1.0 {
                scan_done = tr.initialize();
            }
        }
        let rel = (tr.k() - r).abs() / r;
        eprintln!(
            "recover: k={:.3} true={r:.3} rel={rel:.4} obs={:.2e} w={:.1}",
            tr.k(),
            tr.obs(),
            tr.weight()
        );
        assert!(rel < 0.03, "k={} rel={rel}", tr.k());
    }

    #[test]
    fn tracker_freezes_r_at_steady_state() {
        // 恒 iq、恒 ω：P(k) ≡ 0，k 不可辨识，估计必须停在初值附近。
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let stream = synth_stream(r, radius, 100.0, |_| 3.0, 3.0, dt);
        let mut tr = tracker_for(r, radius, 1.25 * r);
        let mut scan_done = false;
        for (k, (_, du, dv)) in stream.iter().enumerate() {
            let t = k as S * dt;
            let iters = if t >= 1.0 { 2 } else { 0 };
            tr.update(*du, *dv, t, 1.0, iters);
            if !scan_done && t >= 1.0 {
                scan_done = tr.initialize();
            }
        }
        let rel = (tr.k() - 1.25 * r).abs() / r;
        eprintln!(
            "steady: k={:.3} (start {:.3}) rel={rel:.4} obs={:.2e} w={:.1}",
            1.25 * r,
            tr.k(),
            tr.obs(),
            tr.weight()
        );
        assert!(rel < 0.05, "k drifted to {} (rel {rel})", tr.k());
    }


    /// 确定性的对称随机数（xorshift64），只为让“随机 U”可复现。
    struct XorShift(u64);

    impl XorShift {
        fn new(seed: u64) -> Self {
            Self(seed | 1)
        }

        fn sym(&mut self) -> S {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            2.0 * (x >> 11) as S / ((1u64 << 53) as S) - 1.0
        }
    }

    /// 每个样本都满足 `p(k_true) = U + k_true·V` 严格落在半径 `r_true` 的圆上。
    ///
    /// 与论文 `radius_range_is_enforced` 同一构造：`U` 任意、`V = (p - U)/k`，
    /// 所以数据对 `k_true` 绝对共圆。只喂增量，检验流式递推。
    fn circle_stream(k_true: S, r_true: S, n: usize, seed: u64) -> Vec<(V2, V2)> {
        let mut rng = XorShift::new(seed);
        let (mut pu, mut pv) = (V2::ZERO, V2::ZERO);
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let th = i as S * 0.17;
            let target = V2::new(r_true * th.cos(), r_true * th.sin());
            let u = V2::new(1.5 * rng.sym(), 1.5 * rng.sym());
            let v = target.sub(u).mul(1.0 / k_true);
            out.push((u.sub(pu), v.sub(pv)));
            pu = u;
            pv = v;
        }
        out
    }

    fn config_for_range(r_min: S, r_max: S) -> FitConfig {
        let mut cfg = FitConfig::default();
        cfg.tau = 0.0; // 300 个样本的短窗口，不需要遗忘
        cfg.r_min = r_min;
        cfg.r_max = r_max;
        cfg.k_min = -50.0;
        cfg.k_max = 50.0;
        cfg.obs_min = 0.0;
        cfg.max_sigma_rel = 10.0;
        cfg.k_prior_rel = 0.0;
        cfg
    }

    /// 论文 §7 的半径伪观测必须进入**法方程/剖面**，而不只是修输出：
    /// 数据真实半径 1.35、配置范围 [1.0,1.2] 时，Newton 后的半径贴在 1.2，
    /// 且 `k` 被一起拉离无约束解。
    #[test]
    fn radius_penalty_enters_newton() {
        let k_true = 0.7;
        let r_true = 1.35;
        let stream = circle_stream(k_true, r_true, 300, 0x5eed);

        let mut tight = Tracker::new(config_for_range(1.0, 1.2), k_true);
        for (i, (du, dv)) in stream.iter().enumerate() {
            tight.update(*du, *dv, i as S * 0.01, 1.0, 0);
        }
        tight.newton(8);
        let rep = tight.report_now().expect("report");
        assert!(rep.clamped, "半径越界时罚项必须触发");
        assert!(
            (rep.radius - 1.2).abs() < 0.02,
            "radius {} 应被拉到 r_max=1.2",
            rep.radius
        );

        // 同一数据、同一初值的无约束对照。
        let mut free = Tracker::new(config_for_range(0.3 * r_true, 3.0 * r_true), k_true);
        for (i, (du, dv)) in stream.iter().enumerate() {
            free.update(*du, *dv, i as S * 0.01, 1.0, 0);
        }
        free.newton(8);
        let free_rep = free.report_now().expect("free report");
        assert!(!free_rep.clamped);
        assert!((free.k() - k_true).abs() < 1e-6, "free k={}", free.k());
        assert!(
            (free_rep.radius - r_true).abs() < 1e-6,
            "free radius {}",
            free_rep.radius
        );
        assert!(
            (tight.k() - free.k()).abs() > 1.0e-3,
            "罚项没有进入 Newton：tight k={} free k={}",
            tight.k(),
            free.k()
        );

        // 报告的损失必须是约束后 θ 上的**无罚项**残差：μz² 不能混进 P/Σw。
        let w = tight.m.w;
        let l = tight.unit_len();
        let c = rep.center.mul(1.0 / l).sub(tight.off());
        let r = rep.radius / l;
        let theta = [-2.0 * c.x, -2.0 * c.y, c.norm2() - r * r];
        let unpen = tight.m.loss_at(&theta);
        assert!(
            (unpen - rep.loss * w).abs() <= 1.0e-9 * (1.0 + unpen.abs()),
            "loss={} 不是约束后 θ 的无罚项残差 {}",
            rep.loss,
            unpen / w
        );
        let f0 = tight.m.fit().unwrap();
        let pen = tight.radius_penalty(&f0, w).expect("penalty");
        let fpen = fit_with_penalty(&tight.m, Some(pen)).unwrap();
        eprintln!(
            "radius: tight k={:.4} R={:.4} | free k={:.4} R={:.4} | loss={:.3e} unpen={:.3e} pen_p={:.3e}",
            tight.k(),
            rep.radius,
            free.k(),
            free_rep.radius,
            rep.loss,
            unpen / w,
            fpen.p / w
        );
        assert!(
            fpen.p > unpen * 1.001,
            "罚项常数没有体现在带罚项 P 上：{} vs {}",
            fpen.p,
            unpen
        );
    }

    /// 给所有 `V_n` 加同一个常数只是把 `p_n(κ)` 平移 `κc`：`P`、`P'`、`P''`
    /// 必须完全不变，`Σw|V - V̄|²` 也必须不变。这既是 V 原点管理（规范变换）
    /// 的正确性依据，也是可观性指数用中心化方差的依据（模块文档 §4）。
    #[test]
    fn v_constant_offset_is_a_gauge() {
        let c = V2::new(0.9, -1.4);
        let (mut a, mut b) = (Moments::default(), Moments::default());
        for i in 0..60u32 {
            let t = i as S * 0.23;
            let u = V2::new(1.3 * t.cos() + 0.2 * t.sin(), 1.3 * t.sin());
            let v = V2::new(
                0.35 * t.cos() + 0.15 * t.sin(),
                0.3 * t.sin() - 0.1 * t.cos(),
            );
            a.add(u, v, 1.0);
            b.add(u, v.add(c), 1.0);
        }
        let rel = |x: S, y: S| (x - y).abs() / (1.0 + x.abs().max(y.abs()));
        // 物理 V 的二阶矩与原点拆分无关：把 b 的偏移 c 加回去应当等于 a 的 g。
        assert!(rel(v_second(&b, V2::new(-c.x, -c.y)), a.g) < 1.0e-12);
        for &k in &[0.0, 0.37, 1.1] {
            let (mut ma, mut mb) = (a.clone(), b.clone());
            ma.shear(k);
            mb.shear(k);
            let (fa, fb) = (ma.fit().unwrap(), mb.fit().unwrap());
            assert!(rel(fa.p, fb.p) < 1.0e-12, "P 变了 (k={k})");
            assert!(rel(fa.p1, fb.p1) < 1.0e-11, "P' 变了 (k={k})");
            assert!(rel(fa.p2, fb.p2) < 1.0e-10, "P'' 变了 (k={k})");
        }
    }

    #[test]
    fn translate_v_matches_direct() {
        let (mut a, mut samples) = (Moments::default(), Vec::new());
        for k in 0..40u32 {
            let t = k as S * 0.3;
            let u = V2::new(t.cos() * 1.5 + 0.7, t.sin() * 1.5 - 0.4);
            let v = V2::new(0.2 * t.cos() + 0.05 * t, -0.3 * t.sin() + 0.1);
            a.add(u, v, 1.0);
            samples.push((u, v));
        }
        let s = V2::new(0.63, -0.91);
        a.translate_v(s);
        let mut b = Moments::default();
        for (u, v) in &samples {
            b.add(*u, v.sub(s), 1.0);
        }
        assert_close(&a, &b, "translate_v", 1e-12);
        assert!((a.g - b.g).abs() < 1e-9);
    }

    /// `V = ∫i dt` 的常数偏移是规范：注入一个偏置后 `k` 仍须从 +25% 初值恢复到
    /// 真值；`v_off` 负责把这个偏移从存储的 `V'` 里搬走。漏掉剪切时的
    /// `dk·v_off` 修正，这个恢复就会偏。
    #[test]
    fn v_offset_does_not_break_k() {
        let k_true = 0.7;
        let r_true = 1.35;
        let mut stream = circle_stream(k_true, r_true, 300, 0xc0ffee);
        stream[0].1 = stream[0].1.add(V2::new(2.0, -1.0));

        let mut tr = Tracker::new(config_for_range(0.3 * r_true, 3.0 * r_true), 1.25 * k_true);
        for (i, (du, dv)) in stream.iter().enumerate() {
            tr.update(*du, *dv, i as S * 0.01, 1.0, 2);
        }
        let rep = tr.report_now().expect("report");
        eprintln!(
            "v-offset: k={:.6} v_off={:?} R={:.4} obs={:.2e}",
            tr.k(),
            tr.v_off(),
            rep.radius,
            tr.obs()
        );
        assert!((tr.k() - k_true).abs() < 1.0e-3, "k={}", tr.k());
        assert!(tr.v_off().norm() > 0.5, "v_off 没生效: {:?}", tr.v_off());
        assert!((rep.radius - r_true).abs() < 1.0e-3, "R={}", rep.radius);
    }

    /// `coef_at(κ)` 必须与“把矩剪切到 κ 再在 κ=0 求值”给出同一个剖面。
    #[test]
    fn coef_at_matches_shear_round_trip() {
        let k = 384.0;
        let mut rng = XorShift::new(0x1234_5678);
        let mut m = Moments::default();
        for i in 0..300u32 {
            let th = i as S * 0.07;
            let target = V2::new(1.35 * th.cos(), 1.35 * th.sin());
            let u = V2::new(1.5 * rng.sym(), 1.5 * rng.sym());
            let noise = V2::new(0.002 * rng.sym(), 0.002 * rng.sym());
            let v = target.add(noise).sub(u).mul(1.0 / k);
            m.add(u, v, 1.0);
        }
        let rel = |x: S, y: S| (x - y).abs() / (1.0 + x.abs().max(y.abs()));
        for &kappa in &[0.0, 0.7, 20.0, 384.0] {
            let mut ms = m;
            ms.shear(kappa);
            let a = fit_from_coef(&m.coef_at(kappa)).unwrap();
            let b = fit_from_coef(&ms.coef()).unwrap();
            assert!(rel(a.radius, b.radius) < 1.0e-9, "κ={kappa} radius");
            assert!(rel(a.p, b.p) < 1.0e-6, "κ={kappa} P {} vs {}", a.p, b.p);
            assert!(rel(a.p1, b.p1) < 1.0e-6, "κ={kappa} P'");
            assert!(rel(a.p2, b.p2) < 1.0e-6, "κ={kappa} P''");
            for i in 0..3 {
                assert!(
                    (a.theta[i] - b.theta[i]).abs() < 1.0e-9,
                    "κ={kappa} θ[{i}]"
                );
            }
        }
    }

    fn run_tracker_with_shear(shear: bool, stream: &[(V2, V2, V2)], r: S, radius: S) -> (S, S) {
        let mut cfg = FitConfig::default();
        cfg.tau = 2.0;
        cfg.r_min = 0.3 * radius;
        cfg.r_max = 3.0 * radius;
        cfg.k_min = 0.1 * r;
        cfg.k_max = 10.0 * r;
        cfg.obs_min = 1.0e-4;
        cfg.shear = shear;
        let mut tr = Tracker::new(cfg, 1.25 * r);
        let mut scan = false;
        for (i, (_, du, dv)) in stream.iter().enumerate() {
            let t = i as S * 5.0e-5;
            tr.update(*du, *dv, t, 1.0, if t >= 1.0 { 2 } else { 0 });
            if !scan && t >= 1.0 {
                scan = tr.initialize();
            }
        }
        let rep = tr.report_now().unwrap();
        (tr.k(), rep.radius)
    }

    /// 无噪声数据上，两种存储方式必须收敛到同一个 `k` 与半径。
    #[test]
    fn no_shear_tracker_matches_shear() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let stream = synth_stream(
            r,
            radius,
            100.0,
            |t| 3.0 + 1.5 * (TAU * 0.7 * t).cos(),
            3.0,
            dt,
        );
        let (ks, rs) = run_tracker_with_shear(true, &stream, r, radius);
        let (kn, rn) = run_tracker_with_shear(false, &stream, r, radius);
        eprintln!("shear k={ks:.5} R={rs:.6} | no-shear k={kn:.5} R={rn:.6}");

        assert!((ks - kn).abs() / r < 1.0e-5, "k: {ks} vs {kn}");
        assert!(
            (rs - rn).abs() / radius < 1.0e-5,
            "radius: {rs} vs {rn}"
        );
    }

    /// 运行中切换剪切必须是一次精确重投影：报告里的 `k`、圆心、半径、损失不变。
    #[test]
    fn set_shear_switch_preserves_profile() {
        let r = 384.0;
        let radius = 4.04;
        let dt = 5.0e-5;
        let stream = synth_stream(
            r,
            radius,
            100.0,
            |t| 3.0 + 1.5 * (TAU * 0.7 * t).cos(),
            2.0,
            dt,
        );
        let mut tr = tracker_for(r, radius, 1.25 * r);
        let mut scan = false;
        for (i, (_, du, dv)) in stream.iter().enumerate() {
            let t = i as S * dt;
            tr.update(*du, *dv, t, 1.0, if t >= 1.0 { 2 } else { 0 });
            if !scan && t >= 1.0 {
                scan = tr.initialize();
            }
        }
        let a = tr.report_now().unwrap();
        tr.set_shear(false);
        let b = tr.report_now().unwrap();
        tr.set_shear(true);
        let c = tr.report_now().unwrap();
        let rel = |x: S, y: S| (x - y).abs() / (1.0 + x.abs().max(y.abs()));
        // 剪切是精确变换，只差舍入；|k| ~ 400，用 1e-12 的绝对尺度不现实。
        assert!(rel(a.radius, b.radius) < 1.0e-10);
        assert!(rel(a.radius, c.radius) < 1.0e-10);
        assert!(rel(a.k, b.k) < 1.0e-12 && rel(a.k, c.k) < 1.0e-12);
        assert!(rel(a.center.x, b.center.x) < 1.0e-9);
        assert!(rel(a.center.x, c.center.x) < 1.0e-9);
        assert!(rel(a.loss, b.loss) < 1.0e-6);
        assert!(rel(a.loss, c.loss) < 1.0e-6);
        eprintln!(
            "switch: R {:.6}/{:.6}/{:.6} k {:.4}/{:.4}/{:.4}",
            a.radius, b.radius, c.radius, a.k, b.k, c.k
        );
    }

    fn assert_close(a: &Moments, b: &Moments, what: &str, tol: S) {
        let mut worst: S = 0.0;
        let mut who = "";
        let mut chk = |x: S, y: S, name: &'static str| {
            let d = (x - y).abs() / (1.0 + x.abs().max(y.abs()));
            if d > worst {
                worst = d;
                who = name;
            }
        };
        chk(a.w, b.w, "w");
        chk(a.ux, b.ux, "ux");
        chk(a.uy, b.uy, "uy");
        chk(a.ux2, b.ux2, "ux2");
        chk(a.ux_uy, b.ux_uy, "ux_uy");
        chk(a.uy2, b.uy2, "uy2");
        chk(a.vx, b.vx, "vx");
        chk(a.vy, b.vy, "vy");
        chk(a.vx2, b.vx2, "vx2");
        chk(a.vx_vy, b.vx_vy, "vx_vy");
        chk(a.vy2, b.vy2, "vy2");
        chk(a.ux_vx, b.ux_vx, "ux_vx");
        chk(a.ux_vy, b.ux_vy, "ux_vy");
        chk(a.uy_vx, b.uy_vx, "uy_vx");
        chk(a.uy_vy, b.uy_vy, "uy_vy");
        chk(a.a, b.a, "a");
        chk(a.a_ux, b.a_ux, "a_ux");
        chk(a.a_uy, b.a_uy, "a_uy");
        chk(a.a_vx, b.a_vx, "a_vx");
        chk(a.a_vy, b.a_vy, "a_vy");
        chk(a.b, b.b, "b");
        chk(a.b_ux, b.b_ux, "b_ux");
        chk(a.b_uy, b.b_uy, "b_uy");
        chk(a.b_vx, b.b_vx, "b_vx");
        chk(a.b_vy, b.b_vy, "b_vy");
        chk(a.g, b.g, "g");
        chk(a.g_ux, b.g_ux, "g_ux");
        chk(a.g_uy, b.g_uy, "g_uy");
        chk(a.g_vx, b.g_vx, "g_vx");
        chk(a.g_vy, b.g_vy, "g_vy");
        chk(a.a2, b.a2, "a2");
        chk(a.a_b, b.a_b, "a_b");
        chk(a.b2, b.b2, "b2");
        chk(a.a_g, b.a_g, "a_g");
        chk(a.b_g, b.b_g, "b_g");
        chk(a.g2, b.g2, "g2");
        assert!(worst < tol, "{what}: worst field {who} err {worst}");
    }


    /// 合成流：`omega` 可以随时间变化；返回真实电角度。
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

    /// 最新样本必须位于拟合局部坐标原点（交班的前置条件），两种存储方式都要。
    #[test]
    fn step_once_puts_latest_sample_at_origin() {
        let (r, radius, dt) = (384.0, 4.04, 5.0e-5);
        let stream = synth_stream_profile(r, radius, |_| 20.0, |_| 3.0, 0.2, dt);
        for shear in [true, false] {
            let mut cfg = FitConfig::default();
            cfg.r_min = 0.3 * radius;
            cfg.r_max = 3.0 * radius;
            cfg.k_min = 0.1 * r;
            cfg.k_max = 10.0 * r;
            cfg.tau = 10.0;
            cfg.shear = shear;
            let mut tr = Tracker::new(cfg, r);
            let (mut a, mut b) = (V2::ZERO, V2::ZERO);
            let (mut t, mut n, mut last) = (0.0, 0, None);
            for (_th, du, dv) in &stream {
                a = a.add(*du);
                b = b.add(*dv);
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
            eprintln!(
                "origin(shear={shear}): p_local={:?} c={:?} radius={:.4}",
                tr.p_local(),
                c,
                rep.radius
            );
            assert!(
                tr.p_local().norm() < 1.0e-12,
                "latest sample not at origin (shear={shear})"
            );
            assert!((rep.radius - radius).abs() < 0.01, "shear={shear}");
        }
    }

}
