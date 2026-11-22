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
//! * **拟合侧用泄漏积分器**：`u`、`v` 不再每个拟合周期被搬原点，而是**各自**
//!   以 `T_hp = hp_rel·tau` 的时间常数向零泄漏（等效一阶高通）。泄漏掉的量
//!   不保存：`u + k·v` 收敛到相对物理圆心的位置，局部帧原点本身就是圆心；
//!   36 个矩在稳态不再需要任何平移（旧实现每周期把 36 个矩平移两次，约 60 ns）。
//! * **加锁交班**：`Arc<Mutex<FastFlux>>` 内读并清零 `a`、`b`，同时
//!   `c += a + k_cur·b` 保持磁链连续；拟合完成后写三个目标
//!   `c_tgt = -ρ_fit·unit(c_local)`、`k_tgt = k_fit`、`rad_tgt = ρ_fit`。
//!   `c_local = c - (U + k·V)` 是拟合圆心相对**最新样本**的向量（泄漏积分器
//!   下最新样本不在原点，必须减掉 `p_local`）；交班只设定这个目标圆心，
//!   不再从拟合积分器里取值 / 清零。圆心目标**不直接用**拟合圆心 `c_local`：
//!   只用它的方向，长度强制成拟合半径 ρ（最新采样点严格落在半径 ρ 的圆上）。
//!   `R` 的交接乘可辨识性 `w_obs`（稳态不跟噪声游走），圆心/半径直接按
//!   `sync_gain` 交接（`obs` 描述的是 `k` 的可辨识性，不是圆心/半径的估计
//!   质量）。这样积分器清零不会让角度跳变，三个量也不会一次写死。
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
//! * 内部单位 + 泄漏积分器：`u`、`v` **分别**做 `u ← e^{-Δt/T_hp}(u+Δu)`、
//!   `v ← e^{-Δt/T_hp}(v+Δv)`，矩只在这组有界坐标里累加，稳态不再平移
//!   36 个矩。
//! * 在 `κ = k` 处把 `b` 展开到三次、`c` 到四次（用的仍是已有矩
//!   `g_vx/g_vy/b_g/g2`）。
//!
//!   `V` 的偏置**不进拟合**：给所有 `V_n` 加同一个常向量只是把每个 `κ` 的
//!   点集平移 `κc`，`P(k)`、`P'`、`P''`、`k`、半径都不变，所以 `V` 不需要单独
//!   保存偏置。可观性尺度取 `Σw|V|² = m.g`，
//!   它对 `V` 的常数偏置不变。
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

    /// 在 `κ = kap` 处展开的剖面 Taylor 系数。
    ///
    /// 矩里存的是原始 `U`、`V`，求值点在 `κ = k`，需要把
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
            [self.ux_vx * 2.0, self.ux_vy + self.uy_vx, self.vx],
            [self.ux_vy + self.uy_vx, self.uy_vy * 2.0, self.vy],
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

/// 在目标 `κ` 处求剖面（系数由 [`Moments::coef_at`] 给出）；`pen` 非空时把
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

// ===========================================================================
// 流式拟合器
// ===========================================================================

/// 流式 Kåsa 拟合配置。
#[derive(Clone, Copy, Debug)]
pub struct FitConfig {
    /// 指数遗忘时间常数（秒）。`<= 0` 表示不遗忘。
    pub tau: S,
    /// 拟合侧泄漏积分器的时间常数系数：`T_hp = hp_rel · tau`。
    ///
    /// `u`、`v` **分别**做 `u ← exp(-Δt/T_hp)·(u + Δu)`，等效一阶高通：
    /// 直流以 `T_hp` 衰减，交流只乘 `jωT_hp/(1+jωT_hp)`。它替代“每周期把
    /// 36 个矩搬原点”；泄漏掉的量不保存。`T_hp` 必须远大于权重衰减 `tau`
    /// 和数据窗口，否则帧原点在一个窗口内的移动会把圆拉成螺旋、偏置 `k`。
    /// 实测（含 `Tracker::seed_frame`）：`hp_rel` = 20/50/100/200 时
    /// 100 rad/s、1 mA、8 s 的 `R` 偏 +1.5% / +0.29% / +0.11% / +0.07%，
    /// 所以默认 200。`<= 0` 或 `tau <= 0` 表示不泄漏（纯积分，只靠遗忘
    /// 保证有界）。
    pub hp_rel: S,
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
            hp_rel: 200.0,
            r_min: 1.0,
            r_max: 3.0,
            k_min: -1.0e6,
            k_max: 1.0e6,
            k_step_rel: 0.02,
            k_prior_rel: 0.02,
            obs_min: 5.0e-3,
            scan_obs_min: 1.0e-4,
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
    /// 圆心（用户单位，**泄漏积分器局部帧**里的圆心）。
    ///
    /// 泄漏积分器不再保存绝对位置，所以这里不是物理绝对圆心：局部帧原点
    /// 以 `T_hp` 收敛到物理圆心，稳态时 `center ≈ 0`（启动阶段会从
    /// `radius·n(0)` 慢慢衰减到 0）。交班只用 `p_rel` / 局部圆心，不用它。
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
/// 存储原始 `U`、`V`，在 `κ = k` 处把剖面展开到三/四次（`b` 到 `κ³`、
/// `c` 到 `κ⁴`）。`u`、`v` 是**分别**泄漏的积分器（一阶高通），每个拟合周期
/// 只做一次 O(1) 更新，不再平移 36 个矩。泄漏掉的分量不保存：`u + k·v`
/// 收敛到相对物理圆心的位置，局部帧的原点本身就是那个圆心。
///
/// `V` 的常数偏置是纯规范（给所有 `V_n` 加 `c` 只把每个 `κ` 的点集平移
/// `κc`，`P`、`P'`、`P''`、`k`、半径都不变），所以 `V` 不单独保存偏置。
#[derive(Clone, Debug)]
pub struct Tracker {
    cfg: FitConfig,
    m: Moments,
    /// 内部长度单位 `(r_min + r_max)/2`。
    unit_len: S,
    rmin_i: S,
    rmax_i: S,
    /// 当前估计 `k = Rs/L`。
    k: S,
    /// `set_initial` 给的初始 `k`，网格扫描围绕它展开。
    k_initial: S,
    /// 泄漏积分器的 `U` 状态（内部单位）。
    u: V2,
    /// 泄漏积分器的 `V` 状态（内部单位）。
    v: V2,
    t_last: S,
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
            k_initial: k,
            u: V2::ZERO,
            v: V2::ZERO,
            t_last: 0.0,
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

    /// 用已知的启动圆心给泄漏积分器一个初值：`u = u0`、`v = 0`。
    ///
    /// 启动时磁链点在原点、圆心在 `radius·n(θ0)`，所以 `u + k·v` 的初值就是
    /// `-radius·n(θ0)/unit_len`。给对初值可以消掉 `T_hp` 尺度的启动瞬态
    /// （否则帧原点要先从 0 爬到圆心，窗口内的移动会把圆拉成螺旋）。
    pub fn seed_frame(&mut self, u0: V2) {
        self.u = u0;
        self.v = V2::ZERO;
    }

    /// 当前样本在局部参考系中的位置 `U + k·V`（内部单位）。
    #[inline]
    pub fn p_local(&self) -> V2 {
        self.u.add(self.v.mul(self.k))
    }

    /// 已累积的路径长度 `Σ|Δp|`（内部单位）。
    #[inline]
    pub fn path(&self) -> S {
        self.path
    }

    /// 喂入一对增量 `(ΔU, ΔV)`，返回拟合报告。
    ///
    /// * `t`：样本时刻（秒），只用于指数遗忘；
    /// * `weight`：该样本的拟合权重。
    ///
    /// 与 [`Tracker::step_once`] 走**同一条路径**（一次泄漏积分器更新 + 恰好一次
    /// 带可观性权重的 Newton），只是丢掉局部圆心；保留它作为“单样本一步”的
    /// 便捷入口，不再另有一套均值归零 / 变步数 Newton 的实现。
    pub fn update(&mut self, delta_u: V2, delta_v: V2, t: S, weight: S) -> Option<Report> {
        self.step_once(delta_u, delta_v, t, weight)
            .map(|(rep, _)| rep)
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
        self.cfg.hp_rel = cfg.hp_rel;
    }

    /// 一个降采样周期：泄漏积分器更新 → 积分增量入矩 → **一次**带可观性权重
    /// 的 Newton。返回 `(报告, 局部圆心)`；局部圆心是相对**最新样本**的向量。
    ///
    /// `delta_u` / `delta_v` 是快速环自上个周期以来对 `s0` 与 `i` 的积分（用户
    /// 单位），`weight` 是这段时间内 `Σ(slope_dt/Δt)²`。
    pub fn step_once(&mut self, delta_u: V2, delta_v: V2, t: S, weight: S) -> Option<(Report, V2)> {
        let dt = (t - self.t_last).max(0.0);
        self.t_last = t;

        // 1. 遗忘（按降采样时间推进；`tau <= 0` 表示不遗忘）
        let rho = if self.cfg.tau > 0.0 {
            (-dt / self.cfg.tau).exp()
        } else {
            1.0
        };
        self.m.decay(rho);
        self.w2 *= rho * rho;

        // 2. 泄漏积分器（一阶高通）：`u`、`v` 各自以 `T_hp = hp_rel·tau` 向零
        //    泄漏。它替代“每周期平移 36 个矩”：`u + k·v` 收敛到相对物理圆心的
        //    位置，矩里的坐标始终有界。泄漏掉的量不保存——局部帧的原点本身
        //    会收敛到物理圆心，交班只需要相对最新样本的目标圆心。
        let hp = if self.cfg.tau > 0.0 && self.cfg.hp_rel > 0.0 {
            (-dt / (self.cfg.hp_rel * self.cfg.tau)).exp()
        } else {
            1.0
        };
        let inv = 1.0 / self.unit_len;
        let du = delta_u.mul(inv);
        let dv = delta_v.mul(inv);
        self.path += du.add(dv.mul(self.k)).norm();

        self.u = self.u.add(du).mul(hp);
        self.v = self.v.add(dv).mul(hp);

        let q = weight;
        self.m.add(self.u, self.v, q);
        self.w2 += q * q;

        // 3. 恰好一次 Newton；可观性只缩放步长，不硬冻结
        self.steps = self.newton_weighted();

        self.report_pair()
    }

    /// 初始化网格扫描：在 `[k0/scan_rel, k0·scan_rel]` 上取对数网格，选剖面
    /// 损失最小且曲率为正的点，把 `k` 移过去，然后跑几步 Newton。
    ///
    /// 只在窗口足够长（`Σw` 与弧长由调用方把关）时调用。稳态数据上
    /// `P(k) ≡ 0`，扫描找不到显著更优的点，于是保持初始 `k` 不动并返回
    /// `false`——调用方可以稍后重试（等真正的瞬态到来）。
    pub fn initialize(&mut self) -> bool {
        let Some((_, f_cur, _)) = self.fit_hinge(&self.m, self.k) else {
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
        // 候选点的评估：直接在 κ=k_best 处展开（矩保持原始 `U`）。
        let Some((_, _, f0_best)) = self.fit_hinge(&self.m, k_best) else {
            return false;
        };
        // 可辨识性与噪声门限只看**无约束**剖面：半径罚项是先验，不能替数据
        // 证明“这个 k 是数据支持的”。
        let v2 = self.m.g;
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
        let sigma_rel = (f0_best.p.max(0.0) / self.m.w.max(1.0e-300)).sqrt()
            / (2.0 * f0_best.radius.max(1.0e-12));
        if !sigma_rel.is_finite() || sigma_rel > self.cfg.max_sigma_rel {
            return false;
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

    /// 在 `m` 上先做无约束拟合；半径越界时把线性化罚项加进法方程再解一次。
    ///
    /// `kappa` 是求值点（本观测器恒为 `k`）。返回 `(罚项, 带罚项解, 无约束解)`：
    /// 可辨识性 / 噪声门限用无约束解，Newton 步与线搜索用带罚项解。
    fn fit_hinge(
        &self,
        m: &Moments,
        kappa: S,
    ) -> Option<(Option<RadiusPenalty>, Profile, Profile)> {
        let co = m.coef_at(kappa);
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

    /// 任意 `k` 处的 `fit_hinge`：直接在 `κ = k` 展开。
    fn fit_hinge_at(&self, k: S) -> Option<(Option<RadiusPenalty>, Profile, Profile)> {
        self.fit_hinge(&self.m, k)
    }

    /// 固定罚项的剖面损失。线搜索必须与 Newton 步用同一个目标，否则
    /// “`P` 下降”与“带罚项 `P` 下降”会互相打架。
    fn p_at_with_penalty(&self, k: S, pen: Option<RadiusPenalty>) -> Option<S> {
        let co = self.m.coef_at(k);
        fit_with_penalty_coef(&co, pen).map(|f| f.p)
    }

    fn newton(&mut self, iters: usize) -> u32 {
        self.newton_impl(iters, false)
    }

    /// 降采样周期的单次 Newton：`obs` 只做**步长系数**
    /// `w = clamp(|obs|/obs_min,0,1)²`，不设任何基于 `obs` 或 `sigma` 的冻结门限。
    ///
    /// 稳态 `obs→0` 时步长→0（近似慢冻结），瞬态 `obs≥obs_min` 时全步。
    /// 硬门限的 `newton()` 只用于初始化扫描后的抛光（`initialize`）。
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
            let (pen, f, f0) = match self.fit_hinge(&self.m, self.k) {
                Some(x) => x,
                None => break,
            };
            // 满激励时 `P'' = 2Σw(∂r/∂k)² + ...`，而 `|∂r/∂k| <= 2R|V|`，
            // 用 `4R²Σw|V|²` 作尺度得到无量纲可观性指数。这里取存储坐标系
            // （最新样本 `V = 0`）下的 `Σw|V|² = m.g`：它对 `V` 的常数偏置
            // 不变（给所有 `V_n` 加同一个 `c`，`V_n - V_latest` 不变），所以
            // 拟合与门限都与 `V` 偏置无关。门限一律用**无约束**剖面：半径罚项
            // 只会增加曲率，不能拿它冒充“k 被数据辨识出来了”。
            let scale = 4.0 * f0.radius * f0.radius * self.m.g;
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
            let sigma_rel =
                (f0.p.max(0.0) / self.m.w.max(1.0e-300)).sqrt() / (2.0 * f0.radius.max(1.0e-12));
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

    /// 报告 + **局部圆心**：后者是拟合圆心相对**最新样本**的向量（内部单位），
    /// 供快速环做“目标相对圆心”。
    ///
    /// 泄漏积分器下最新样本在局部坐标 `p_local = U + k·V`，不在原点；所以
    /// 返回 `c - p_local`，而不是 `c` 本身。`c` 是局部帧里的圆心（帧原点
    /// 会收敛到物理圆心），不保存绝对位置。
    fn report_pair(&self) -> Option<(Report, V2)> {
        let kappa = self.k;
        let co = self.m.coef_at(kappa);
        let f0 = fit_from_coef(&co)?;
        // 可观性尺度与 `newton_impl` 完全一致：`Σw|V|²` 对 `V` 的常数偏置不变。
        let scale = 4.0 * f0.radius * f0.radius * self.m.g;
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
        // `p_local = U + k·V`：最新样本在局部坐标系中的位置。
        let p_loc = self.u.add(self.v.mul(kappa));
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

        Some((
            Report {
                k: self.k,
                radius: r * l,
                center: c.mul(l),
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
            },
            c.sub(p_loc),
        ))
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
    /// 圆心、半径、`R` 每次拟合向拟合值移动的**基础**比例。
    ///
    /// `0.05` 对应约 20 个拟合周期（200 ms）收敛。`R` 的实际增益还要乘可辨识
    /// 性 `w_obs`（稳态 `obs→0` 时不跟噪声游走）；圆心/半径直接用它——实测按
    /// `|k_fit − k_fast|` 冻结圆心会让启动瞬态更差（R 没收敛时，用拟合的 `k`
    /// 算出的圆心反而把快速环往更准的圆上拉）。
    pub sync_gain: S,
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
    /// 快速环的磁链向量（用户单位 A，相对拟合圆心；圆心即原点）。
    ///
    /// 它就是快速环的 `c + ∫s0 + R·∫i`，理想情况下落在以原点为圆心、
    /// 半径 `radius` 的圆上，是画"磁链圆"轨迹用的。
    pub flux: V2,
    pub obs: S,
    pub sigma_rel: S,
    /// `k` 的标准误（诊断/调参用）。
    pub sigma_k: S,
    /// 剖面一阶/二阶导（诊断）。
    pub p1: S,
    pub p2: S,
    pub steps: u32,
    pub weight: S,
    pub data_arc: S,
    /// 拟合圆心（用户单位，物理绝对圆心；`k` 变化时有 `Σ(k_i − k)·v_i` 残差；
    /// 诊断用）。
    pub center: V2,
    /// 拟合圆心（内部单位，相对最新样本；只取它的方向，长度会被换成半径）。
    pub center_local: V2,
    /// 交班**实际使用的圆心**，画在磁链图坐标系里（用户单位）：
    /// `C_used − C_fast`，即以"快速环圆心"为原点时该圆心的位置。
    ///
    /// 它由"最新样本 + 拟合半径"推出（方向取 `center_local`、长度取 ρ），
    /// 每次拟合更新一次；它与原点的偏差就是快速环圆心和这个圆心的差距。
    pub center_used: V2,
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
            // 直接观测器不需要随机游走先验：步长只由可观性加权。拟合侧是
            // 泄漏积分器（`hp_rel·tau`），帧原点缓慢跟踪绝对位置，矩里的
            // 坐标始终有界；`κ = k` 处的三/四次展开没有相消问题。
            fit: FitConfig {
                k_prior_rel: 0.0,
                ..FitConfig::default()
            },
            fit_period: 0.01,
            speed_tau: 0.02,
            sync_gain: 0.05,
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
            flux: V2::ZERO,
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
            center_used: V2::ZERO,
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
        // 已知启动圆心：泄漏积分器从 `u = -圆心/unit_len` 起步，帧原点一开始
        // 就是圆心，避免 `u`、`v` 从 0 慢慢爬到稳态而在第一个窗口里拖出螺旋。
        if let Some(tr) = self.tracker.as_mut() {
            tr.seed_frame(c0);
        }
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
        // t=0 时快速环与拟合用同一个圆心假设，图上重合于原点。
        self.center_used = V2::ZERO;
        self.theta_meas = self.theta;
        self.omega = omega;
        self.flux_mag = radius;
        // t=0 时磁链点在原点、圆心在 `radius·n(θ)`，相对圆心就是 `-radius·n(θ)`。
        self.flux = V2::new(-self.theta.cos(), -self.theta.sin()).mul(radius);
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
                if let Some((rep, c_local)) = tr.step_once(a.mul(l), b.mul(l), self.total_time, q) {
                    let k_fit = tr.k();
                    let rad_fit = rep.radius / l;
                    // 拟合圆心只用来定方向：从最新采样点出发、沿拟合圆心的方向
                    // 走一个拟合半径 ρ，得到"最新点严格落在半径 ρ 圆上"的圆心。
                    // 直接用 `c_local` 会把它的长度误差（最新点不刚好在圆上、
                    // 或 k 没收敛时圆本身有偏）带进快速环。
                    let c_use = if c_local.norm2() > 0.0 {
                        c_local.mul(rad_fit / c_local.norm())
                    } else {
                        c_local
                    };
                    // `R` 的交接乘可辨识性：稳态 `obs→0` 时不把拟合的噪声游走带进快速环。
                    let w_obs = (rep.obs / self.fit.obs_min.max(1.0e-12)).clamp(0.0, 1.0);
                    let w_k = self.sync_gain * w_obs;
                    let mut center_used = self.center_used;
                    if let Ok(mut f) = self.fast.lock() {
                        // 交班前的 `f.c` = 最新样本相对快速环圆心；加上由半径推
                        // 出的圆心偏移，就是它在这个磁链图上的位置（原点 = 快速
                        // 环圆心）。它每次拟合更新一次。
                        center_used = f.c.add(c_use).mul(l);
                        // 圆心/半径不乘 `obs`：它们不是"可辨识性"问题。注意它们也
                        // 不能按 `|k_fit − k_fast|` 冻结——R 还没收敛时，用拟合的
                        // k 算出的方向恰好是把快速环往更准的圆上拉，冻结反而更偏。
                        let tgt = c_use.mul(-1.0);
                        f.c = f.c.add(tgt.sub(f.c).mul(self.sync_gain));
                        f.k += (k_fit - f.k) * w_k;
                        f.rad += (rad_fit - f.rad) * self.sync_gain;
                    }
                    self.center_used = center_used;
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
        let (theta, omega, mag, rel, k_now, rad_now) = {
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
            (theta, f.omega_lp, rel.norm(), rel, f.k, f.rad)
        };

        self.theta = theta;
        self.theta_meas = theta;
        self.omega = omega;
        self.r = k_now;
        self.radius = rad_now * self.unit_len;
        self.flux_mag = mag * self.unit_len;
        self.flux = rel.mul(self.unit_len);
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
    fn profile_derivatives_match_finite_difference() {
        let mut a = Moments::default();
        for k in 0..60u32 {
            let t = k as S * 0.27;
            let u = V2::new(1.3 * t.cos(), 1.3 * t.sin());
            let v = V2::new(0.4 * t.cos() + 0.2 * t.sin(), 0.4 * t.sin() - 0.2 * t.cos());
            a.add(u, v, 1.0);
        }
        let p_at = |k: S| fit_from_coef(&a.coef_at(k)).unwrap().p;
        for &k0 in &[0.2, 0.5, 0.8] {
            let f = fit_from_coef(&a.coef_at(k0)).unwrap();
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
        let stream = synth_stream(
            r,
            radius,
            100.0,
            |t| 3.0 + 1.5 * (TAU * 0.7 * t).cos(),
            3.0,
            dt,
        );
        let mut tr = tracker_for(r, radius, 1.25 * r);
        let mut scan_done = false;
        for (k, (_, du, dv)) in stream.iter().enumerate() {
            let t = k as S * dt;
            tr.update(*du, *dv, t, 1.0);
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
            tr.update(*du, *dv, t, 1.0);
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
            tight.update(*du, *dv, i as S * 0.01, 1.0);
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
            free.update(*du, *dv, i as S * 0.01, 1.0);
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
        let c = rep.center.mul(1.0 / l);
        let r = rep.radius / l;
        let theta = [-2.0 * c.x, -2.0 * c.y, c.norm2() - r * r];
        let co = tight.m.coef_at(tight.k());
        let unpen = loss_from_coef(&co, &theta);
        assert!(
            (unpen - rep.loss * w).abs() <= 1.0e-9 * (1.0 + unpen.abs()),
            "loss={} 不是约束后 θ 的无罚项残差 {}",
            rep.loss,
            unpen / w
        );
        let f0 = fit_from_coef(&co).unwrap();
        let pen = tight.radius_penalty(&f0, w).expect("penalty");
        let fpen = fit_with_penalty_coef(&co, Some(pen)).unwrap();
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
    /// 必须完全不变。拟合与可观性门限都建立在这一点上：`V` 不需要单独保存
    /// 偏置（泄漏积分器只会让这个偏置慢慢衰减，仍然只是规范）。
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
        for &k in &[0.0, 0.37, 1.1] {
            let (fa, fb) = (
                fit_from_coef(&a.coef_at(k)).unwrap(),
                fit_from_coef(&b.coef_at(k)).unwrap(),
            );
            assert!(rel(fa.p, fb.p) < 1.0e-12, "P 变了 (k={k})");
            assert!(rel(fa.p1, fb.p1) < 1.0e-11, "P' 变了 (k={k})");
            assert!(rel(fa.p2, fb.p2) < 1.0e-10, "P'' 变了 (k={k})");
        }
    }

    /// `V = ∫i dt` 的常数偏移是规范：注入一个偏置后 `k` 仍须从 +25% 初值恢复到
    /// 真值。`V` 的偏置只影响每个 `κ` 的平移，`P`、`P'`、`P''` 与 `k` 都不受影响。
    #[test]
    fn v_offset_does_not_break_k() {
        let k_true = 0.7;
        let r_true = 1.35;
        let mut stream = circle_stream(k_true, r_true, 300, 0xc0ffee);
        stream[0].1 = stream[0].1.add(V2::new(2.0, -1.0));

        let mut tr = Tracker::new(config_for_range(0.3 * r_true, 3.0 * r_true), 1.25 * k_true);
        for (i, (du, dv)) in stream.iter().enumerate() {
            tr.update(*du, *dv, i as S * 0.01, 1.0);
        }
        let rep = tr.report_now().expect("report");
        eprintln!(
            "v-offset: k={:.6} R={:.4} obs={:.2e}",
            tr.k(),
            rep.radius,
            tr.obs()
        );
        assert!((tr.k() - k_true).abs() < 1.0e-3, "k={}", tr.k());
        assert!((rep.radius - r_true).abs() < 1.0e-3, "R={}", rep.radius);
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
            "direct: tail={worst_tail:.4} jump={worst_jump:.4} r={:.3} r_fit={:.3} radius={:.4} obs={:.2e} center_used={:?}",
            ob.r, ob.r_fit, ob.radius, ob.obs, ob.center_used
        );
        assert!(worst_tail < 0.05, "tail error {worst_tail} rad");
        assert!(worst_jump < 0.15, "angle jump {worst_jump} rad");
        // 磁链向量（相对拟合圆心）应当落在半径圆上，供 GUI 的"磁链圆"图使用。
        assert!(
            (ob.flux.norm() - ob.radius).abs() < 0.1 * ob.radius,
            "flux {:?} 与半径 {} 不一致",
            ob.flux,
            ob.radius
        );
        // 图上"拟合圆心"应当收敛到原点附近（快速环圆心 ≈ 拟合圆心）。
        assert!(
            ob.center_used.norm() < 0.2 * ob.radius,
            "center_used {:?} 偏离原点（radius {}）",
            ob.center_used,
            ob.radius
        );
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

    /// 泄漏积分器下最新样本**不在**局部原点：`p_local` 是它在帧里的位置，
    /// `step_once` 返回的局部圆心必须是 `c - p_local`（相对最新样本）。
    #[test]
    fn step_once_returns_sample_relative_center() {
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
        let (rep, c_local) = last.expect("report");
        let p = tr.p_local();
        eprintln!(
            "center: p_local={p:?} c_local={c_local:?} radius={:.4} center={:?}",
            rep.radius, rep.center
        );
        // 最新样本在局部帧里确实是非零的（不再搬原点）。
        assert!(p.norm() > 0.1, "p_local 不应被清零: {p:?}");
        // 返回的是“圆心 - 最新样本”，不是局部帧里的圆心本身。
        let expect = rep.center.mul(1.0 / tr.unit_len()).sub(p);
        assert!(
            c_local.sub(expect).norm() < 1.0e-12,
            "c_local 不是相对最新样本的圆心"
        );
        assert!(
            (c_local.norm() - rep.radius / tr.unit_len()).abs() < 1.0e-3 * rep.radius,
            "c_local 长度应与半径一致：|c_local|={}",
            c_local.norm()
        );
        assert!((rep.radius - radius).abs() < 0.01);
        // 样本落在拟合圆上：|p - c| = radius。
        assert!(
            (rep.p_rel.norm() - rep.radius).abs() < 1.0e-3 * rep.radius,
            "|p_rel|={} 与半径 {} 不一致",
            rep.p_rel.norm(),
            rep.radius
        );
        // 启动 0.2 s 远小于 T_hp=200 s，局部帧原点还几乎没漂，所以局部圆心
        // 仍接近 `radius·n(0)`；再久就会衰减到 0（这是泄漏积分器的语义）。
        let c_ref = V2::new(radius, 0.0);
        assert!(
            rep.center.sub(c_ref).norm() < 0.02,
            "center {:?} 偏离启动圆心 {:?}",
            rep.center,
            c_ref
        );
    }

    /// 泄漏积分器把直流漂移限制在 `rate·T_hp`（等效一阶高通的稳态）。
    #[test]
    fn leaky_integrator_bounds_dc_offset() {
        let mut cfg = FitConfig::default();
        cfg.tau = 0.01;
        cfg.hp_rel = 20.0;
        cfg.min_weight = 1.0;
        let mut tr = Tracker::new(cfg, 1.0);
        let dt = 1.0e-4;
        let rate = 1.0; // 用户单位/秒的直流漂移
        let t_end = 2.0; // 10 个 T_hp
        let n = (t_end / dt) as usize;
        for i in 1..=n {
            tr.step_once(V2::new(rate * dt, 0.0), V2::ZERO, i as S * dt, 1.0);
        }
        let l = tr.unit_len();
        let t_hp = cfg.hp_rel * cfg.tau;
        let u_expect = rate * t_hp / l;
        let u = tr.u.norm();
        eprintln!("leak: u={u:.4} (expect {u_expect:.4})");
        assert!(
            (u - u_expect).abs() < 0.02 * u_expect,
            "泄漏积分器稳态 {u} 应为 {u_expect}"
        );
    }

    /// `update()` 路径（现在就是 `step_once()`）：泄漏积分器让 `V` 有界，
    /// 不再要求最新样本的 `V` 归零；拟合仍要给出正确的半径。
    #[test]
    fn update_keeps_leaky_v_bounded() {
        let (r, radius, dt) = (384.0, 4.04, 5.0e-5);
        let stream = synth_stream_profile(r, radius, |_| 20.0, |_| 3.0, 0.4, dt);
        let mut cfg = FitConfig::default();
        cfg.r_min = 0.3 * radius;
        cfg.r_max = 3.0 * radius;
        cfg.k_min = 0.1 * r;
        cfg.k_max = 10.0 * r;
        cfg.tau = 10.0;
        let mut tr = Tracker::new(cfg, r);
        let (mut a, mut b) = (V2::ZERO, V2::ZERO);
        let (mut t, mut n, mut last) = (0.0, 0, None);
        for (_th, du, dv) in &stream {
            a = a.add(*du);
            b = b.add(*dv);
            t += dt;
            n += 1;
            if n == 400 {
                last = tr.update(a, b, t, 400.0);
                a = V2::ZERO;
                b = V2::ZERO;
                n = 0;
            }
        }
        let rep = last.expect("report");
        eprintln!(
            "leaky V: k={:.4} radius={:.4} |V|={:.4} g={:.3e}",
            tr.k(),
            rep.radius,
            tr.v.norm(),
            tr.m.g
        );
        assert!(tr.v.norm() < 0.1, "V 应当有界，实际 {}", tr.v.norm());
        assert!((rep.radius - radius).abs() < 0.01, "radius {}", rep.radius);
    }
}
