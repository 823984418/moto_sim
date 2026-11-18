//! 基于**指数遗忘 Kåsa 流式圆拟合**的磁链 / 电阻观测器。
//!
//! 本模块是重新写的：拟合部分不再是 `flux_circle_observer.rs` 的“攒窗口 → 批量
//! 几何拟合”，而是把窗口精确压缩成 36 个与 `k` 无关的加权矩，每个采样只做
//! 一次 O(1) 的变量投影 Newton（算法与推导见 `2026-11-18-kasa-streaming`）。
//!
//! # 1. 与磁链圆的对应
//!
//! 零矢量期间（端电压为零）的电流斜率记为 `s0 = di/dt|v=0`，令
//!
//! ```text
//!     A(t) = ∫ s0 dt ,   B(t) = ∫ i dt ,   p(k) = A + k B ,   k = R = Rs / L
//! ```
//!
//! 由 `s0 = -(Rs i + ω × λ) / L` 可得
//!
//! ```text
//!     p(R) = A + R B = -(λ(t) - λ(0)) / L
//! ```
//!
//! 即 `p(R)` 落在圆心 `c = λ(0)/L`、半径 `ρ = ψf/L` 的圆上。这正是流式 Kåsa
//! 问题 `p_n(k) = U_n + k V_n`：本模块取 `U = A`、`V = B`，`k` 就是待辨识的
//! `Rs/L`。圆上相位就是电角度：`θ = atan2(-(p - c).y, -(p - c).x)`。
//!
//! # 2. 为什么这比“窗口批量拟合”好
//!
//! * **O(1) 内存**：36 个矩精确概括整个（指数遗忘的）窗口，不需要保存样本，
//!   也就不存在“窗口长度 / 降采样 / 拟合触发 / 重锚点”这些离散逻辑；
//! * **O(1) 时间**：每拍 2~3 次 3×3 Cholesky；圆心和半径**每拍**都与当前 `k`
//!   一致（它等价于“固定当前 R 的几何拟合”），不会出现 `flux_circle_observer`
//!   里“R 冻结但几何还是自由拟合那一套”的不一致；
//! * **数值稳定**：历史矩始终保存在“当前 `k` 的参考系”里（剪切 `U_s = U + k_ref V`），
//!   所以求值永远在 `κ = 0` 处进行，`A` 与 `R B` 之间那种 1e2 量级的相消被移除了；
//!   浮动原点再把点云质心移到零，使 `|p| ~ ρ`；
//! * **可观性显式化**：`P''/(4 R² Σw|V|²)` 给出一个无量纲的 `k` 可观性指数。
//!   稳态（iq、ω 不变）时 `P(k) ≡ 0`，该指数趋于零，`k` 自动冻结；只有电流/转速
//!   瞬态才允许更新 `R`。
//!
//! # 3. 与 `flux_circle_observer` 相同的工程约定
//!
//! * `set_initial(theta, omega, r, radius)` 写入 ALIGN 后的已知角度与铭牌参数；
//! * 角度经二阶 PLL 输出，`pll_wn_rel/min/min_run/narrow_time/zeta` 含义一致；
//! * 零矢量不可用（`current_slope_dt <= 0`）时本拍不更新，保持输出并置
//!   `valid = false`；
//! * 拟合权重 `w ∝ (slope_dt / Δt)²`，与零矢量时长成正比。
//!
//! 与 `flux_circle_observer` 的唯一“接口差异”是：拟合并未估计圆心漂移 `v`。
//! 指数遗忘本身就是一个因果低通，圆心（即 `λ(0)/L`）的慢漂会被窗口自然跟上，
//! 不需要额外的仿射参数；少一个参数也让 `k` 的剖面更干净。

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, clarke};

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
fn outer(a: V2, b: V2) -> [[S; 2]; 2] {
    [[a.x * b.x, a.x * b.y], [a.y * b.x, a.y * b.y]]
}

#[inline]
fn mv(m: &[[S; 2]; 2], v: V2) -> V2 {
    V2::new(
        m[0][0] * v.x + m[0][1] * v.y,
        m[1][0] * v.x + m[1][1] * v.y,
    )
}

#[inline]
fn mtr(m: &[[S; 2]; 2]) -> [[S; 2]; 2] {
    [[m[0][0], m[1][0]], [m[0][1], m[1][1]]]
}

#[inline]
fn madd(a: &[[S; 2]; 2], b: &[[S; 2]; 2]) -> [[S; 2]; 2] {
    [
        [a[0][0] + b[0][0], a[0][1] + b[0][1]],
        [a[1][0] + b[1][0], a[1][1] + b[1][1]],
    ]
}

#[inline]
fn msub(a: &[[S; 2]; 2], b: &[[S; 2]; 2]) -> [[S; 2]; 2] {
    [
        [a[0][0] - b[0][0], a[0][1] - b[0][1]],
        [a[1][0] - b[1][0], a[1][1] - b[1][1]],
    ]
}

#[inline]
fn msc(a: &[[S; 2]; 2], s: S) -> [[S; 2]; 2] {
    [[a[0][0] * s, a[0][1] * s], [a[1][0] * s, a[1][1] * s]]
}

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
/// 每个系数都是下面这些矩的固定线性组合。注意 `uv` 存的是完整的非对称
/// `Σ w U Vᵀ`：平移变换里 `(ΣUVᵀ)s` 与 `(ΣVUᵀ)s` 不同。
#[derive(Clone, Debug, Default)]
struct Moments {
    w: S,
    u: V2,
    v: V2,
    uu: [[S; 2]; 2],
    uv: [[S; 2]; 2],
    vv: [[S; 2]; 2],
    a0: S,
    au: V2,
    av: V2,
    b0: S,
    bu: V2,
    bv: V2,
    g0: S,
    gu: V2,
    gv: V2,
    /// `Σ w * (α², αβ, β², αγ, βγ, γ²)`
    aa: S,
    ab: S,
    bb: S,
    ag: S,
    bg: S,
    gg: S,
}

impl Moments {
    #[inline]
    fn decay(&mut self, rho: S) {
        self.w *= rho;
        self.u = self.u.mul(rho);
        self.v = self.v.mul(rho);
        self.uu = msc(&self.uu, rho);
        self.uv = msc(&self.uv, rho);
        self.vv = msc(&self.vv, rho);
        self.a0 *= rho;
        self.au = self.au.mul(rho);
        self.av = self.av.mul(rho);
        self.b0 *= rho;
        self.bu = self.bu.mul(rho);
        self.bv = self.bv.mul(rho);
        self.g0 *= rho;
        self.gu = self.gu.mul(rho);
        self.gv = self.gv.mul(rho);
        self.aa *= rho;
        self.ab *= rho;
        self.bb *= rho;
        self.ag *= rho;
        self.bg *= rho;
        self.gg *= rho;
    }

    /// 加入一个权重为 `q` 的样本。
    #[inline]
    fn add(&mut self, u: V2, v: V2, q: S) {
        let alpha = u.norm2();
        let beta = u.dot(v);
        let gamma = v.norm2();

        self.w += q;
        self.u = self.u.add(u.mul(q));
        self.v = self.v.add(v.mul(q));
        self.uu = madd(&self.uu, &msc(&outer(u, u), q));
        self.uv = madd(&self.uv, &msc(&outer(u, v), q));
        self.vv = madd(&self.vv, &msc(&outer(v, v), q));

        self.a0 += q * alpha;
        self.au = self.au.add(u.mul(q * alpha));
        self.av = self.av.add(v.mul(q * alpha));

        self.b0 += q * beta;
        self.bu = self.bu.add(u.mul(q * beta));
        self.bv = self.bv.add(v.mul(q * beta));

        self.g0 += q * gamma;
        self.gu = self.gu.add(u.mul(q * gamma));
        self.gv = self.gv.add(v.mul(q * gamma));

        self.aa += q * alpha * alpha;
        self.ab += q * alpha * beta;
        self.bb += q * beta * beta;
        self.ag += q * alpha * gamma;
        self.bg += q * beta * gamma;
        self.gg += q * gamma * gamma;
    }

    /// 把整个历史重新投影到 `k + lam`：`U -> U + lam V`。
    ///
    /// 这是精确变换：对任意 `κ`，`p(κ)` 不变。它等价于“用新的 `k` 把每个历史
    /// 样本重新算一遍”，所以 `P(k)` 不变，但之后求值都落在 `κ = 0`。
    fn shear(&mut self, lam: S) {
        if lam == 0.0 {
            return;
        }
        let l2 = lam * lam;
        let l3 = l2 * lam;
        let l4 = l2 * l2;

        let (u, v, uu, uv, vv) = (self.u, self.v, self.uu, self.uv, self.vv);
        let (a0, au, av) = (self.a0, self.au, self.av);
        let (b0, bu, bv) = (self.b0, self.bu, self.bv);
        let (g0, gu, gv) = (self.g0, self.gu, self.gv);
        let (aa, ab, bb, ag, bg, gg) = (self.aa, self.ab, self.bb, self.ag, self.bg, self.gg);

        self.u = u.add(v.mul(lam));
        self.uu = madd(
            &madd(&uu, &msc(&madd(&uv, &mtr(&uv)), lam)),
            &msc(&vv, l2),
        );
        self.uv = madd(&uv, &msc(&vv, lam));

        self.a0 = a0 + 2.0 * lam * b0 + l2 * g0;
        self.au = au
            .add(av.mul(lam))
            .add(bu.mul(2.0 * lam))
            .add(bv.mul(2.0 * l2))
            .add(gu.mul(l2))
            .add(gv.mul(l3));
        self.av = av.add(bv.mul(2.0 * lam)).add(gv.mul(l2));

        self.b0 = b0 + lam * g0;
        self.bu = bu.add(bv.mul(lam)).add(gu.mul(lam)).add(gv.mul(l2));
        self.bv = bv.add(gv.mul(lam));

        // γ 不变，但 U 变了，所以 Σ w γ U 必须跟着动。
        self.gu = gu.add(gv.mul(lam));

        self.aa = aa + 4.0 * lam * ab + 2.0 * l2 * ag + 4.0 * l2 * bb + 4.0 * l3 * bg + l4 * gg;
        self.ab = ab + lam * ag + 2.0 * lam * bb + 3.0 * l2 * bg + l3 * gg;
        self.bb = bb + 2.0 * lam * bg + l2 * gg;
        self.ag = ag + 2.0 * lam * bg + l2 * gg;
        self.bg = bg + lam * gg;
        // w, v, vv, g0, gv, gg 在剪切下不变
    }

    /// 坐标系平移 `U -> U - s`（`V` 不动）。精确；Kåsa 评分不变，圆心读数加回 `s`。
    fn translate_u(&mut self, s: V2) {
        if s.norm2() == 0.0 {
            return;
        }
        let w = self.w;
        let u = self.u;
        let v = self.v;
        let uu = self.uu;
        let uv = self.uv;
        let vv = self.vv;
        let (a0, au, av) = (self.a0, self.au, self.av);
        let (b0, bu, bv) = (self.b0, self.bu, self.bv);
        let (g0, gu, gv) = (self.g0, self.gu, self.gv);
        let (aa, ab, bb, ag, bg) = (self.aa, self.ab, self.bb, self.ag, self.bg);

        let s2 = s.norm2();
        let su = s.dot(u);
        let sv = s.dot(v);
        let cs = mv(&uv, s); // Σ U (V·s)
        let cts = mv(&mtr(&uv), s); // Σ V (U·s)
        let uus = mv(&uu, s);
        let vvs = mv(&vv, s);

        self.u = u.sub(s.mul(w));
        self.uu = madd(
            &msub(&msub(&uu, &outer(u, s)), &outer(s, u)),
            &msc(&outer(s, s), w),
        );
        self.uv = msub(&uv, &outer(s, v));

        self.a0 = a0 - 2.0 * su + s2 * w;
        self.au = au
            .sub(s.mul(a0))
            .sub(uus.mul(2.0))
            .add(u.mul(s2))
            .add(s.mul(2.0 * su))
            .sub(s.mul(s2 * w));
        self.av = av.sub(cts.mul(2.0)).add(v.mul(s2));

        self.b0 = b0 - sv;
        self.bu = bu.sub(s.mul(b0)).sub(cs).add(s.mul(sv));
        self.bv = bv.sub(vvs);

        self.gu = gu.sub(s.mul(g0));

        self.aa = aa - 4.0 * s.dot(au) + 2.0 * s2 * a0 + 4.0 * s.dot(uus) - 4.0 * s2 * su
            + s2 * s2 * w;
        self.ab = ab - s.dot(av) - 2.0 * s.dot(bu) + s2 * b0 + 2.0 * s.dot(cs) - s2 * sv;
        self.bb = bb - 2.0 * s.dot(bv) + s.dot(vvs);
        self.ag = ag - 2.0 * s.dot(gu) + s2 * g0;
        self.bg = bg - s.dot(gv);
        // w, v, vv, g0, gv, gg 在平移下不变
    }

    #[inline]
    fn a_mat(&self) -> M3 {
        [
            [self.uu[0][0], self.uu[0][1], self.u.x],
            [self.uu[1][0], self.uu[1][1], self.u.y],
            [self.u.x, self.u.y, self.w],
        ]
    }

    #[inline]
    fn b_vec(&self) -> [S; 3] {
        [self.au.x, self.au.y, self.a0]
    }

    /// 当前参考 `k`（`κ = 0`）处的剖面 Taylor 系数。
    fn coef(&self) -> Coef {
        let a1 = [
            [
                self.uv[0][0] * 2.0,
                self.uv[0][1] + self.uv[1][0],
                self.v.x,
            ],
            [
                self.uv[1][0] + self.uv[0][1],
                self.uv[1][1] * 2.0,
                self.v.y,
            ],
            [self.v.x, self.v.y, 0.0],
        ];
        let a2 = [
            [self.vv[0][0], self.vv[0][1], 0.0],
            [self.vv[1][0], self.vv[1][1], 0.0],
            [0.0, 0.0, 0.0],
        ];
        Coef {
            a0: self.a_mat(),
            a1,
            a2,
            b0: self.b_vec(),
            b1: [
                self.av.x + 2.0 * self.bu.x,
                self.av.y + 2.0 * self.bu.y,
                2.0 * self.b0,
            ],
            b2: [
                self.gu.x + 2.0 * self.bv.x,
                self.gu.y + 2.0 * self.bv.y,
                self.g0,
            ],
            c0: self.aa,
            c1: 4.0 * self.ab,
            c2: 4.0 * self.bb + 2.0 * self.ag,
        }
    }

    /// 在**当前**参考 `k`（即 `κ = 0`）处做变量投影：
    /// `P = min_θ J`、`θ* = -A⁻¹ b`，以及 `P` 的一、二阶导数。
    fn fit(&self) -> Option<Profile> {
        if !(self.w > 0.0) {
            return None;
        }
        let co = self.coef();
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
}

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
    /// 浮动原点阈值（内部单位）。
    pub recenter_eps: S,
    /// `P''` 的绝对下限。
    pub p2_eps: S,
    /// 半径伪观测权重（相对 `Σw`）。
    pub r_kappa: S,
    /// 允许更新 `k` 所需的最小有效样本数（`Σw`）。
    pub min_weight: S,
    /// 圆模型可信度门限：径向残差 RMS / 半径 超过它时冻结 `k`。
    ///
    /// 高噪声下圆模型本身不成立，剖面曲率会被噪声抬高，`k` 的更新不可信
    /// （与 `flux_circle_observer` 的 `max_sigma_rel` 同一作用）。
    pub max_sigma_rel: S,
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
            recenter_eps: 0.25,
            p2_eps: 1.0e-18,
            r_kappa: 50.0,
            min_weight: 8.0,
            max_sigma_rel: 0.02,
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
/// 存储的 `(U, V)` 始终是“当前参考 `k`”下的量：`U_s = U + k_ref V`。因此
/// 每拍求值都在 `κ = 0` 处进行，不需要组装 `Σ kʲ b_j`。
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
    /// `set_initial` 给的初始 `k`，网格扫描围绕它展开。
    k_initial: S,
    /// 当前样本在局部参考系中的位置 `U_s`。
    u: Vacc,
    v: Vacc,
    /// 局部原点在绝对（用户）坐标系中的位置。
    off: V2,
    t_last: S,
    started: bool,
    /// 已经累积的路径长度 `Σ|Δp|`（内部单位）。它与 `k` 无关，用作“数据
    /// 实际覆盖了多长弧”的度量；拿 PLL 的 `|ω|dt` 会在 `k` 错时严重低估。
    path: S,
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
            u: Vacc::default(),
            v: Vacc::default(),
            off: V2::ZERO,
            t_last: 0.0,
            started: false,
            path: 0.0,
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

        // 2. 积分增量（内部单位）。存储的是 U_s = U + k V，所以增量是
        //    ΔU + k ΔV，不能只加 ΔU。
        let inv = 1.0 / self.unit_len;
        let du_s = delta_u.mul(inv).add(delta_v.mul(inv * self.k));
        self.u.add(du_s);
        self.v.add(delta_v.mul(inv));
        self.path += du_s.norm();

        // 3. 累积矩
        let (ui, vi) = (self.u.val(), self.v.val());
        let q = if weight.is_finite() && weight > 0.0 {
            weight
        } else {
            1.0
        };
        self.m.add(ui, vi, q);

        // 4. 浮动原点
        self.recenter();

        // 5. 变量投影 Newton（由调用方用 newton_iters 控制是否允许更新 k）
        self.steps = if newton_iters > 0 {
            self.newton(newton_iters)
        } else {
            0
        };
        self.recenter();

        self.report()
    }

    /// 立刻重新求一次报告（不喂新样本）。
    pub fn report_now(&self) -> Option<Report> {
        self.report()
    }

    /// 初始化网格扫描：在 `[k0/scan_rel, k0·scan_rel]` 上取对数网格，选剖面
    /// 损失最小且曲率为正的点，把矩剪切过去，然后跑几步 Newton。
    ///
    /// 只在窗口足够长（`Σw` 与弧长由调用方把关）时调用。稳态数据上
    /// `P(k) ≡ 0`，扫描找不到显著更优的点，于是保持初始 `k` 不动并返回
    /// `false`——调用方可以稍后重试（等真正的瞬态到来）。
    pub fn initialize(&mut self) -> bool {
        let f_cur = match self.m.fit() {
            Some(f) => f,
            None => return false,
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
            let mut m = self.m.clone();
            m.shear(k - self.k);
            if let Some(fit) = m.fit() {
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
        let mut m = self.m.clone();
        m.shear(k_best - self.k);
        let fit = match m.fit() {
            Some(f) => f,
            None => return false,
        };
        let scale = 4.0 * fit.radius * fit.radius * m.g0;
        let obs = if scale > 0.0 && scale.is_finite() {
            fit.p2 / scale
        } else {
            0.0
        };
        if !obs.is_finite() || obs < self.cfg.scan_obs_min || !(m.g0 > 0.0) {
            return false;
        }
        // 圆模型可信度：高噪声下“更优的盆地”只是噪声，不能把 k 带跑。
        let sigma_rel = (fit.p.max(0.0) / m.w.max(1.0e-300)).sqrt()
            / (2.0 * fit.radius.max(1.0e-12));
        if !sigma_rel.is_finite() || sigma_rel > self.cfg.max_sigma_rel {
            return false;
        }

        let dk = k_best - self.k;
        self.m.shear(dk);
        self.u.add(self.v.val().mul(dk));
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
        let s = self.m.u.mul(1.0 / w);
        if !s.finite() || s.norm() <= self.cfg.recenter_eps {
            return;
        }
        self.m.translate_u(s);
        self.u.shift(s);
        self.off = self.off.add(s);
    }

    /// 任意 `k` 处的剖面损失（拷贝矩 + 剪切 + 求值）。
    fn p_at(&self, k: S) -> Option<S> {
        let mut m = self.m.clone();
        m.shear(k - self.k);
        m.fit().map(|f| f.p)
    }

    fn newton(&mut self, iters: usize) -> u32 {
        // 3 个有效样本必共圆，`k` 没有信息。
        if self.m.w < self.cfg.min_weight {
            return 0;
        }
        let k_start = self.k;
        let mut taken = 0;
        for _ in 0..iters {
            let f = match self.m.fit() {
                Some(f) => f,
                None => break,
            };
            // 满激励时 `P'' = 2Σw(∂r/∂k)² + ...`，而 `|∂r/∂k| <= 2R|V|`，
            // 用 `4R²Σw|V|²` 作尺度得到无量纲可观性指数。
            let scale = 4.0 * f.radius * f.radius * self.m.g0;
            self.obs = if scale > 0.0 && scale.is_finite() {
                f.p2 / scale
            } else {
                0.0
            };
            if !self.obs.is_finite() || self.obs < self.cfg.obs_min {
                break;
            }
            // 高噪声下圆模型不成立：冻结 k，避免把噪声拟合进 R。
            let sigma_rel = (f.p.max(0.0) / self.m.w.max(1.0e-300)).sqrt()
                / (2.0 * f.radius.max(1.0e-12));
            if !sigma_rel.is_finite() || sigma_rel > self.cfg.max_sigma_rel {
                break;
            }

            // 随机游走先验：等价于给法方程加阻尼。以本拍起点为参考，
            // 激励小时步长自动收缩。
            let prior = self.cfg.k_prior_rel.max(0.0) * scale;
            let denom = f.p2 + 2.0 * prior;
            if !denom.is_finite() || denom <= self.cfg.p2_eps {
                break;
            }
            let mut step = -(f.p1 + 2.0 * prior * (self.k - k_start)) / denom;
            if !step.is_finite() {
                break;
            }
            let lim = self.cfg.k_step_rel * (1.0 + self.k.abs());
            step = step.clamp(-lim, lim);
            if step.abs() <= 1.0e-14 * (1.0 + self.k.abs()) {
                break;
            }

            // 回溯线搜索：只接受让真实 P 下降的步长。
            let p_cur = f.p;
            let mut s = step;
            let mut ok = false;
            for _ in 0..10 {
                let k_try = (self.k + s).clamp(self.cfg.k_min, self.cfg.k_max);
                if k_try == self.k {
                    break;
                }
                match self.p_at(k_try) {
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
            // 精确地把历史重新投影到新的 k。
            self.m.shear(dk);
            self.u.add(self.v.val().mul(dk));
            self.k = k_new;
            taken += 1;
            if dk.abs() < 1e-14 * (1.0 + self.k.abs()) {
                break;
            }
        }
        taken
    }

    /// 半径软约束：把 `R² = |a|²/4 - d` 拉回 `[r_min², r_max²]`。
    fn apply_radius(&self, theta: [S; 3]) -> ([S; 3], bool) {
        let w = self.m.w;
        if !(w > 0.0) {
            return (theta, false);
        }
        let r2 = 0.25 * (theta[0] * theta[0] + theta[1] * theta[1]) - theta[2];
        let r2min = self.rmin_i * self.rmin_i;
        let r2max = self.rmax_i * self.rmax_i;
        let target = if r2 < r2min {
            r2min
        } else if r2 > r2max {
            r2max
        } else {
            return (theta, false);
        };
        let g = [0.5 * theta[0], 0.5 * theta[1], -1.0];
        let z = target - r2 + dot3(&g, &theta);
        let kappa = self.cfg.r_kappa * w;

        let mut a = self.m.a_mat();
        let mut b = self.m.b_vec();
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += kappa * g[i] * g[j];
            }
            b[i] -= kappa * z * g[i];
        }
        match chol3(&a) {
            Some(ch) => {
                let u = chol_solve(&ch, &b);
                ([-u[0], -u[1], -u[2]], true)
            }
            None => (theta, true),
        }
    }

    fn report(&self) -> Option<Report> {
        let f = self.m.fit()?;
        let scale = 4.0 * f.radius * f.radius * self.m.g0;
        let obs = if scale > 0.0 && scale.is_finite() {
            f.p2 / scale
        } else {
            0.0
        };
        let (theta, clamped) = self.apply_radius(f.theta);
        let c = V2::new(-0.5 * theta[0], -0.5 * theta[1]);
        let r2 = c.norm2() - theta[2];
        if !(r2 > 0.0) || !r2.is_finite() {
            return None;
        }
        let r = r2.sqrt();
        let p_loc = self.u.val();
        let rel = p_loc.sub(c);
        let dist = rel.norm();
        let l = self.unit_len;
        let w = self.m.w.max(1.0e-300);
        let sigma_rel = (f.p.max(0.0) / w).sqrt() / (2.0 * r.max(1.0e-12));

        Some(Report {
            k: self.k,
            radius: r * l,
            center: c.add(self.off).mul(l),
            p_rel: rel.mul(l),
            dist: dist * l,
            radial_err: (dist - r) * l,
            loss: f.p / w,
            sigma_rel,
            weight: self.m.w,
            p1: f.p1,
            p2: f.p2,
            obs,
            steps: self.steps,
            clamped,
        })
    }
}

// ===========================================================================
// 观测器
// ===========================================================================

/// 基于流式 Kåsa 圆拟合的磁链 / 电阻观测器。
#[derive(Clone, Debug)]
pub struct KasaFluxObserver {
    // ---- 拟合 ----
    pub fit: FitConfig,
    /// 是否在线辨识 `R`（关闭后只估计圆心/半径，`k` 保持 `set_initial` 的值）。
    pub fit_enabled: bool,
    /// 半径范围的相对尺度（相对 `set_initial` 的 `radius`）。
    pub radius_scale: (S, S),
    /// `k` 范围的相对尺度（相对 `set_initial` 的 `r`）。
    pub k_scale: (S, S),
    /// 开始把拟合圆心用于测角前，至少累计的弧长（rad）。
    ///
    /// 小弧下圆心/半径病态，这段时间用 `set_initial` 的已知圆心（ALIGN）。
    pub min_arc: S,
    /// 开始把拟合圆心用于测角前的最小有效样本数。
    pub min_weight: S,
    /// 允许做初始化网格扫描所需的最小有效样本数 `Σw`。
    pub scan_weight: S,
    /// 半径更新的可信度门限：`sigma_rel` 超过它时只更新圆心、保持旧半径。
    ///
    /// 半径是 `ψf/L` 的副产物；高噪声下半径会被代数拟合偏大，但圆心（决定
    /// 角度）仍可用。保持旧半径比报告一个 +7% 的噪声半径更诚实。
    pub radius_sigma_rel: S,
    /// 第一次初始化扫描的最早时刻（秒）。
    pub scan_time: S,
    /// 初始化扫描失败后的重试间隔（秒）。
    pub scan_retry: S,
    /// 初始化扫描的最大尝试次数（含第一次）。
    pub scan_max_tries: usize,

    // ---- PLL ----
    pub pll_wn_rel: S,
    pub pll_wn_min: S,
    pub pll_wn_min_run: S,
    pub pll_narrow_time: S,
    pub pll_zeta: S,
    /// `|p-c|/radius` 的可信区间。
    pub mag_lo: S,
    pub mag_hi: S,

    // ---- 输出 / 诊断 ----
    /// `R = Rs/L`（s⁻¹）。
    pub r: S,
    /// 圆半径（A），即 `ψf/L`。
    pub radius: S,
    /// 电角度估计。
    pub theta: S,
    /// 电角速度估计。
    pub omega: S,
    /// 最近一次输出的模长 / 角度是否可信。
    pub valid: bool,
    /// 最近一次 `atan2(-(p-c))` 得到的测量角（未过 PLL）。
    pub theta_meas: S,
    /// 最近一次 `|p-c|`。
    pub flux_mag: S,
    /// 最近一次剖面的无量纲可观性指数。
    pub obs: S,
    /// 最近一次剖面的 `P'`、`P''`。
    pub p1: S,
    pub p2: S,
    /// 径向残差 RMS / 半径：圆模型可信度（> `fit.max_sigma_rel` 时冻结 `k`）。
    pub sigma_rel: S,
    /// 最近一拍实际走的 Newton 步数。
    pub steps: u32,
    /// 有效样本数 `Σw`。
    pub weight: S,
    /// 数据实际覆盖的弧长（rad），由 `Σ|Δp| / radius_nominal` 得到。
    pub data_arc: S,
    /// 当前圆心（A，绝对坐标）。
    pub center: V2,
    /// 已经用上拟合圆心的样本数。
    pub fit_updates: usize,
    /// 有效更新次数。
    pub updates: usize,
    /// 初始 `R` 与半径（有效性闸门 / 诊断）。
    pub r_nominal: S,
    pub radius_nominal: S,
    /// 是否已经被拟合圆心接管。
    pub use_fit: bool,
    /// 是否已经做过初始化网格扫描。
    pub scan_done: bool,
    /// 是否已允许 Newton 更新 `R`（首轮成熟扫描之后才打开）。
    pub newton_enabled: bool,
    /// 已经做过的初始化扫描次数。
    pub scan_tries: usize,

    tracker: Option<Tracker>,
    unit_len: S,
    total_time: S,
    arc: S,
    narrowed: bool,
    theta0: S,
    last: ObserverOutput,
}

impl Default for KasaFluxObserver {
    fn default() -> Self {
        Self {
            fit: FitConfig::default(),
            fit_enabled: true,
            radius_scale: (0.3, 3.0),
            k_scale: (0.1, 10.0),
            min_arc: 2.0 * std::f64::consts::PI,
            min_weight: 8.0,
            scan_weight: 200.0,
            radius_sigma_rel: 0.05,
            scan_time: 0.2,
            scan_retry: 0.2,
            scan_max_tries: 6,
            pll_wn_rel: 0.2,
            pll_wn_min: 50.0,
            pll_wn_min_run: 1.0,
            pll_narrow_time: 2.0,
            pll_zeta: 0.9,
            mag_lo: 0.3,
            mag_hi: 3.0,
            r: 0.0,
            radius: 0.0,
            theta: 0.0,
            omega: 0.0,
            valid: false,
            theta_meas: 0.0,
            flux_mag: 0.0,
            obs: 0.0,
            p1: 0.0,
            p2: 0.0,
            sigma_rel: 0.0,
            steps: 0,
            weight: 0.0,
            data_arc: 0.0,
            center: V2::ZERO,
            fit_updates: 0,
            updates: 0,
            r_nominal: 0.0,
            radius_nominal: 0.0,
            use_fit: false,
            scan_done: false,
            newton_enabled: false,
            scan_tries: 0,
            tracker: None,
            unit_len: 1.0,
            total_time: 0.0,
            arc: 0.0,
            narrowed: false,
            theta0: 0.0,
            last: ObserverOutput::default(),
        }
    }
}

impl KasaFluxObserver {
    pub fn new() -> Self {
        Self::default()
    }

    /// 写入启动（ALIGN / 交班）时刻的状态。
    ///
    /// `theta` 是已知的初始电角度，`r` / `radius` 是初始参数（`r` 允许偏离真值）。
    /// 圆心按 `p = 0 ⟹ c = radius·n(theta)` 初始化。在拟合可用之前用它测角，
    /// 因此启动阶段的精度取决于 `radius` 的铭牌误差。
    pub fn set_initial(&mut self, theta: S, omega: S, r: S, radius: S) {
        self.theta = wrap_pi(theta);
        self.theta0 = self.theta;
        self.omega = omega;
        self.r = r;
        self.radius = radius;
        self.r_nominal = r;
        self.radius_nominal = radius;

        let mut cfg = self.fit;
        cfg.r_min = self.radius_scale.0 * radius;
        cfg.r_max = self.radius_scale.1 * radius;
        cfg.k_min = self.k_scale.0 * r;
        cfg.k_max = self.k_scale.1 * r;
        self.fit = cfg;
        self.unit_len = 0.5 * (cfg.r_min + cfg.r_max);
        self.tracker = Some(Tracker::new(cfg, r));

        self.center = V2::new(self.theta.cos(), self.theta.sin()).mul(radius);
        self.total_time = 0.0;
        self.arc = 0.0;
        self.narrowed = false;
        self.use_fit = false;
        self.scan_done = false;
        self.newton_enabled = false;
        self.scan_tries = 0;
        self.fit_updates = 0;
        self.updates = 0;
        self.valid = false;
        self.obs = 0.0;
        self.p1 = 0.0;
        self.p2 = 0.0;
        self.sigma_rel = 0.0;
        self.steps = 0;
        self.weight = 0.0;
        self.data_arc = 0.0;
        self.theta_meas = self.theta;
        self.flux_mag = radius;
        self.last = ObserverOutput {
            electrical_angle: self.theta,
            electrical_speed: self.omega,
            continuous_speed: self.omega,
        };
    }

    /// 诊断：返回当前样本在绝对坐标中的位置与圆心 `(p, c)`。
    pub fn debug_state(&self) -> (V2, V2) {
        let (p, c) = match self.tracker.as_ref() {
            Some(tr) => {
                let p = tr.p_local().add(tr.off()).mul(tr.unit_len());
                (p, self.center)
            }
            None => (V2::ZERO, self.center),
        };
        (p, c)
    }

    /// 把可在线修改的拟合参数同步给 `Tracker`（不重置已累积的矩）。
    ///
    /// `set_initial` 会把 `fit` 里与量纲/范围有关的字段固化进 `Tracker`；其余
    /// 调参（τ、门限、步长、先验、Newton 步数）允许在运行中修改，本函数负责
    /// 把它们送进去。`update` 每拍都会调用一次，开销是十几个标量赋值。
    pub fn sync_fit(&mut self) {
        let Some(tr) = self.tracker.as_mut() else {
            return;
        };
        tr.cfg.tau = self.fit.tau;
        tr.cfg.obs_min = self.fit.obs_min;
        tr.cfg.scan_obs_min = self.fit.scan_obs_min;
        tr.cfg.k_prior_rel = self.fit.k_prior_rel;
        tr.cfg.k_step_rel = self.fit.k_step_rel;
        tr.cfg.min_weight = self.fit.min_weight;
        tr.cfg.max_sigma_rel = self.fit.max_sigma_rel;
        tr.cfg.scan_rel = self.fit.scan_rel;
        tr.cfg.scan_gain = self.fit.scan_gain;
        tr.cfg.scan_points = self.fit.scan_points;
        tr.cfg.recenter_eps = self.fit.recenter_eps;
        tr.cfg.p2_eps = self.fit.p2_eps;
        tr.cfg.r_kappa = self.fit.r_kappa;
    }
}

impl Observer<3> for KasaFluxObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        self.sync_fit();
        if !(delta_time > 0.0) || !delta_time.is_finite() {
            self.valid = false;
            return self.last.clone();
        }
        // 墙钟时间：即使本拍没有零矢量，遗忘也应继续。
        self.total_time += delta_time;
        if !self.narrowed && self.total_time >= self.pll_narrow_time {
            self.pll_wn_min = self.pll_wn_min_run;
            self.narrowed = true;
        }

        let slope_dt = input.current_slope_dt;
        if !(slope_dt > 0.0) || !slope_dt.is_finite() {
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

        let Some(tr) = self.tracker.as_mut() else {
            self.valid = false;
            return self.last.clone();
        };

        // 零矢量越长，斜率噪声越小：w ∝ Δt²。
        let q = if self.fit_enabled {
            (slope_dt / delta_time).powi(2).clamp(1.0e-6, 1.0e3)
        } else {
            1.0
        };
        let iters = if self.fit_enabled && self.newton_enabled {
            self.fit.inner_iters
        } else {
            0
        };
        // U = ∫s0 dt（斜率积分），V = ∫i dt（电流积分）。
        let mut rep = tr.update(
            s.mul(delta_time),
            i.mul(delta_time),
            self.total_time,
            q,
            iters,
        );

        // 数据实际覆盖的弧长：路径长度 / 标称半径。它与 k 无关，k 错时也不会
        // 像 PLL 的 |ω|dt 那样被低估。
        self.data_arc = if self.radius_nominal > 0.0 {
            tr.path() * tr.unit_len() / self.radius_nominal
        } else {
            0.0
        };

        // 窗口成熟后做初始化网格扫描。第一轮之后即使扫描没找到更好的盆地，
        // 也允许 Newton 接管（此时窗口已足够长，obs 门限会挡住不可辨识的样本）；
        // 之后若扫描仍失败则隔 scan_retry 重试，最多 scan_max_tries 次。
        let scan_due =
            self.total_time >= self.scan_time + self.scan_tries as S * self.scan_retry;
        if self.fit_enabled
            && !self.scan_done
            && tr.weight() >= self.scan_weight
            && scan_due
            && self.data_arc >= self.min_arc
        {
            self.scan_tries += 1;
            if tr.initialize() {
                self.scan_done = true;
                self.newton_enabled = true;
                rep = tr.report_now();
            } else if self.scan_tries >= self.scan_max_tries.max(1) {
                // 激励一直不够：放弃找盆地，从当前 k 开始让 obs 门限自己冻结。
                self.scan_done = true;
                self.newton_enabled = true;
            }
        }

        self.r = tr.k();
        self.weight = tr.weight();
        self.updates += 1;
        if let Some(rp) = rep {
            self.obs = rp.obs;
            self.p1 = rp.p1;
            self.p2 = rp.p2;
            self.sigma_rel = rp.sigma_rel;
            self.steps = rp.steps;
            if rp.weight >= self.min_weight.max(self.fit.min_weight)
                && (self.use_fit || self.data_arc >= self.min_arc)
            {
                self.center = rp.center;
                if rp.sigma_rel <= self.radius_sigma_rel {
                    self.radius = rp.radius;
                }
                self.use_fit = true;
                self.fit_updates += 1;
            }
        }

        // 角度测量：拟合接管后用剖面圆心；启动阶段用已知圆心。
        let rel = match rep {
            Some(rp) if self.use_fit => rp.p_rel,
            _ => {
                let p = tr.p_local().add(tr.off()).mul(tr.unit_len());
                p.sub(self.center)
            }
        };
        let mag = rel.norm();
        let theta_meas = (-rel.y).atan2(-rel.x);
        self.theta_meas = theta_meas;
        self.flux_mag = mag;

        // 二阶 PLL：ω̂ += ki·e·dt，θ̂ += (ω̂ + kp·e)·dt
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
        out
    }
}

// ===========================================================================
// 测试
// ===========================================================================

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
        assert!((before.g0 - a.g0).abs() < 1e-9);
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

    #[test]
    fn observer_tracks_angle_with_ideal_slope() {
        let r = 384.0;
        let radius = 4.04;
        let omega = TAU * 16.0;
        let dt = 5.0e-5;
        let stream = synth_stream(r, radius, omega, |t| 3.0 + 0.8 * (TAU * 0.5 * t).cos(), 3.0, dt);

        let mut ob = KasaFluxObserver::new();
        ob.set_initial(0.0, 0.0, r, radius);
        ob.pll_wn_min = 50.0;

        let mut worst: S = 0.0;
        for (k, (_, du, dv)) in stream.iter().enumerate() {
            let t = k as S * dt;
            let input = ObserverInput {
                voltage: [0.0; 3],
                current: inverse_clarke([dv.x / dt, dv.y / dt]),
                current_slope: inverse_clarke([du.x / dt, du.y / dt]),
                current_slope_dt: dt,
                target_speed: 0.0,
            };
            let out = ob.update(dt, &input);
            if t > 1.0 {
                worst = worst.max(wrap_pi(out.electrical_angle - wrap_pi(omega * t)).abs());
            }
        }
        eprintln!(
            "angle: worst={:.4}rad k={:.3} r={:.4} obs={:.2e}",
            worst,
            ob.r,
            ob.radius,
            ob.obs
        );
        assert!(worst < 0.05, "worst angle error = {worst} rad");
    }

    #[test]
    fn observer_recovers_r_with_excitation() {
        let r = 384.0;
        let radius = 4.04;
        let omega = TAU * 16.0;
        let dt = 5.0e-5;
        let stream = synth_stream(r, radius, omega, |t| 3.0 + 1.5 * (TAU * 0.7 * t).cos(), 3.0, dt);

        let mut ob = KasaFluxObserver::new();
        ob.set_initial(0.0, 0.0, 1.25 * r, radius);
        ob.pll_wn_min = 50.0;
        ob.fit.inner_iters = 2;

        for (_, du, dv) in stream.iter() {
            let input = ObserverInput {
                voltage: [0.0; 3],
                current: inverse_clarke([dv.x / dt, dv.y / dt]),
                current_slope: inverse_clarke([du.x / dt, du.y / dt]),
                current_slope_dt: dt,
                target_speed: 0.0,
            };
            ob.update(dt, &input);
        }
        let rel = (ob.r - r).abs() / r;
        eprintln!(
            "observer recover: k={:.3} true={r:.3} rel={rel:.4} radius={:.3} obs={:.2e} scan={}",
            ob.r, ob.radius, ob.obs, ob.scan_done
        );
        assert!(rel < 0.05, "k={} rel={rel}", ob.r);
        assert!((ob.radius - radius).abs() / radius < 0.05);
    }

    #[test]
    fn observer_keeps_angle_with_wrong_r_at_steady_state() {
        // 稳态下 R 不可辨识；即使 R 错 25%，角度仍应由圆几何正确给出。
        let r = 384.0;
        let radius = 4.04;
        let omega = TAU * 16.0;
        let dt = 5.0e-5;
        let stream = synth_stream(r, radius, omega, |_| 2.5, 2.0, dt);

        let mut ob = KasaFluxObserver::new();
        ob.set_initial(0.0, 0.0, 1.25 * r, radius);
        ob.pll_wn_min = 50.0;
        ob.fit_enabled = false; // 冻结 R，只验证几何测角

        let mut worst: S = 0.0;
        for (k, (_, du, dv)) in stream.iter().enumerate() {
            let t = k as S * dt;
            let input = ObserverInput {
                voltage: [0.0; 3],
                current: inverse_clarke([dv.x / dt, dv.y / dt]),
                current_slope: inverse_clarke([du.x / dt, du.y / dt]),
                current_slope_dt: dt,
                target_speed: 0.0,
            };
            let out = ob.update(dt, &input);
            if t > 1.0 {
                worst = worst.max(wrap_pi(out.electrical_angle - wrap_pi(omega * t)).abs());
            }
        }
        eprintln!("wrong-R angle: worst={:.4}rad r={:.3}", worst, ob.r);
        assert!(worst < 0.05, "worst angle error = {worst} rad");
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
        chk(a.u.x, b.u.x, "u.x");
        chk(a.u.y, b.u.y, "u.y");
        chk(a.v.x, b.v.x, "v.x");
        chk(a.v.y, b.v.y, "v.y");
        for i in 0..2 {
            for j in 0..2 {
                chk(a.uu[i][j], b.uu[i][j], "uu");
                chk(a.uv[i][j], b.uv[i][j], "uv");
                chk(a.vv[i][j], b.vv[i][j], "vv");
            }
        }
        chk(a.a0, b.a0, "a0");
        chk(a.au.x, b.au.x, "au.x");
        chk(a.au.y, b.au.y, "au.y");
        chk(a.av.x, b.av.x, "av.x");
        chk(a.av.y, b.av.y, "av.y");
        chk(a.b0, b.b0, "b0");
        chk(a.bu.x, b.bu.x, "bu.x");
        chk(a.bu.y, b.bu.y, "bu.y");
        chk(a.bv.x, b.bv.x, "bv.x");
        chk(a.bv.y, b.bv.y, "bv.y");
        chk(a.g0, b.g0, "g0");
        chk(a.gu.x, b.gu.x, "gu.x");
        chk(a.gu.y, b.gu.y, "gu.y");
        chk(a.gv.x, b.gv.x, "gv.x");
        chk(a.gv.y, b.gv.y, "gv.y");
        chk(a.aa, b.aa, "aa");
        chk(a.ab, b.ab, "ab");
        chk(a.bb, b.bb, "bb");
        chk(a.ag, b.ag, "ag");
        chk(a.bg, b.bg, "bg");
        chk(a.gg, b.gg, "gg");
        assert!(worst < tol, "{what}: worst field {who} err {worst}");
    }
}
