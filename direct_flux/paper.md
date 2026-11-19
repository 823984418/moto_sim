# 无锁相环的直接磁链积分观测器：双速率积分、加锁交班与可观测性加权单步牛顿

**摘要** 磁链圆观测器的角度精度最终受限于两件事：圆拟合给出的相位是否可信，以及从相位到角度输出的滤波器是否跟得上转速变化。上一版实现把圆拟合的相位送进二阶锁相环（PLL）；PLL 在稳态能压噪声，但在加速/减速后会把角度拖出 1~3 rad，减速到停机时甚至丢锁（实测稳态误差 125°）。本文去掉 PLL，改为**直接磁链积分 + 反正切**：电流环频率上维护 $\int s_0\,dt$ 与 $\int i\,dt$ 两个积分器，用 $p=c+\int s_0+R\int i$ 直接取 $\theta=\operatorname{atan2}(-p_y,-p_x)$；圆拟合降到 10 ms 一次，只负责 $R$、圆心和半径。为了在“清积分器”时不打断角度，本文给出一个**加锁交班**：清零前先把积分值吸收进相对圆心 $c$（$c\leftarrow c+a+k b$，磁链严格不变），再把拟合圆心、$R$、半径写成目标，三者用同一个增益按可观性加权滑过去；拟合本身每次只做**一步** Newton，步长只由可观性 $P''/(4R^2\sum w|V|^2)$ 缩放，不设基于可观性的冻结门限。真实电机仿真里，加速→减速到 20 rad/s 的稳态角度误差从旧实现的 7.7° 降到 1.55°，减速到 0 从 125° 降到 1.94°；代价是 10 mA 噪声下角度为 17.4°（无 PLL 的原始噪声），且 $R$ 在 10 mA 下会被噪声推走，这是“只按可观测性加权”的直接后果，本文在 §10 如实讨论。

**关键词** 磁链观测器；无锁相环；双速率；积分器交班；变量投影；可观测性加权；Kåsa 圆拟合

---

## 1 引言

### 1.1 问题从哪里来

上一版（`kasa_flux_observer`，2026-11-18）把磁链圆观测器的拟合层换成了指数遗忘的流式 Kåsa：36 个矩、每拍一次变量投影 Newton、圆心与当前 $R$ 始终一致。它在恒速稳态下表现很好（1 mA 被动观测 0.55°，10 mA 5.3°），但角度输出经过一个二阶 PLL。PLL 的带宽在 `pll_narrow_time` 之后降到 $\max(0.2|\omega|,1)$：20 rad/s 时只有 4 rad/s。二阶 PLL 跟踪频率斜坡的稳态相位滞后是

$$
e_{ss}=\frac{\dot\omega}{w_n^2},
$$

减速 $1000\ \mathrm{rad/s^2}$ 时 $w_n=4$ 给出 62 rad 的滞后，必然绕圈；停机后 $w_n=1$，恢复时间常数约 1 s。实测“先加速再减速”后角度会冲到 2~3 rad，减速到 0 时稳态 RMS 125°、偏置 95°。对照旧 `flux_circle_observer` 同样失败（126.9°），说明这是 PLL 带宽调度的问题，不是圆拟合的几何问题：同一段数据里圆上测量角误差始终 $<0.06$ rad。

### 1.2 本文做了什么

1. **去掉 PLL**。角度由快速环的磁链积分直接反正切得到，转速只对 $\Delta\theta/\Delta t$ 做一阶低通；不再有 PLL 的捕获、窄带切换和丢锁。
2. **双速率**。$\int s_0$、$\int i$ 与测角在电流环频率（20 kHz）；圆拟合降到 `fit_period = 10 ms`，每次只从快速积分器取一次增量，矩更新与 Newton 都在低速环。
3. **加锁交班**。清积分器、写目标圆心/半径/$R$ 都在同一把 `Arc<Mutex<...>>` 里完成，并且先做 $c\leftarrow c+a+k b$ 保证磁链连续；三个目标用同一个增益按可观性加权滑过去，避免角度跳变，也避免把拟合值一次写死。
4. **最新样本原点 + 单步牛顿**。每个拟合样本都把 $U$、$V$ 精确平移到“最新样本在原点”，于是拟合圆心天然相对最新点；每次拟合只做一步 Newton，步长只由可观性缩放
   $w=\big(\mathrm{clamp}(|P''|/(4R^2\sum w|V|^2)/\text{obs\_min},0,1)\big)^2$，
   不设基于观测性的冻结门限，也不加随机游走先验。

### 1.3 与上一版的关系

拟合的 36 个矩、变量投影、剪切、$V$ 的原点管理、半径伪观测都复用上一版；它们现在和观测器一起放在 `direct_flux_observer.rs` 的“拟合核心”里（旧 `KasaFluxObserver` 已删除）。本文新增的是**观测器层的时序与交班**：`DirectFluxObserver` 和 `Tracker::step_once`。

---

## 2 磁链圆模型

### 2.1 从零矢量斜率到圆

零矢量期间端电压为零，电流斜率记为 $s_0$。由 PMSM 电压方程，

$$
s_0=\frac{di}{dt}\Big|_{v=0}=-\frac{R_s i+\omega\times\lambda}{L},
$$

其中 $R_s$ 是定子电阻、$L$ 是电感、$\lambda$ 是磁链。记

$$
A(t)=\int s_0\,dt,\qquad B(t)=\int i\,dt,\qquad k=\frac{R_s}{L},
$$

则

$$
A+kB=-\frac{\lambda(t)-\lambda(0)}{L}.
$$

当 $\lambda(t)=\psi_f\,n(\theta)$ 旋转时，$p(k)=A+kB$ 落在圆心 $c=\lambda(0)/L$、半径 $\rho=\psi_f/L$ 的圆上：

$$
\lvert p(k)-c\rvert=\rho,\qquad \theta=\operatorname{atan2}\big(-(p-c)_y,\,-(p-c)_x\big).
$$

### 2.2 待估量与可观性

$k=R_s/L$ 是待辨识量，$\rho=\psi_f/L$ 是半径，$c$ 是圆心。$R$ 对角度的影响来自 $p(k)$ 对 $k$ 的导数：$V=B$。若 $B$ 正好是绕圆心的无穷小相似变换（旋转 + 缩放），则 $p(k)$ 仍是同一个圆（只是缩放/旋转），$k$ 与半径完全混淆——这正是**稳态不可辨识**：恒 $i_q$、恒 $\omega$ 时只有启动/负载瞬态才提供 $k$ 的信息。本文的可观测性指数

$$
\mathrm{obs}=\frac{P''(k)}{4R^2\sum_n w_n\lvert V_n\rvert^2}
$$

就是“剖面在当前 $k$ 处的曲率，相对于满激励曲率”的无量纲比值；稳态趋于 0，瞬态到 $10^{-1}$ 量级。

---

## 3 双速率结构

### 3.1 快速环（电流环频率）

快速环状态

```rust
struct FastFlux {
    a: V2,        // ∫ s0 dt（内部单位）
    b: V2,        // ∫ i dt（内部单位）
    c: V2,        // 相对圆心：磁链相对圆心的偏移 = c + a + k b
    k: S,         // 当前 R
    rad: S,       // 当前半径（内部单位）
    theta_prev: S,
    omega_lp: S,
    q_acc: S,     // Σ (slope_dt/Δt)²
}
```

每个 `update`（$\Delta t=50\ \mu s$）：

$$
a\leftarrow a+s_0\,\Delta t/\ell,\qquad b\leftarrow b+i\,\Delta t/\ell,
$$

其中 $\ell=(r_{\min}+r_{\max})/2$ 是内部长度单位（§6），与 §2.1 的电感 $L$ 区分。

然后

$$
p=c+a+k\,b,\qquad \theta=\operatorname{atan2}(-p_y,-p_x).
$$

转速不用 PLL，只对角度差做一阶低通：

$$
\omega\leftarrow \omega+\alpha\left(\frac{\Delta\theta}{\Delta t}-\omega\right),\qquad
\alpha=\mathrm{clamp}\!\left(\frac{\Delta t}{\tau_\omega},0,1\right),
$$

默认 $\tau_\omega=20$ ms。$\omega$ 只用于电流环前馈/速度环，**不参与角度**。

### 3.2 低速环（10 ms）

`fit_accum` 累加墙钟时间；到 `fit_period`（默认 10 ms，即 200 个快速拍）时，在锁内取走 $a,b,q$ 并清零，交给 `Tracker::step_once`：

1. 按 $\rho=e^{-\Delta t/\tau}$ 衰减 36 个矩（$\tau=1$ s，$\Delta t=10$ ms）；
2. 把增量 $(\Delta A,\Delta B)$ 入矩：$U_s\leftarrow U_s+\Delta A+k_{\mathrm{ref}}\Delta B$，$V\leftarrow V+\Delta B$；
3. 最新样本搬到原点（§5.1）；
4. 恰好一次 Newton（§5.3）；
5. 再搬一次原点，保证下一周期从 0 起算；
6. 返回报告与局部圆心 $c_{\mathrm{local}}$。

### 3.3 无 PLL 的代价

角度不再被滤波，10 mA 噪声下输出就是圆上测量角的噪声。实测 1 mA 稳态 1.56°（旧 PLL 0.55°），10 mA 17.4°（旧 5.3°）。这是“去掉 PLL”的直接代价；若需要更平滑的输出，应另加一个**输出低通**（不构成闭环），而不是把 PLL 加回来。

---

## 4 加锁交班

### 4.1 为什么需要交班

圆拟合的内部矩是在“上次清零以来”的坐标里累积的；每 10 ms 要把积分器清零给下一段用。若直接把 $a,b$ 清零，磁链 $p=c+a+k b$ 会跳变，角度就跳。因此清零与圆心更新必须原子地完成。

### 4.2 清零时保持磁链

设清零前 $p^-=c+a+k_{\mathrm{cur}}b$。清零后 $a=b=0$，要让 $p^+=p^-$，只需

$$
\boxed{\,c\leftarrow c+a+k_{\mathrm{cur}}b\,}
$$

这一步在**同一把锁**里与“取走并清零 $a,b$”一起做，所以快速环（若在另一个中断里）永远看不到不一致的中间状态。注意这里用的是**当前** $k$，不是拟合的新 $k$。

### 4.3 三个目标

拟合完成后得到：

- 局部圆心 $c_{\mathrm{local}}$（以最新样本为原点、当前 $k_{\mathrm{fit}}$ 下）；
- 拟合 $R$：$k_{\mathrm{fit}}$；
- 拟合半径：$\rho_{\mathrm{fit}}$。

最新样本相对圆心的正确位置是 $-c_{\mathrm{local}}$，所以圆心目标

$$
c_{\mathrm{tgt}}=-c_{\mathrm{local}}.
$$

$R$ 与半径直接以拟合值为目标。三者在同一把锁里写入，并用**同一个**增益 $g$（`sync_gain`，默认 0.05）滑动：

$$
\begin{aligned}
c&\leftarrow c+g\,(c_{\mathrm{tgt}}-c),\\
k&\leftarrow k+g\,(k_{\mathrm{fit}}-k),\\
\mathrm{rad}&\leftarrow \mathrm{rad}+g\,(\rho_{\mathrm{fit}}/\ell-\mathrm{rad}).
\end{aligned}
$$

其中 `rad` 是内部单位半径，$\ell=(r_{\min}+r_{\max})/2$ 是内部长度单位（与 §2.1 的电感 $L$ 区分）。

$g=0.05$ 对应约 20 个拟合周期（200 ms）收敛。因为清零后 $b=0$，$k$ 的变化不会立刻改变 $p$；角度只会因 $c$ 的加权移动而平滑变化。

### 4.4 为什么默认不乘可观性

拟合的 Newton 步已经由可观性加权（§5.3），交班只是把**已经收敛的拟合值**搬到快速环。若交班再乘 $\mathrm{obs}$，稳态 $\mathrm{obs}\to0$ 时快速环会停在过时的 $R$/圆心，与拟合的圆不一致：实测 1.25× 初值时 $R$ 卡在 +10%，角度出现 $\pi$ 偏置。因此默认 `sync_obs_weighted = false`；保留该开关做 A/B。

### 4.5 连续性

清零瞬间 $p$ 不变（§4.2）；随后 $c$ 的加权移动使 $p$ 变化 $g(c_{\mathrm{tgt}}-c)$，是**有意的**平滑修正，不是跳变。下一拍起 $a,b$ 重新累积，$k$ 已是新值，磁链从同一点继续。若拟合不可用（`step_once` 返回 `None`），仍然执行 $c\leftarrow c+a+k b$，积分器清零不会造成角度跳变。

### 4.6 时序

```text
每拍（快速环，锁内）:
    a += s0·dt/L ; b += i·dt/L ; q_acc += (slope_dt/dt)²
    if fit_accum >= fit_period:
        (a0,b0,q0) = (a,b,q_acc); a=b=q_acc=0
        c += a0 + k_cur·b0          // 磁链连续
        take = (a0,b0,q0)

锁外:
    (rep,c_local) = tracker.step_once(a0·L, b0·L, t, q0)
    if rep:
        锁内:
            c_tgt = -c_local
            c += g(c_tgt-c); k += g(k_fit-k); rad += g(rad_fit-rad)
            k_cur = k

锁内:
    p = c + a + k·b
    θ = atan2(-p.y,-p.x)
    ω += α(Δθ/dt - ω)
```

---

## 5 最新样本原点与单步牛顿

### 5.1 把最新样本搬到原点

每个拟合样本加入矩之后，对全部历史做两次**精确平移**：

$$
U_s\leftarrow U_s-u_{\mathrm{new}},\qquad V'\leftarrow V'-v_{\mathrm{new}},
$$

即 `Tracker::recenter_latest`。它是精确变换：圆拟合对平移不变，半径/圆心只是跟着平移；`off`、`v_off` 累加被搬走的偏移，绝对坐标诊断不受影响。之后 `p_local == 0`，下一个周期的增量从 0 起算。

### 5.2 剪切与 $v_{\mathrm{off}}$

矩里存的是 $U_s=U+k_{\mathrm{ref}}V$。Newton 改变 $k$ 时对整个历史做剪切，并把物理 $V=V'+v_{\mathrm{off}}$ 的缺失部分补上：

$$
U_s\leftarrow U_s+\Delta k\,V',\qquad
U_s\leftarrow U_s+\Delta k\,v_{\mathrm{off}},\qquad
U_{\mathrm{new}}\leftarrow U_{\mathrm{new}}+\Delta k\,v_{\mathrm{off}}.
$$

`Tracker::newton` 里对应 `shear(dk)`、`translate_u(v_off·(-dk))`、`u += v_off·dk`。漏掉这一项会系统性偏（上一版的单测钉住了它）。

### 5.3 可观性加权的一步 Newton

对当前矩做变量投影，得到 $P$、$P'$、$P''$。Newton 步

$$
\delta k=-\frac{P'}{P''}\cdot w,\qquad
w=\left(\mathrm{clamp}\!\left(\frac{|P''|}{4R^2\sum w|V|^2}\Big/\mathrm{obs\_min},0,1\right)\right)^2.
$$

- `obs_min`（默认 $5\times10^{-3}$）只是**权重参考尺度**，不是“过/不过”的门限；
- 平方主要是压低**稳态**的噪声步长（约两个数量级）；实测 1 mA 的 $+1.38\%$ 残差主要来自启动瞬态，不是稳态噪声。线性权重在稳态会额外累积到 $+1.6\%/8$ s；
- `obs<0`（当前点剖面凹）时权重用 $|obs|$，但分母 $P''$ 仍须为正，否则本步跳过（见 §5.5）。

步长再经信任域 $|\delta k|\le 0.02(1+|k|)$ 截断，并用带罚项的 $P$ 做回溯线搜索（只接受下降的步长，最多折半 10 次）。每次拟合**恰好一步**，不做内层迭代；连续 100 次拟合等效于 100 步 Newton。

### 5.4 半径约束

若拟合半径越出 $[r_{\min},r_{\max}]$，把线性化伪观测 $\mu(g^\top\theta-z)^2$ 加进 $M_0/b_0/c_0$（$\mu=50\sum w$），这一步 Newton、线搜索和输出都用同一个带罚项剖面（上一版的 `fit_hinge`/`fit_with_penalty`）。范围内则完全不介入——这是先验范围，不是 $R$ 的门限。

### 5.5 剩余的门限行为（审计）

按“只让可观性给步长加权”的原则，直接路径里剩下的“门限类”行为只有下面这些，分三类：

| 类别 | 位置 | 行为 | 是否可去 |
|---|---|---|---|
| 结构性 | `m.w < min_weight` | 少于 3~4 个样本，圆拟合奇异、$k$ 无信息 | 不可去（可解性下限） |
| 数值 | `denom = P'' <= p2_eps` | Newton 分母非正，跳过本步 | 不可去（除零/凹点保护）；它不是按噪声或可观性冻结 |
| 数值 | `step.abs() <= 1e-14(1+|k|)` | 已收敛，停止 | 不可去 |
| 限幅 | `|δk| <= k_step_rel(1+|k|)` | 信任域截断 | 可调，但不是冻结 |
| 限幅 | `k_min <= k <= k_max` | 硬范围 | 可调 |
| 算法 | 线搜索 10 次折半 | 只接受 $P$ 下降 | 可去但会失去鲁棒性 |
| 先验 | 半径 hinge `[r_min,r_max]` | 越界才加罚项 | 可去（但会失去半径范围） |
| 诊断 | `mag_lo/mag_hi` | 只置 `valid` | 不影响估计 |
| 采样 | `fit_accum >= fit_period` | 10 ms 调度 | 不是门限 |
| 测量 | `slope_dt <= 0` | 本拍无零矢量，不积分、`valid=false` | 无测量，不可去 |

旧实现的 `obs_min` 硬门限、`sigma_rel` 冻结、初始化扫描、`min_arc` 都没有搬进 `DirectFluxObserver`。

---

## 6 复用的数值措施

- **内部单位** $L=(r_{\min}+r_{\max})/2$：半径 $O(1)$。
- **$U$ 浮动原点**：质心超过 $0.25L$ 时精确平移，防长积分漂移。
- **$V$ 原点管理**：存储 $V'=V-v_{\mathrm{off}}$，当 $|\bar V'|>0.25\,\mathrm{RMS}(V')$ 时把均值搬进 $v_{\mathrm{off}}$；可观性尺度用物理 $V$ 的二阶矩，与 $v_{\mathrm{off}}$ 的拆分无关。
- **补偿求和**：$\int s_0$、$\int i$ 用 Neumaier 累加器。

---

## 7 复杂度

- 快速环：每拍一次积分、一次 `atan2`、一次低通，$O(1)$。
- 低速环：每 10 ms 一次 36 矩衰减/更新、一次变量投影（2~3 次 $3\times3$ Cholesky）、一次线搜索；$O(1)$，与历史长度无关。
- 内存：36 个矩 + 十几个标量（快速环另加 5 个 `V2`/标量），约 500 B。

---

## 8 Rust 实现

- `src/simulation/controller/observer/direct_flux_observer.rs`：`DirectFluxObserver`、`FastFlux`、`Observer<3>` 实现、流式拟合核心（`Moments` / `Tracker` / `FitConfig` / `Report`）以及全部单测。旧的 `kasa_flux_observer.rs`、`flux_circle_observer.rs` 已删除。
- `examples/example1.rs`：`use_observer = 12`（`direct_flux_observer`，也是唯一的高性能观测器），Scope `direct_flux` / `direct_flux_mag` / `direct_flux_valid`，以及 `angle`/`angle_error`/`speed` 里的 `direct_*` 曲线；headless 默认就是直接观测器，`--observe-only` 做被动评价。

复现：

```powershell
# 1 mA 被动观测
cargo run --release --example example1 -- --headless --observe-only `
  --seconds 8 --speed 20 --current-noise 0.001
# 加速→减速到 20
cargo run --release --example example1 -- --headless --observe-only `
  --seconds 8 --speed 20 --current-noise 0.001 `
  --speed2 100 --speed2-time 1 --speed3 20 --speed3-time 3
# 减速到 0
cargo run --release --example example1 -- --headless --observe-only `
  --seconds 8 --speed 20 --current-noise 0.001 `
  --speed2 100 --speed2-time 1 --speed3 0 --speed3-time 3
```

---

## 9 实验

### 9.1 设置

8 s，20 rad/s（电），`--observe-only`（电流环用传感器角，观测器被动），电流噪声 1 mA / 10 mA，`tau=1 s`，`fit_period=10 ms`，`sync_gain=0.05`，`obs_min=5e-3`。角度误差为电角度。

### 9.2 稳态

| 场景 | 旧 Kåsa（PLL） | 直接磁链（无 PLL） |
|---|---|---|
| 1 mA 稳态 RMS / max | 0.549° / 1.507° | **1.565° / 4.634°** |
| 1 mA $R$ / 半径 | +0.41% / −0.22% | +1.38% / −0.42% |
| 10 mA 稳态 RMS / max | 5.281° / 14.402° | **17.418° / 58.411°** |
| 10 mA $R$ / 半径 | 0.00% / +0.30% | **+59.98% / −6.40%** |

10 mA 的 $R$ 漂移是“只按可观性加权”的直接后果：噪声本身在 $P''$ 里造出 $\mathrm{obs}\approx3\times10^{-3}$，接近 `obs_min`，仅靠 $\mathrm{obs}$ 无法区分相干激励与噪声曲率（旧实现的 `sigma` 冻结把它按在铭牌值）。这一项留给 §10 讨论。

### 9.3 加速→减速（本文的主结果）

| 场景 | 旧 Kåsa（PLL） | 直接磁链 |
|---|---|---|
| 加速→减速到 20，稳态 RMS / max | 7.708° / 34.579° | **1.550° / 4.346°** |
| 减速到 0，稳态 RMS / max | 125.360° / 179.999° | **1.939° / 6.727°** |
| 减速到 0 的 $R$ / 半径 | −0.76% / −3.76% | −0.62% / −3.12% |

减速到 20 时 $R$ −0.57%、半径 +0.27%；减速到 0 时磁链圆退化成点/线，半径有 −3.1% 的漂移，但角度不再丢锁。

### 9.4 单测

`cargo test --release --lib` 共 19 个通过，其中直接观测器 3 个：

- `direct_flux_tracks_speed_profile`：20→100→20，尾段误差 0.0006 rad，逐拍最大跳变 0.0050 rad（交班连续性）；
- `direct_flux_recovers_r_with_weighted_newton`：$R$ +5% 初值，恢复到 0.02%，尾段角度误差 0.003 rad（该合成流激励较弱，测试里把 `obs_min` 降到 $10^{-5}$；真实仿真仍用 $5\times10^{-3}$）；
- `step_once_puts_latest_sample_at_origin`：每个拟合周期后 `p_local ≈ 0`，半径误差 < 0.01 A。

---

## 10 讨论

### 10.1 可观性 vs 噪声

去掉 `sigma` 门限后，10 mA 的 $R$ 会被噪声推走（+60%）。这不是交班或积分器的 bug，而是“只有可观性加权”这一原则在高噪声下的固有边界：$\mathrm{obs}$ 度量的是剖面曲率相对满激励的比值，噪声同样会抬高曲率。可选而不违反“不设门限”的补救：

- **软信噪比权重**：$w\leftarrow w\cdot\min(1,\sigma_{\mathrm{ref}}/\sigma_{\mathrm{rel}})^2$，仍是权重，不是门限；
- **相干性权重**：对 $P'$、$P''$ 先低通再判据，抑制零均值噪声；
- **接受现状**：$R$ 的错误主要造成径向缩放，角度对径向缩放不敏感（减速到 0 的 1.94° 即为例证）。

### 10.2 停机退化

$\omega=0$、$i$ 恒定时磁链点不动，圆拟合退化成点/线，半径不可辨识（实测 −3.1%）。角度只依赖圆心，受影响较小；若要求停机保持，可加“$|\omega|$ 接近 0 时冻结圆心/半径”的调度（这属于输出调度，不是 $R$ 门限）。

### 10.3 交班增益与拟合周期

`sync_gain=0.05` 对应 200 ms 交班；`fit_period=10 ms` 是刻意放慢的，便于把积分器留在快速环。把 `fit_period` 降到 5 ms 会让 $R$ 在瞬态内有更多 Newton 步，但矩更新与 CPU 翻倍。

### 10.4 与上一版的关系

上一版的 36 矩、变量投影、剪切、$V$ 原点管理、半径伪观测全部保留；本文换掉的是“观测器层”：PLL → 直接反正切、逐拍更新 → 双速率、每拍多步 Newton → 每 10 ms 一步、拟合值直接写 → 加锁加权交班。

---

## 附录 A 符号表

| 符号 | 含义 |
|---|---|
| $s_0$ | 零矢量电流斜率 |
| $A=\int s_0dt$ | 斜率积分 |
| $B=\int i dt$ | 电流积分 |
| $k=R_s/L$ | 待辨识参数 |
| $p=c+a+k b$ | 相对圆心的磁链 |
| $c$ | 相对圆心（快速环） |
| $c_{\mathrm{local}}$ | 拟合圆心（以最新样本为原点） |
| $R$、$\rho$ | 圆半径 / 拟合半径 |
| $P(k)$ | 变量投影剖面 |
| $\mathrm{obs}$ | $P''/(4R^2\sum w|V|^2)$ |
| $g$ | 交班增益 `sync_gain` |
| $\tau$ | 矩遗忘时间常数（1 s） |
| $\tau_\omega$ | 测速低通（20 ms） |

## 附录 B 交班连续性

设清零前 $p^-=c+a+k_{\mathrm{cur}}b$。清零后 $a'=b'=0$，取 $c'=c+a+k_{\mathrm{cur}}b$，则

$$
p^+=c'+a'+k_{\mathrm{cur}}b'=c+a+k_{\mathrm{cur}}b=p^-.
$$

随后目标 $c_{\mathrm{tgt}}=-c_{\mathrm{local}}$ 与 $k_{\mathrm{fit}}$、$\rho_{\mathrm{fit}}$ 写入，$c,k,\mathrm{rad}$ 各移动 $g$ 倍误差。因为 $b'=0$，$k$ 的移动不改变 $p^+$；$p$ 的变化只来自 $c$ 的 $g(c_{\mathrm{tgt}}-c)$，是人为的平滑修正。下一拍 $a,b$ 从 0 重新累积，$p$ 从同一点继续。

## 附录 C 复现与选型

- 直接磁链观测器是当前唯一的磁链圆观测器：`use_observer = 12`；headless 不需要选型参数，直接跑 `example1 --headless [--observe-only]` 即可。
- 直接观测器的参数：`fit.tau=1.0`、`fit_period=0.01`、`speed_tau=0.02`、`sync_gain=0.05`、`sync_obs_weighted=false`、`fit.k_prior_rel=0`、`fit.obs_min=5e-3`、`fit.shear=false`（`--shear` 可打开对照）。

## 参考文献

1. I. Kåsa, "A circle fitting procedure and its error analysis," *IEEE Transactions on Instrumentation and Measurement*, vol. 25, no. 1, pp. 8–14, 1976.
2. G. H. Golub and V. Pereyra, "The differentiation of pseudo-inverses and nonlinear least squares problems whose variables separate," *SIAM Journal on Numerical Analysis*, vol. 10, no. 2, pp. 413–432, 1973.
3. `2026-11-18-kasa-streaming/paper.md`，指数遗忘 Kåsa 圆拟合与变量投影（拟合层的理论来源）。
4. `direct_flux/REPORT.md`，本仓库里的实现、测试与实测记录。
