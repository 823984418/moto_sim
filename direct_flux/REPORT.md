# 直接磁链积分观测器：实现与验证

对应设计文档：[`paper.md`](paper.md)。
实现：`src/simulation/controller/observer/direct_flux_observer.rs`。
状态：项目里**唯一**的磁链圆观测器；旧的 `flux_circle_observer.rs` 与
`kasa_flux_observer.rs` 已删除，拟合核心搬到 `direct_flux_observer.rs` 的前半部分。

---

## 1. 文件与代码结构

`direct_flux_observer.rs` 现在包含两半，只服务同一个观测器：

1. **流式拟合核心**
   - `V2` / `S` / `wrap_pi`；
   - `Moments`：36 个指数遗忘加权矩（`Σw`、`ΣwU`、`ΣwV`、`ΣwUUᵀ`、`ΣwUVᵀ`、
     `ΣwVVᵀ`、`Σwα`、…、`Σwγ²`）；
   - `Coef` / `Profile` / `fit_from_coef` / `fit_with_penalty_coef`：变量投影；
   - `FitConfig` / `Tracker` / `Report`：遗忘、浮动原点、V 原点管理、半径罚项、
     可观性指数、门限与 Newton。
2. **直接观测器**
   - `FastFlux`：快速环的 `∫s0`、`∫i`、相对圆心 `c`、`R`、半径、测速；
   - `DirectFluxObserver`：双速率调度、锁内交班、`Observer<3>` 实现。

`mod.rs` 只保留 `pub mod direct_flux_observer;`；`example1` 的
`use_observer = 12` 就是它。

---

## 2. 模型

零矢量期间（端电压为零）的电流斜率记为 `s0 = di/dt|v=0`，令

```text
A(t) = ∫ s0 dt ,   B(t) = ∫ i dt ,   k = R = Rs / L ,   p(k) = A + k B
```

由 `s0 = -(Rs i + ω × λ) / L` 得

```text
p(R) = A + R B = -(λ(t) - λ(0)) / L
```

即 `p(R)` 落在圆心 `c = λ(0)/L`、半径 `ρ = ψf/L` 的圆上。相位就是电角度：

```text
θ = atan2(-(p - c).y, -(p - c).x)
```

---

## 3. 流式拟合核心

### 3.1 36 个矩与变量投影

窗口被精确压缩成 36 个与 `k` 无关的加权矩，指数遗忘

```text
Σw ← ρ Σw + q ,   ΣwU ← ρ ΣwU + q U ,   …
```

其中 `q = (slope_dt/Δt)²`（零矢量越长越可信）。Kåsa 法方程写成

```text
J(k, θ) = θᵀ A(k) θ + 2 θᵀ b(k) + c(k)
A(k) = A0 + k A1 + k² A2
b(k) = b0 + k b1 + k² b2 + k³ b3
c(k) = c0 + k c1 + k² c2 + k³ c3 + k⁴ c4
```

固定 `k` 时有闭式 `θ*(k) = -A(k)⁻¹ b(k)` 和剖面

```text
P(k) = c(k) - b(k)ᵀ A(k)⁻¹ b(k)
```

`P'`、`P''` 都用解析式（不是差分）。每个拟合样本做一次带线搜索的 Newton。

### 3.2 半径伪观测

半径的允许范围写成对 `θ` 的线性化伪观测 `μ (gᵀθ - z)²`，越界时把
`μggᵀ`、`-μzg`、`μz²` 加进 `A0/b0/c0`。于是 `P`、`P'`、`P''` 一起变成
**带罚项**的剖面，Newton 步、回溯线搜索、输出都用同一个目标；报告的
`loss` / `sigma_rel` 仍用约束后 `θ` 上的**无罚项**残差，罚项常数不会污染
噪声门限。

### 3.3 浮动原点与 V 原点管理

- `U = A` 的质心超过阈值时把全部矩精确平移 `U → U - s`，`s` 累进 `off`；
- `V = B` 的加权均值超过 `RMS(V')·recenter_v_rel` 时搬进 `v_off`
  （物理 `V = V' + v_off`）；这是纯规范，`P`、`P'`、`P''`、`k`、圆心、半径不变；
- **不再用补偿求和**：`u`、`v` 每个拟合周期清零、只累加一次增量，长积分在
  `off`/`v_off` 里；去掉 Neumaier 累加器后 4 组工况输出逐位不变。

### 3.4 为什么不再需要剪切

矩里存原始 `U`、`V'`，在 `κ = k` 处把 `b` 展开到三次、`c` 到四次
（`b3 = [g_vx, g_vy, 0]`、`c3 = 4b_g`、`c4 = g2`，都是已有矩）。直接观测器
每个拟合周期都把最新样本搬到原点，矩里的 `|U|`、`|V|` 只有“一个周期内增量”
的量级，`V` 的慢漂又在 `v_off` 里、求值时用 `translate_u(-κ·v_off)` 精确补回，
所以 `κ = k` 处没有论文 §6.1 那种大数相消。旧的剪切代码和 `--shear` 已删除
（A/B 见 §7.1）。

---

## 4. 直接观测器

### 4.1 双速率

| 环 | 频率 | 做什么 |
|---|---|---|
| 快速环 | 20 kHz（每次 `update`） | `∫s0`、`∫i`、`atan2`、一阶低通测速 |
| 低速环 | 每 `fit_period`（默认 10 ms） | 取一次积分增量、更新 36 矩、一次 Newton、交班 |

### 4.2 加锁交班

`FastFlux` 在 `Arc<Mutex<...>>` 里。到拟合时刻，在**同一把锁**内：

1. 取走 `a`、`b`、`q` 并清零；
2. `c += a + k_cur·b`（磁链 `c + a + k b` 严格连续）；
3. 锁外跑 `Tracker::step_once`；
4. 锁内把 `c`、`k`、`rad` 向目标滑动：

```text
c   ← c   + sync_gain·(c_tgt   - c)
k   ← k   + sync_gain·(k_fit   - k)
rad ← rad + sync_gain·(rad_fit - rad)
```

`c_tgt = -(p_newest - c_fit) = -c_local`。最新样本的局部 `U`、`V` 已清零，但
物理位置是 `κ·v_off`，所以 `c_local` 返回的是 `c - κ·v_off`。三者共用一个
增益，积分器清零不会让角度跳变，也不会把拟合值一次写死（默认 20 个周期
≈ 200 ms 收敛）。

### 4.3 角度与测速

```text
rel = c + a + k b ,   θ = atan2(-rel.y, -rel.x)
ω   = LPF(Δθ/Δt) ,    speed_tau = 20 ms
```

没有 PLL。上一版 PLL 在加速/减速时会因 `ω̇/wn²` 的稳态滞后把角度拖出几 rad，
直接反正切没有这个滞后；代价是角度噪声就是原始反正切噪声。

---

## 5. 门限与权重

直接观测器**不做硬冻结**，所有门限先做权重，乘在 Newton 步长上：

```text
w_obs   = clamp(|obs| / obs_min, 0, 1)²
w_sigma = clamp((σ_max / σ_rel)², 0, 1)
w_sig   = δ² / (δ² + σ_k²)
σ_k     = sqrt(2P / ((N_eff - 3) P''))
N_eff   = (Σw)² / Σw²
step    = raw_step · w_obs · w_sig · w_sigma
```

其中 `obs = P'' / (4R²Σw|V|²)`，`σ_rel` 是径向残差 RMS / 半径。
`k_sigma`（`|δ| < k_sigma·σ_k` 时跳过）和 `gate_sigma_rel` 是**可选**的硬门限，
默认关；打开时软权重仍然相乘。

---

## 6. 测试

```text
cargo test --release --lib
```

当前 12 个测试全部通过，和本观测器相关的有：

- `direct_flux_tracks_speed_profile`：加速→减速后角度跟得上，交班不跳变；
- `direct_flux_recovers_r_with_weighted_newton`：`R` 初值 +25% 能拉回；
- `k_sigma_gate_can_be_switched`：显著性硬门限可开关，且软权重仍生效；
- `step_once_puts_latest_sample_at_origin`：拟合后最新样本在局部原点；
- `radius_penalty_enters_newton`：半径罚项进入法方程，且报告损失用无罚项残差；
- `v_constant_offset_is_a_gauge`：`V` 的常数偏移不改变 `P`、`P'`、`P''`；
- `v_offset_does_not_break_k`：`v_off` 非零时 `k` 仍能从 +25% 初值恢复；
- 以及 36 矩的平移、剖面导数、观测器跟踪等基础测试。

---

## 7. 实测

命令：

```powershell
cargo run --release --example example1 -- --headless --observe-only `
  --seconds 8 --speed 100 --current-noise 0.001
```

`--observe-only` 下电流环仍用传感器角，观测器只做旁观；角度误差是电角度。

| 场景 | 角度 RMS | `R` | 半径 |
|---|---|---|---|
| 100 rad/s, 1 mA, 8 s | 1.573° | 384.167 (+0.03%) | 4.0432 (+0.02%) |
| 20 rad/s, 1 mA, 8 s | 1.557° | 384.657 (+0.16%) | 4.0365 (−0.15%) |
| 5 rad/s, 10 mA, 6 s | 17.7° | 533.7 (+38.97%) | 3.4659 (−14.26%) |
| 0.5 rad/s, 10 mA, 负载 0.5, 5 s | 65.3° | 383.47 (−0.15%) | 1.2869 |
| 闭环 20→100→5 rad/s, 1 mA, 8 s | 1.634° | 385.29 (+0.33%) | 4.0218 (−0.51%) |

10 mA 下 `R` 会被噪声推走（5 rad/s 表里 +39%）：`obs` 分不清相干激励和噪声
曲率，这是“只按可观性加权”的直接后果，也是 `--k-sigma` / `--hard-sigma`
保留的原因。1 mA 下 `R` 与半径都在 ±0.5% 内。

### 7.1 剪切 / 补偿求和 / 二次平移的 A/B

| 改动 | 结果 |
|---|---|
| 剪切（保留开关但默认关）→ 删除整条路径 | 4 组工况的 `steady`、`observer` 行逐位一致 |
| Neumaier 补偿求和 → 普通 `V2` 累加 | 同上，逐位一致 |
| `step_once` 末尾的第二次 `recenter_latest` → 删除 | 同上，逐位一致（无剪切时 Newton 不碰 `u`、`v`，第二次平移本来就是空操作） |

唯一有非零差异的是更早的剪切开关对照（0.5 rad/s、10 mA、负载 0.5）：
`shear=true` 65.341° / 383.470 / 1.2869，`shear=false` 65.335° / 383.467 /
1.2873，属于舍入量级。0.5~100 rad/s、1~10 mA、`fit_period` 放大到 0.2 s 的
其余工况都逐位一致。

结论：这套“浮动原点 + 每拍重锚”的结构里，剪切、补偿求和、第二次平移都是
冗余的，已全部删除。

### 7.2 平移 vs 拟合的开销

release 构建、单次调用：

| 操作 | 时间 |
|---|---|
| `translate_u` + `translate_v` | ~60 ns |
| 36 矩 `add` | ~15 ns |
| `coef_at` + `fit_from_coef` | ~120 ns |

平移发生在 10 ms 一次的拟合里，约占该周期的 `6×10⁻⁶`；合并 `translate_u` 和
`translate_v` 最多省一半，换不来可测量的收益，却要把两组精确变换合成一个带
`s·t` 交叉项的大公式，风险大于收益。因此保留两条独立的精确平移。

---

## 8. headless 参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--seconds` | 8 | 仿真时长 |
| `--speed` | 20 | 目标电角速度（rad/s） |
| `--current-noise` | — | 电流采样噪声标准差（A） |
| `--r-scale` | 1.0 | `R` 初值相对铭牌的比例 |
| `--observe-only` | — | 观测器只旁观，电流环用传感器角 |
| `--handover` | 0 | 先传感器闭环，到该时刻再交班 |
| `--speed2/--speed3` + `--*-time` | — | 速度阶跃（复现加速/减速） |
| `--load` | — | 静摩擦转矩 |
| `--tau` | 1.0 | 矩遗忘时间常数（s） |
| `--fit-period` | 0.01 | 圆拟合周期（s） |
| `--obs-min` | 5e-3 | 可观性权重尺度 |
| `--k-prior` | 0.02 | 随机游走先验（直接观测器默认 0） |
| `--k-step` | 0.02 | 信任域 `|δk| ≤ k_step·(1+|k|)` |
| `--max-sigma-rel` | 0.02 | 噪声可信度权重尺度 |
| `--k-sigma` | 0 | 可选显著性硬门限（以 `σ_k` 为单位） |
| `--hard-sigma` | 关 | 可选的 `sigma_rel` 硬门限 |
| `--verbose` | — | 每 5% 时长打印一次跟踪 |

GUI：`cargo run --release --example example1`，`use_observer` 滑块选
`direct_flux_observer`（12）。

---

## 9. 局限

1. **`R` 只在瞬态可辨识**：恒 `iq`/`ω` 时 `P(k) ≡ 0`，稳态必须靠 `obs` 权重
   冻结；10 mA 噪声下仍会漂。
2. **角度噪声就是原始反正切噪声**：没有 PLL 的低通，10 mA 下 ~17° 是物理
   SNR 限制，不是拟合问题。
3. **V 的斜坡去不掉**：V 原点管理只搬常数偏移；电流采样直流偏置积分出的
   斜坡仍会改变 `P(k)`，需要前端去偏置或带泄漏积分。
4. **斜率仍是 ODE 合成**：功率桥是平均模型，没有真实 PWM 双采样，这一局限
   继承自 `example1`。
