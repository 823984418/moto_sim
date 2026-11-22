# MotoSim

电机控制仿真程序（Rust + eframe）。

## 观测器

磁链圆观测器只有一个：`DirectFluxObserver`
（`src/simulation/controller/observer/direct_flux_observer.rs`）。

- 没有 PLL：电流环频率上维护 `∫s0`、`∫i`，直接 `atan2` 测角；
- 圆拟合每 10 ms 跑一次、每次恰好一步 Newton，负责 `R`、圆心和半径；
- 拟合核心（36 个指数遗忘矩、变量投影、半径伪观测）也在这个文件里。

旧的 `flux_circle_observer`（窗口批量拟合 + PLL）和 `kasa_flux_observer`
（流式拟合 + PLL）已删除。

### 高阻（开路）观测器

`HighZFluxObserver`
（`src/simulation/controller/observer/high_z_flux_observer.rs`）是独立实现：
高阻态下电流恒为 0；硬件在每个载波周期里用约 20% 的下管零矢量窗口采样
电流斜率，磁链由斜率积分 `p = ∫s0 dt / unit_len` 得到。所以没有 `∫i`、
没有 `R`/Newton，只做一次二维 Kåsa 圆拟合。

- 斜率积分器带泄漏 `hp_rel`（时间常数 `hp_rel·tau`，默认 200），用来抑制斜率
  直流偏置造成的积分漂移；
- 半径可选：`use_prior_radius = true` 用先验 `radius_prior`；否则用拟合半径并
  夹在 `[radius_min, radius_max]`；
- GUI 里 `use_observer = 13` 选中它时：电流强制为 0，斜率噪声按 20% 窗口
  （`slope_dt = 0.2/carrier − 3 µs`）注入，并**重置**该观测器的积分器/矩/
  圆心；旁边的“高阻：先验半径”复选框切换半径策略。磁链圆图随选择切换：
  13 时显示斜率积分 `p`（固定帧）、拟合圆心和参考圆，其他观测器时仍显示
  直接观测器的磁链圆。

## 运行

```powershell
# GUI
cargo run --release --example example1

# 无界面：被动观测
cargo run --release --example example1 -- --headless --observe-only `
  --seconds 8 --speed 100 --current-noise 0.001

# 闭环（use_observer = 12，直接观测器）
cargo run --release --example example1 -- --headless --seconds 8 --speed 20
```

测试：

```powershell
cargo test --release --lib
```

## 文档

- [`direct_flux/paper.md`](direct_flux/paper.md)：算法设计与推导；
- [`direct_flux/REPORT.md`](direct_flux/REPORT.md)：实现、测试、实测与参数。
