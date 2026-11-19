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
