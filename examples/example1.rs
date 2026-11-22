use eframe::{App, CreationContext, Frame, NativeOptions};
use egui::{CentralPanel, CollapsingHeader, ScrollArea, Slider, Ui, Widget};
use egui_plot::{Legend, Line, Plot, PlotPoint, Points, default_label_formatter};
use moto_sim::model::fan::{FAN_LOAD, FAN_MOTOR};
use moto_sim::model::zq10y_dead_mapping;
use moto_sim::simulation::controller::current_regulator::three_phase_pi_current_regulator::ThreePhasePICurrentRegulator;
use moto_sim::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use moto_sim::simulation::controller::inline_tune::{InductanceTune, ResTune};
use moto_sim::simulation::controller::observer::any_observer::{AllObserver, AnyObserver};
use moto_sim::simulation::controller::observer::base_flux_observer::BaseFluxObserver;
use moto_sim::simulation::controller::observer::direct_flux_observer::DirectFluxObserver;
use moto_sim::simulation::controller::observer::ds_observer::DsObserver;
use moto_sim::simulation::controller::observer::ex_flux_observer::ExFluxObserver;
use moto_sim::simulation::controller::observer::flux_observer::FluxObserver;
use moto_sim::simulation::controller::observer::grad_observer::GradObserver;
use moto_sim::simulation::controller::observer::high_z_flux_observer::HighZFluxObserver;
use moto_sim::simulation::controller::observer::mix_observer::MixObserver;
use moto_sim::simulation::controller::observer::mp_observer::MpObserver;
use moto_sim::simulation::controller::observer::sensor_observer::SensorObserver;
use moto_sim::simulation::controller::observer::ss_observer::{SSObserver, SSample};
use moto_sim::simulation::controller::observer::ss2_observer::{S2Sample, SS2Observer};
use moto_sim::simulation::controller::observer::super_flux_observer::SuperFluxObserver;
use moto_sim::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use moto_sim::simulation::motion_load::ideal_motion_load::IdealMotionLoad;
use moto_sim::simulation::motion_load::{MotionLoad, MotionLoadInput, MotionLoadOutput};
use moto_sim::simulation::motor::pmsm::PermanentMagnetSynchronousMotor;
use moto_sim::simulation::motor::{Motor, MotorInput, MotorOutput};
use moto_sim::simulation::noise::Noise;
use moto_sim::simulation::power_bridge::two_level_power_bridge::TwoLevelPowerBridge;
use moto_sim::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};
use moto_sim::simulation::{angle_normal, clarke, inverse_clarke, mtpa_id, nn, rotate};
use moto_sim::ui::font::load_chinese_font;
use moto_sim::util::{BoxedFunction, LinearInterpolation, Timer};
use rand::SeedableRng;
use rand::rngs::StdRng;

/// GUI 里高阻观测器的编号（滑块 0..=13 的最后一项）。
const HIGH_Z_OBSERVER: usize = 13;

/// 斜率采样两个电流采样点的默认低通带宽（Hz），GUI 可调。
const SLOPE_LPF_DEFAULT_HZ: f64 = 2200.0;

pub fn zq10y_dead_mapping_with_error() -> BoxedFunction {
    let iq14_base: f64 = 2.0f64.powi(-14);
    LinearInterpolation::new(vec![
        (0.0, 0.0),
        (0.007, 255e-5 * iq14_base),
        (-0.007, -255e-5 * iq14_base),
        (0.07, 990e-5 * iq14_base),
        (-0.07, -990e-5 * iq14_base),
        (0.14, 1227e-5 * iq14_base),
        (-0.14, -1227e-5 * iq14_base),
        (0.28, 1703e-5 * iq14_base),
        (-0.28, -1703e-5 * iq14_base),
        (0.42, 1850e-5 * iq14_base),
        (-0.42, -1850e-5 * iq14_base),
        (0.56, 1874e-5 * iq14_base),
        (-0.56, -1874e-5 * iq14_base),
        (1.0, 1900e-5 * iq14_base),
        (-1.0, -1900e-5 * iq14_base),
        (2.0, 1900e-5 * iq14_base),
        (-2.0, -1900e-5 * iq14_base),
    ])
    .into()
}

#[derive(Debug, Default, Clone)]
pub struct Simulation {
    pub delta_time: f64,
    pub timer: Timer,
    pub cmd_speed: f64,
    pub use_observer: usize,

    angle_sensor_noise: Noise,
    current_sample_noise: [Noise; 3],
    /// 斜率采样两个时刻各自的电流采样噪声（单位正态，使用时乘 σ_i）。
    slope_sample_noise_a: [Noise; 3],
    slope_sample_noise_b: [Noise; 3],
    /// 两个采样点各自的固定一阶低通状态（3 相）。
    slope_lpf_a: [f64; 3],
    slope_lpf_b: [f64; 3],
    /// 用于斜率分母的 dt 也过同一个带宽的滤波器。
    slope_dt_lpf: f64,
    slope_lpf_ready: bool,
    output_voltage_noise: [Noise; 3],
    bus_voltage_sample_noise: Noise,
    sample_current_offset: [f64; 3],
    output_voltage_offset: [f64; 3],

    /// 零矢量斜率噪声倍率（0 = 理想斜率，1 = 与电流采样噪声一致）。
    pub slope_noise_scale: f64,
    /// 是否把功率桥死区电压注入零矢量斜率。
    pub slope_dead_voltage: bool,
    /// 注入斜率的死区电压倍率（便于做 1 V / 5 V 的敏感性对比）。
    pub slope_dead_scale: f64,
    /// 电流采样噪声的标准差（A），用于由 Δt 反推斜率噪声。
    pub current_noise_std: f64,
    /// 最近一次传给观测器的零矢量采样间隔（秒），0 表示无有效零矢量。
    pub last_slope_dt: f64,
    /// 高阻模式每个载波周期导通下管的比例（采样斜率/给自举电容充电）。
    ///
    /// 高阻时把它给到零矢量窗口：`slope_dt = high_z_duty/carry_freq - 3 µs`。
    pub high_z_duty: f64,
    /// 斜率采样两个电流采样点的一阶低通带宽（Hz）。
    ///
    /// 两个采样点用同一个带宽；每控制拍 `α = 1 - exp(-2π·f·Δt)`，忽略两点
    /// 之间 dt 造成的相位差。GUI 可调。
    pub slope_lpf_hz: f64,
    /// GUI 演示用：把电流采样噪声切到 1 mA（PAPER.md 的假设值）。
    pub low_noise: bool,
    /// GUI 演示用：闭环时把速度环降慢，避免与观测器的相位滞后耦合。
    pub slow_speed_loop: bool,

    pub target_speed: f64,
    /// 速度 PI 的比例/积分增益（A/(rad/s)）。原值 0.1；闭环磁链观测器
    /// 需要更慢的速度环，避免与 PLL 相位滞后耦合。
    pub speed_kp: f64,
    pub speed_ki: f64,
    /// 速度指令斜坡速率（电角速度 rad/s²）。1000 是原 GUI 行为；
    /// 低速无传感器实验可取 5–20，模拟 PAPER.md §7 的慢速启动。
    pub speed_ramp_rate: f64,
    speed_i: f64,
    set_id: bool,
    id: f64,

    power_bridge_input: PowerBridgeInput<3>,
    power_bridge_output: PowerBridgeOutput<3>,
    motor_input: MotorInput<3>,
    motor_output: MotorOutput<3>,
    motion_load_input: MotionLoadInput,
    motion_load_output: MotionLoadOutput,
    observer_input: ObserverInput<3>,
    sensor_observer_output: ObserverOutput,
    grad_observer_output: ObserverOutput,
    flux_observer_output: ObserverOutput,
    mix_observer_output: ObserverOutput,
    mp_observer_output: ObserverOutput,
    ds_observer_output: ObserverOutput,
    ex_flux_observer_output: ObserverOutput,
    base_flux_observer_output: ObserverOutput,
    ss_observer_output: ObserverOutput,
    super_flux_observer_output: ObserverOutput,
    ss2_observer_output: ObserverOutput,
    all_observer_output: ObserverOutput,
    pub direct_flux_observer_output: ObserverOutput,
    pub high_z_flux_observer_output: ObserverOutput,
    /// 上一次的观测器选择，用于“切到高阻观测器时清零其状态”。
    prev_use_observer: usize,

    current_regulator_input: CurrentRegulatorInput<3>,
    current_regulator_output: CurrentRegulatorOutput<3>,

    dead_fix: BoxedFunction,
    res_tune: ResTune,
    inductance_tune: InductanceTune,

    pub power_bridge: TwoLevelPowerBridge<3>,
    pub motor: PermanentMagnetSynchronousMotor,
    pub motion_load: IdealMotionLoad,
    pub sensor_observer: SensorObserver,
    pub grad_observer: GradObserver,
    pub flux_observer: FluxObserver,
    pub mix_observer: MixObserver,
    pub mp_observer: MpObserver,
    pub ds_observer: DsObserver,
    pub ex_flux_observer: ExFluxObserver,
    pub base_flux_observer: BaseFluxObserver,
    pub ss_observer: SSObserver,
    pub super_flux_observer: SuperFluxObserver,
    pub ss2_observer: SS2Observer,
    pub all_observer: AllObserver,
    pub direct_flux_observer: DirectFluxObserver,
    pub high_z_flux_observer: HighZFluxObserver,

    pub current_regulator: ThreePhasePICurrentRegulator,
}

impl Simulation {
    pub fn new() -> Self {
        let motor = PermanentMagnetSynchronousMotor {
            // inductance_dq: [9.9e-3, 9.9e-3],
            ..FAN_MOTOR
        };
        let sensor_observer = SensorObserver {
            pole_pairs: motor.pole_pairs,
            pll_kp: 10.0,
            pll_ki: 1000.0,
            ..Default::default()
        };
        let flux_observer = FluxObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            speed_lp_factor: 10.0,
            position_factor: 10.0,
            position: [
                f64::cos(std::f64::consts::PI * 0.0),
                f64::sin(std::f64::consts::PI * 0.0),
            ],
            ..Default::default()
        };
        let mix_observer = MixObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            theta_error_kp: 1.0,
            theta_error_ki: 10.0,
            speed_error_kp: 0.0,
            speed_error_ki: 1000.0,
            speed_lp_factor: 10.0,
            target_speed_kp: 0.0,
            target_speed_ki: 0.1,
            ..Default::default()
        };
        let grad_observer = GradObserver {
            rs: 1.0 * motor.rs,
            l0: 1.0 * (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            l1: (motor.inductance_dq[0] - motor.inductance_dq[1]) * 0.5,
            flux: 1.0 * motor.flux,
            kr: 0.1 * 0.0,
            kl0: 0.0001 * 0.0,
            kflux: 0.001 * 0.0,
            kl1_ds: 0.0,
            kl1_de: 0.0,
            kspeed_l1: 1000.0,
            kspeed_flux: 1000.0,
            kspeed_kangle: 1000.0,
            kangle_l1_di: 1.0,
            kangle_l1_ds: 1.0,
            kangle_flux: 1.0,
            speed_lp_factor: 10.0,
            angle: std::f64::consts::PI * 0.1,
            ..Default::default()
        };
        let mp_observer = MpObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            sample: vec![1.0; 1024],
            background: 0.00001,
            error_factor: 0.1,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        let ds_observer = DsObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            pll_angle_kp: 10.0,
            pll_angle_ki: 100.0,
            pll_speed_ki: 0.0,
            sync_angle_lp_factor: 100.0,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        let ex_flux_observer = ExFluxObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            lambda: rotate([motor.flux, 0.0], std::f64::consts::TAU / 360.0 * -45.0),
            alpha: 20.0,
            gamma: 100.0,
            epsilon: 0.00000001,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        let base_flux_observer = BaseFluxObserver {
            rs: motor.rs * 1.1,
            inductance: motor.inductance_dq[1],
            lambda: rotate([motor.flux, 0.0], std::f64::consts::TAU / 360.0 * -45.0),
            alpha: 20.0,
            gamma: 100.0,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        let mut ss_observer = SSObserver {
            error_factor: 0.001,
            rng: StdRng::seed_from_u64(10),
            gen_rs: 1.0 * motor.rs,
            gen_l0: 1.0 * (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            gen_l1: 1.0 * (motor.inductance_dq[1] - motor.inductance_dq[0]) * 0.5,
            gen_flux: 1.0 * motor.flux,
            gen_angle: 1.0,
            target_sample_count: 1024,
            min_power: 0.00001,
            gen_power: 0.1,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        ss_observer.add_sample(SSample {
            power: 1.0,
            rs: motor.rs,
            l0: (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            l1: (motor.inductance_dq[0] - motor.inductance_dq[1]) * 0.5,
            flux: motor.flux,
            angle: 0.0,
        });
        ss_observer.rs_cell_inv = 1e2 / motor.rs;
        ss_observer.l0_cell_inv = 1e2 / motor.inductance_dq[1];
        ss_observer.l1_cell_inv = 1e2 / motor.inductance_dq[1];
        ss_observer.flux_cell_inv = 1e2 / motor.flux;
        ss_observer.angle_cell_inv = std::f64::consts::TAU * 1e2;

        let mut super_flux_observer = SuperFluxObserver {
            alpha: 20.0,
            gamma: 100.0,
            power_alpha: 0.001,
            speed_lp_factor: 10.0,
            ..Default::default()
        };
        for r in 0..10 {
            let rs = r as f64 * 1.0;
            super_flux_observer.add_sample(rs, 9.9e-3);
        }

        let mut ss2_observer = SS2Observer {
            error_factor: 0.1,
            target_sample_count: 4096,
            speed_lp_factor: 10.0,
            dist_rs: 0.01,
            dist_l0: 0.01,
            dist_l1: 0.0,
            dist_flux: 0.0,
            dist_angle: 0.01,
            ..Default::default()
        };
        ss2_observer.samples.push(S2Sample {
            power: 0.0,
            rs: 0.8 * motor.rs,
            l0: 1.0 * (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            l1: 1.0 * (motor.inductance_dq[0] - motor.inductance_dq[1]) * 0.5,
            flux: 1.0 * motor.flux,
            angle: 0.0,
        });

        let mut all_observer = AllObserver {
            time: 0.0,
            delta_time: 0.001,
            is: [0.0; 2],
            us: [0.0; 2],
            any_observer: AnyObserver::new(),
            angle: 0.0,
            speed_lp: 0.0,
            speed_lp_factor: 10.0,
        };
        all_observer.any_observer.history = 5000;
        all_observer.any_observer.resistor = 3.8;
        all_observer.any_observer.resistor_rate = 0.0;
        all_observer.any_observer.inductor = 9.9e-3;
        all_observer.any_observer.inductor_rate = 0.0;

        // 直接磁链积分观测器（无 PLL，快速环反正切 + 10 ms 圆拟合）。
        // 初始角度取本仿真已知的转子位置（等价于启动前的 ALIGN），R 与半径取
        // 电机铭牌值。拟合参数必须在 `set_initial` 之前写入：`Tracker` 是在
        // `set_initial` 里用这份配置构造的。
        let mean_inductance = (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5;
        let nominal_r = motor.rs / mean_inductance;
        let nominal_radius = motor.flux / mean_inductance;
        let initial_electrical_angle = angle_normal(std::f64::consts::PI * 0.5 * motor.pole_pairs);
        let mut direct_flux_observer = DirectFluxObserver::new();
        direct_flux_observer.fit.tau = 1.0;
        direct_flux_observer.fit.hp_rel = 2.0;
        // 解耦 PLL：type-2，50 Hz；GUI 里可关/可调带宽。
        direct_flux_observer.pll_enable = true;
        direct_flux_observer.pll_wn = std::f64::consts::TAU * 50.0;
        direct_flux_observer.set_initial(
            initial_electrical_angle,
            0.0,
            nominal_r * 0.5,
            nominal_radius,
        );

        // 高阻观测器：内部长度单位取额定半径 ψf/L，先验半径 = 1；电流恒为 0，
        // 只积分零矢量斜率并做二维 Kåsa 圆拟合。
        let mut high_z_flux_observer = HighZFluxObserver::new(nominal_radius, 1.0);
        high_z_flux_observer.tau = 1.0;
        high_z_flux_observer.hp_rel = 2.0;

        Self {
            delta_time: 0.0001,
            timer: Timer::new(vec![1e-7, 1.0 / 20e3]),
            target_speed: 0.0,
            speed_kp: 0.1,
            speed_ki: 0.1,
            speed_ramp_rate: 1000.0,
            angle_sensor_noise: Noise::new(0.01, 0),
            current_sample_noise: [
                Noise::new(0.01, 1),
                Noise::new(0.01, 2),
                Noise::new(0.01, 3),
            ],
            slope_sample_noise_a: [
                Noise::new(1.0, 21),
                Noise::new(1.0, 22),
                Noise::new(1.0, 23),
            ],
            slope_sample_noise_b: [
                Noise::new(1.0, 24),
                Noise::new(1.0, 25),
                Noise::new(1.0, 26),
            ],
            slope_lpf_a: [0.0; 3],
            slope_lpf_b: [0.0; 3],
            slope_dt_lpf: 0.0,
            slope_lpf_ready: false,
            slope_noise_scale: 1.0,
            // 零矢量斜率是 v=0 时的 di/dt，死区/器件压降不进入这一测量，
            // 因此默认不注入。保留开关仅用于对照“平均模型误解”的敏感性。
            slope_dead_voltage: false,
            slope_dead_scale: 1.0,
            current_noise_std: 0.01,
            last_slope_dt: 0.0,
            high_z_duty: 0.2,
            slope_lpf_hz: SLOPE_LPF_DEFAULT_HZ,
            low_noise: false,
            slow_speed_loop: false,
            output_voltage_noise: [
                Noise::new(0.01, 4),
                Noise::new(0.01, 5),
                Noise::new(0.01, 6),
            ],
            bus_voltage_sample_noise: Noise::new(1.0, 7),
            // sample_current_offset: [0.001, 0.002, 0.003],
            // output_voltage_offset: [0.02, -0.01, 0.01],
            motion_load: IdealMotionLoad {
                angle: std::f64::consts::PI * 0.5,
                ..FAN_LOAD
            },
            power_bridge: TwoLevelPowerBridge {
                bus_capacitance: 390e-6,
                dead_mapping: zq10y_dead_mapping(),
                carry_freq: 16e3,
                ..Default::default()
            },
            res_tune: ResTune {
                output_res: motor.rs,
                input_lp_factor: 0.1,
                output_lp_factor: 0.1,
                ..Default::default()
            },
            inductance_tune: InductanceTune {
                sample_lp_factor: 5000.0,
                input_lp_factor: 500.0,
                output_lp_factor: 0.001,
                ..Default::default()
            },
            sensor_observer,
            grad_observer,
            flux_observer,
            mix_observer,
            mp_observer,
            ds_observer,
            ex_flux_observer,
            base_flux_observer,
            ss_observer,
            super_flux_observer,
            ss2_observer,
            all_observer,
            direct_flux_observer,
            high_z_flux_observer,
            high_z_flux_observer_output: ObserverOutput::default(),
            prev_use_observer: 0,
            dead_fix: zq10y_dead_mapping(),
            current_regulator: ThreePhasePICurrentRegulator {
                kp: [
                    0.8e4 * motor.inductance_dq[0],
                    0.8e4 * motor.inductance_dq[1],
                ],
                ki: [0.8e4 * motor.rs, 0.8e4 * motor.rs],
                ..Default::default()
            },
            motor,
            ..Default::default()
        }
    }

    /// 重建电流采样噪声。`current_noise_std` 同时决定零矢量斜率噪声的尺度。
    pub fn set_current_noise(&mut self, std: f64) {
        self.current_sample_noise = [Noise::new(std, 1), Noise::new(std, 2), Noise::new(std, 3)];
        self.current_noise_std = std;
    }

    pub fn update(&mut self) {
        // 切换观测器：切到高阻观测器时清空它的积分器/矩/圆心，从当前斜率重新
        // 积分。其它观测器是持续运行的，不需要在这里动。
        if self.use_observer != self.prev_use_observer {
            if self.use_observer == HIGH_Z_OBSERVER {
                self.high_z_flux_observer.reset();
                self.high_z_flux_observer_output = ObserverOutput::default();
            }
            self.prev_use_observer = self.use_observer;
        }
        self.timer.time += self.delta_time;
        while let Some(i) = self.timer.check_time() {
            let delta_time = self.timer.event_step[i];
            match i {
                0 => {
                    self.power_bridge.power_input =
                        310.0 * f64::sin(50.0 * std::f64::consts::TAU * self.timer.time);
                    self.angle_sensor_noise.update(delta_time);
                    for i in 0..3 {
                        self.current_sample_noise[i].update(delta_time);
                        self.output_voltage_noise[i].update(delta_time);
                    }
                    self.bus_voltage_sample_noise.update(delta_time);

                    self.motor_input.speed = self.motion_load_output.speed;
                    self.motor_input.angle = self.motion_load_output.angle;
                    self.motor_input.voltage = self.power_bridge_output.output_voltage;
                    if self.use_observer == HIGH_Z_OBSERVER {
                        // 高阻：三相开路，电流强制为 0，端电压 = 反电势。
                        let theta_e = self.motion_load_output.angle * self.motor.pole_pairs;
                        let omega_e = self.motion_load_output.speed * self.motor.pole_pairs;
                        let e_abc =
                            inverse_clarke(rotate([0.0, omega_e * self.motor.flux], theta_e));
                        self.motor.current_dq = [0.0, 0.0];
                        self.motor_input.voltage = e_abc;
                    }
                    self.motion_load_input.torque = self.motor_output.torque;
                    self.power_bridge_input.output_current = self.motor_output.current;

                    self.power_bridge_output = self
                        .power_bridge
                        .update(delta_time, &self.power_bridge_input);
                    for i in 0..3 {
                        self.power_bridge_output.output_voltage[i] +=
                            self.output_voltage_offset[i] + self.output_voltage_noise[i].value;
                    }
                    self.motor_output = self.motor.update(delta_time, &self.motor_input);
                    self.motion_load_output =
                        self.motion_load.update(delta_time, &self.motion_load_input);
                }
                1 => {
                    if self.cmd_speed > self.target_speed {
                        self.target_speed += self.speed_ramp_rate * delta_time;
                        if self.target_speed > self.cmd_speed {
                            self.target_speed = self.cmd_speed;
                        }
                    }
                    if self.cmd_speed < self.target_speed {
                        self.target_speed -= self.speed_ramp_rate * delta_time;
                        if self.target_speed < self.cmd_speed {
                            self.target_speed = self.cmd_speed;
                        }
                    }

                    self.sensor_observer.sensor_angle =
                        self.motion_load.angle + self.angle_sensor_noise.value;

                    let sample_bus_voltage =
                        self.power_bridge_output.bus_voltage + self.bus_voltage_sample_noise.value;

                    if sample_bus_voltage > 380.0 {
                        self.power_bridge.braking_current = 3.0;
                    }
                    if sample_bus_voltage < 350.0 {
                        self.power_bridge.braking_current = 0.01;
                    }

                    // 电流重建
                    let mut sample_current = self.motor_output.current;
                    for i in 0..3 {
                        sample_current[i] +=
                            self.sample_current_offset[i] + self.current_sample_noise[i].value;
                    }
                    // 电压重建
                    let mut sample_voltage = self
                        .power_bridge_input
                        .output_duty
                        .map(|i| i * 0.5 * sample_bus_voltage);
                    for i in 0..3 {
                        sample_voltage[i] += self.dead_fix.eval(sample_current[i])
                            * sample_bus_voltage
                            * self.power_bridge.carry_freq
                            * self.power_bridge.dead_scale;
                    }

                    self.observer_input.voltage = sample_voltage;
                    self.observer_input.current = sample_current;
                    self.observer_input.target_speed = self.target_speed;
                    if self.use_observer == HIGH_Z_OBSERVER {
                        // 高阻态端电压的物理值是反电势（高阻观测器本身不用电压，
                        // 它积分的是下面零矢量窗口的电流斜率；这里保留给其它
                        // 旁观观测器和电压 scope）。
                        let theta_e = self.motion_load_output.angle * self.motor.pole_pairs;
                        let omega_e = self.motion_load_output.speed * self.motor.pole_pairs;
                        let mut voltage =
                            inverse_clarke(rotate([0.0, omega_e * self.motor.flux], theta_e));
                        for i in 0..3 {
                            voltage[i] +=
                                self.output_voltage_offset[i] + self.output_voltage_noise[i].value;
                        }
                        self.observer_input.voltage = voltage;
                        // 电流仍取采样值（0 + 偏置/噪声）。
                        self.observer_input.current = sample_current;
                    }

                    // ---- 零矢量电流斜率传感器模型 ----
                    // 本仿真的功率桥是平均模型，没有真实载波/开关状态，所以这里
                    // 用电机 dq ODE 在 v=0 时的导数合成“双采样斜率”，再按零矢量
                    // 时长注入噪声。它不是从真实 PWM 波形里测出来的。
                    let duty = self.power_bridge_input.output_duty;
                    let duty_max = duty.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    let duty_min = duty.iter().copied().fold(f64::INFINITY, f64::min);
                    // 高阻模式硬件：每个载波约 20% 时间导通下管采样斜率，其余高阻。
                    // 这里不做载波级波形，只把斜率窗口设为 20% 来产生相应的噪声。
                    let zero_vector_fraction = if self.use_observer == HIGH_Z_OBSERVER {
                        self.high_z_duty.clamp(0.01, 1.0)
                    } else {
                        (1.0 - 0.5 * (duty_max - duty_min)).clamp(0.0, 1.0)
                    };
                    let carrier_period = if self.power_bridge.carry_freq > 0.0 {
                        1.0 / self.power_bridge.carry_freq
                    } else {
                        delta_time
                    };
                    let slope_dt = zero_vector_fraction * carrier_period - 3.0e-6;
                    if slope_dt >= 1.0e-6 {
                        let theta_e = self.motion_load.angle * self.motor.pole_pairs;
                        let omega_e = self.motion_load.speed * self.motor.pole_pairs;
                        let [id, iq] = self.motor.current_dq;
                        let ld = self.motor.inductance_dq[0];
                        let lq = self.motor.inductance_dq[1];
                        let l = 0.5 * (ld + lq);

                        // vd = vq = 0 时的 dq 电流导数
                        let did = (-self.motor.rs * id + omega_e * lq * iq) / ld;
                        let diq =
                            (-self.motor.rs * iq - omega_e * (self.motor.flux + ld * id)) / lq;
                        let i_ab = rotate([id, iq], theta_e);
                        let di_ab = rotate([did, diq], theta_e);
                        // αβ 系里还要加上坐标系旋转项 ωJ·i
                        let mut slope_ab =
                            [di_ab[0] - omega_e * i_ab[1], di_ab[1] + omega_e * i_ab[0]];

                        // 零矢量斜率是端电压为零时的 di/dt，死区/器件压降不进入
                        // 这一测量。下面的注入仅作为“错误理解”的对照实验，
                        // 默认关闭（--inject-dead 才打开）。
                        if self.slope_dead_voltage {
                            let dead_ab = clarke(self.power_bridge.dead_voltage);
                            slope_ab[0] += self.slope_dead_scale * dead_ab[0] / l;
                            slope_ab[1] += self.slope_dead_scale * dead_ab[1] / l;
                        }

                        // 两个采样点：t0 和 t0+dt 各采一次电流，各自加独立
                        // 采样噪声，再各自过同一个固定带宽的一阶低通，最后
                        // slope = (i2_f - i1_f)/dt。
                        let i_3ph = inverse_clarke(i_ab);
                        let s_3ph = inverse_clarke(slope_ab);
                        let sigma_i = self.current_noise_std * self.slope_noise_scale;
                        let mut i_a = [0.0; 3];
                        let mut i_b = [0.0; 3];
                        for i in 0..3 {
                            self.slope_sample_noise_a[i].update(delta_time);
                            self.slope_sample_noise_b[i].update(delta_time);
                            i_a[i] = i_3ph[i]
                                + self.slope_sample_noise_a[i].value * sigma_i;
                            i_b[i] = i_3ph[i]
                                + s_3ph[i] * slope_dt
                                + self.slope_sample_noise_b[i].value * sigma_i;
                        }
                        if !self.slope_lpf_ready {
                            self.slope_lpf_a = i_a;
                            self.slope_lpf_b = i_b;
                            self.slope_dt_lpf = slope_dt;
                            self.slope_lpf_ready = true;
                        }
                        let alpha = (1.0
                            - (-std::f64::consts::TAU * self.slope_lpf_hz * delta_time).exp())
                        .clamp(0.0, 1.0);
                        // 分母 dt 也过同一个带宽的滤波器。
                        self.slope_dt_lpf += alpha * (slope_dt - self.slope_dt_lpf);
                        let mut slope_3ph = [0.0; 3];
                        for i in 0..3 {
                            self.slope_lpf_a[i] += alpha * (i_a[i] - self.slope_lpf_a[i]);
                            self.slope_lpf_b[i] += alpha * (i_b[i] - self.slope_lpf_b[i]);
                            slope_3ph[i] = (self.slope_lpf_b[i] - self.slope_lpf_a[i])
                                / self.slope_dt_lpf;
                        }
                        self.observer_input.current_slope = slope_3ph;
                        self.observer_input.current_slope_dt = slope_dt;
                        self.last_slope_dt = slope_dt;
                    } else {
                        // 调制度太高，零矢量不足以做双采样。
                        self.observer_input.current_slope = [0.0; 3];
                        self.observer_input.current_slope_dt = 0.0;
                        self.last_slope_dt = 0.0;
                    }

                    self.res_tune.update(delta_time, &self.observer_input);
                    self.inductance_tune
                        .update(delta_time, &self.observer_input);

                    self.sensor_observer_output = self
                        .sensor_observer
                        .update(delta_time, &self.observer_input);
                    self.grad_observer_output =
                        self.grad_observer.update(delta_time, &self.observer_input);
                    self.flux_observer_output =
                        self.flux_observer.update(delta_time, &self.observer_input);
                    self.mix_observer_output =
                        self.mix_observer.update(delta_time, &self.observer_input);
                    self.mp_observer_output =
                        self.mp_observer.update(delta_time, &self.observer_input);
                    self.ds_observer_output =
                        self.ds_observer.update(delta_time, &self.observer_input);
                    self.ex_flux_observer_output = self
                        .ex_flux_observer
                        .update(delta_time, &self.observer_input);
                    self.base_flux_observer_output = self
                        .base_flux_observer
                        .update(delta_time, &self.observer_input);
                    self.ss_observer_output =
                        self.ss_observer.update(delta_time, &self.observer_input);
                    self.super_flux_observer_output = self
                        .super_flux_observer
                        .update(delta_time, &self.observer_input);
                    self.ss2_observer_output =
                        self.ss2_observer.update(delta_time, &self.observer_input);
                    self.all_observer_output =
                        self.all_observer.update(delta_time, &self.observer_input);
                    self.direct_flux_observer_output = self
                        .direct_flux_observer
                        .update(delta_time, &self.observer_input);
                    self.high_z_flux_observer_output = self
                        .high_z_flux_observer
                        .update(delta_time, &self.observer_input);

                    let use_observer = match self.use_observer {
                        0 => &self.sensor_observer_output,
                        1 => &self.grad_observer_output,
                        2 => &self.flux_observer_output,
                        3 => &self.mix_observer_output,
                        4 => &self.mp_observer_output,
                        5 => &self.ds_observer_output,
                        6 => &self.ex_flux_observer_output,
                        7 => &self.base_flux_observer_output,
                        8 => &self.ss_observer_output,
                        9 => &self.super_flux_observer_output,
                        10 => &self.ss2_observer_output,
                        11 => &self.all_observer_output,
                        12 => &self.direct_flux_observer_output,
                        13 => &self.high_z_flux_observer_output,
                        _ => &self.sensor_observer_output,
                    };
                    self.current_regulator_input.electrical_speed = use_observer.electrical_speed;
                    self.current_regulator_input.electrical_angle = use_observer.electrical_angle;
                    self.current_regulator_input.continuous_speed = use_observer.continuous_speed;
                    self.current_regulator_input.current = sample_current;

                    if self.use_observer == HIGH_Z_OBSERVER {
                        // 高阻态：速度环/电流环不参与，桥臂全关，电流强制为 0
                        // （电机端电压在 plant 事件里被设为反电势）。
                        self.speed_i = 0.0;
                        self.current_regulator_input.command_current = [0.0; 3];
                        self.power_bridge_input.output_duty = [0.0; 3];
                    } else {
                        let max_current = 2.0;
                        let speed_error =
                            self.target_speed - self.current_regulator_input.electrical_speed;
                        let speed_p = speed_error * self.speed_kp;
                        self.speed_i += speed_error * self.speed_ki * delta_time;
                        self.speed_i = self.speed_i.clamp(-max_current, max_current);
                        let output_torque =
                            (self.speed_i + speed_p).clamp(-max_current, max_current);
                        let id = if self.set_id {
                            self.id
                        } else {
                            mtpa_id(
                                output_torque,
                                self.motor.flux,
                                self.motor.inductance_dq[0],
                                self.motor.inductance_dq[1],
                            )
                        }
                        .clamp(-max_current, max_current);
                        let command_current = rotate(
                            [
                                id + 0.0
                                    * f64::cos(
                                        use_observer.electrical_angle - self.res_tune.tune_angle,
                                    ),
                                output_torque,
                            ],
                            use_observer.electrical_angle,
                        );
                        self.current_regulator_input.command_current =
                            inverse_clarke(command_current);
                        self.current_regulator_output = self
                            .current_regulator
                            .update(delta_time, &self.current_regulator_input);

                        let command_voltage = self.current_regulator_output.command_voltage;

                        self.power_bridge_input.output_duty =
                            command_voltage.map(|i| nn(i * 2.0 / sample_bus_voltage));
                    }
                }
                _ => unreachable!(),
            }
        }
    }
}

pub struct ScopeData<S> {
    name: String,
    data: Vec<PlotPoint>,
    update: Box<dyn FnMut(&mut S) -> f64>,
    /// 可选可见性：返回 false 时不采样并清空已有轨迹（切换观测器时用）。
    visible: Option<Box<dyn FnMut(&mut S) -> bool>>,
}

impl<S> ScopeData<S> {
    pub fn new<F: FnMut(&mut S) -> f64 + 'static>(name: &str, f: F) -> Self {
        Self {
            name: name.to_string(),
            data: Vec::new(),
            update: Box::new(f),
            visible: None,
        }
    }

    /// 只在 `f` 为真时采样；为假时清空已有数据，曲线上不留下切换前的残影。
    pub fn visible_if<F: FnMut(&mut S) -> bool + 'static>(mut self, f: F) -> Self {
        self.visible = Some(Box::new(f));
        self
    }
}

pub struct Scope<S> {
    sample_count: usize,
    data: Vec<(String, Vec<ScopeData<S>>)>,
}

impl<S> Scope<S> {
    pub fn new() -> Self {
        Self {
            sample_count: 10000,
            data: Vec::new(),
        }
    }

    pub fn update(&mut self, time: f64, source: &mut S) {
        for (_, datas) in &mut self.data {
            for d in datas {
                if let Some(visible) = &mut d.visible {
                    if !visible(source) {
                        d.data.clear();
                        continue;
                    }
                }
                d.data.push(PlotPoint::new(time, (d.update)(source)));
            }
        }
    }

    pub fn ui(&mut self, ui: &mut Ui) {
        for (_, datas) in &mut self.data {
            for d in datas {
                if d.data.len() > self.sample_count {
                    let begin = d.data.len() - self.sample_count;
                    d.data.copy_within(begin.., 0);
                    d.data.truncate(self.sample_count);
                }
            }
        }
        let mut auto_x = false;
        let mut auto_y = false;
        ui.horizontal(|ui| {
            if ui.button("auto").clicked() {
                auto_x = true;
                auto_y = true;
            }
            if ui.button("auto_x").clicked() {
                auto_x = true;
            }
            if ui.button("auto_y").clicked() {
                auto_y = true;
            }
        });
        ScrollArea::vertical().show(ui, |ui| {
            for (group_name, datas) in &self.data {
                CollapsingHeader::new(group_name).show(ui, |ui| {
                    Plot::new(group_name)
                        .height(120.0)
                        .legend(Legend::default())
                        .label_formatter(default_label_formatter)
                        .show(ui, |ui| {
                            let mut auto_bounds = ui.auto_bounds();
                            auto_bounds.x |= auto_x;
                            auto_bounds.y |= auto_y;
                            ui.set_auto_bounds(auto_bounds);
                            for d in datas {
                                if !d.data.is_empty() {
                                    ui.line(Line::new(&d.name, d.data.as_slice()));
                                }
                            }
                        });
                });
            }
        });
    }
}

pub struct Application {
    simulation: Simulation,
    run: bool,
    scope: Scope<Simulation>,
    /// 快速环磁链向量（相对拟合圆心）的轨迹，供"磁链圆"图使用。
    flux_trace: Vec<PlotPoint>,
    /// 上一次绘图用的观测器编号；切换时清空轨迹，避免两个观测器的点混在一起。
    last_plot_observer: usize,
}

impl Application {
    pub fn new(cc: &CreationContext) -> anyhow::Result<Self> {
        load_chinese_font(&cc.egui_ctx);
        let mut this = Self {
            simulation: Simulation::new(),
            run: false,
            scope: Scope::new(),
            flux_trace: Vec::new(),
            last_plot_observer: 0,
        };
        this.reset();
        Ok(this)
    }

    pub fn reset(&mut self) {
        self.run = false;
        self.flux_trace.clear();
        self.last_plot_observer = 0;
        self.simulation = Simulation::new();
        self.scope.data = vec![
            (
                "angle".to_string(),
                vec![
                    ScopeData::new("real_angle", |s: &mut Simulation| {
                        angle_normal(s.motion_load.angle * s.motor.pole_pairs)
                    }),
                    // ScopeData::new("sensor_angle", |s: &mut Simulation| {
                    //     s.sensor_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("flux_angle", |s: &mut Simulation| {
                    //     s.flux_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("grad_angle", |s: &mut Simulation| {
                    //     s.grad_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("mix_angle", |s: &mut Simulation| {
                    //     s.mix_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("mp_angle", |s: &mut Simulation| {
                    //     s.mp_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("ds_angle", |s: &mut Simulation| {
                    //     s.ds_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("ex_flux_angle", |s: &mut Simulation| {
                    //     s.ex_flux_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("base_flux_angle", |s: &mut Simulation| {
                    //     s.base_flux_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("ss_angle", |s: &mut Simulation| {
                    //     s.ss_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("super_flux_angle", |s: &mut Simulation| {
                    //     s.super_flux_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("ss2_angle", |s: &mut Simulation| {
                    //     s.ss2_observer_output.electrical_angle
                    // }),
                    // ScopeData::new("all_angle", |s: &mut Simulation| {
                    //     s.all_observer_output.electrical_angle
                    // }),
                    ScopeData::new("direct_angle", |s: &mut Simulation| {
                        s.direct_flux_observer_output.electrical_angle
                    }),
                    ScopeData::new("high_z_angle", |s: &mut Simulation| {
                        s.high_z_flux_observer_output.electrical_angle
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                ],
            ),
            (
                "angle_error".to_string(),
                vec![
                    ScopeData::new("sensor_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.sensor_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("flux_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.flux_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("grad_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.grad_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("mix_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.mix_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    // ScopeData::new("mp_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.mp_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("ds_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.ds_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("ex_flux_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.ex_flux_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("base_flux_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.base_flux_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("ss_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.ss_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("super_flux_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.super_flux_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("ss2_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.ss2_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("all_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.all_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // 直接磁链：输出角误差与未滤波的反正切角误差（无 PLL）。
                    ScopeData::new("direct_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.direct_flux_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("direct_meas", |s: &mut Simulation| {
                        angle_normal(
                            s.direct_flux_observer.theta_meas
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("high_z_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.high_z_flux_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                ],
            ),
            (
                "speed".to_string(),
                vec![
                    ScopeData::new("real_speed", |s: &mut Simulation| {
                        s.motion_load.speed * s.motor.pole_pairs
                    }),
                    // ScopeData::new("sensor_speed", |s: &mut Simulation| {
                    //     s.sensor_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("flux_speed", |s: &mut Simulation| {
                    //     s.flux_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("grad_speed", |s: &mut Simulation| {
                    //     s.grad_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("mix_speed", |s: &mut Simulation| {
                    //     s.mix_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("mp_speed", |s: &mut Simulation| {
                    //     s.mp_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("ds_speed", |s: &mut Simulation| {
                    //     s.ds_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("ex_flux_speed", |s: &mut Simulation| {
                    //     s.ex_flux_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("base_flux_speed", |s: &mut Simulation| {
                    //     s.base_flux_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("ss_speed", |s: &mut Simulation| {
                    //     s.ss_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("super_flux_speed", |s: &mut Simulation| {
                    //     s.super_flux_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("ss2_speed", |s: &mut Simulation| {
                    //     s.ss2_observer_output.electrical_speed
                    // }),
                    // ScopeData::new("all_speed", |s: &mut Simulation| {
                    //     s.all_observer_output.electrical_speed
                    // }),
                    ScopeData::new("direct_speed", |s: &mut Simulation| {
                        s.direct_flux_observer_output.electrical_speed
                    }),
                    ScopeData::new("high_z_speed", |s: &mut Simulation| {
                        s.high_z_flux_observer_output.electrical_speed
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                ],
            ),
            (
                "idq".to_string(),
                vec![
                    ScopeData::new("id", |s: &mut Simulation| s.motor.current_dq[0]),
                    ScopeData::new("iq", |s: &mut Simulation| s.motor.current_dq[1]),
                ],
            ),
            (
                "udq".to_string(),
                vec![
                    ScopeData::new("ud", |s: &mut Simulation| {
                        rotate(
                            clarke(s.power_bridge_output.output_voltage),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[0]
                    }),
                    ScopeData::new("uq", |s: &mut Simulation| {
                        rotate(
                            clarke(s.power_bridge_output.output_voltage),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[1]
                    }),
                    ScopeData::new("sud", |s: &mut Simulation| {
                        rotate(
                            clarke(s.power_bridge_output.output_voltage),
                            -s.current_regulator_input.electrical_angle,
                        )[0]
                    }),
                    ScopeData::new("suq", |s: &mut Simulation| {
                        rotate(
                            clarke(s.power_bridge_output.output_voltage),
                            -s.current_regulator_input.electrical_angle,
                        )[1]
                    }),
                ],
            ),
            (
                "output_torque".to_string(),
                vec![ScopeData::new("torque", |s: &mut Simulation| {
                    s.motor_output.torque
                })],
            ),
            (
                "bus_voltage".to_string(),
                vec![ScopeData::new("bus", |s: &mut Simulation| {
                    s.power_bridge.bus_voltage
                })],
            ),
            (
                "dead_voltage".to_string(),
                vec![
                    ScopeData::new("u", |s: &mut Simulation| s.power_bridge.dead_voltage[0]),
                    ScopeData::new("v", |s: &mut Simulation| s.power_bridge.dead_voltage[1]),
                    ScopeData::new("w", |s: &mut Simulation| s.power_bridge.dead_voltage[2]),
                ],
            ),
            (
                "sidq".to_string(),
                vec![
                    ScopeData::new("sid", |s: &mut Simulation| {
                        rotate(
                            clarke(s.current_regulator_input.current),
                            -s.current_regulator_input.electrical_angle,
                        )[0]
                    }),
                    ScopeData::new("siq", |s: &mut Simulation| {
                        rotate(
                            clarke(s.current_regulator_input.current),
                            -s.current_regulator_input.electrical_angle,
                        )[1]
                    }),
                ],
            ),
            // 以下 sdq / ex_flux / ex_fluxx 属于旧观测器（mix_observer、base_flux_observer），
            // 与当前直接磁链工作无关，先注释掉。
            // (
            //     "sdq".to_string(),
            //     vec![
            //         ScopeData::new("sd", |s: &mut Simulation| s.mix_observer.sync_speed_lp[0]),
            //         ScopeData::new("sq", |s: &mut Simulation| s.mix_observer.sync_speed_lp[1]),
            //     ],
            // ),
            // (
            //     "ex_flux".to_string(),
            //     vec![
            //         ScopeData::new("x", |s: &mut Simulation| s.base_flux_observer.x[0]),
            //         ScopeData::new("y", |s: &mut Simulation| s.base_flux_observer.x[1]),
            //     ],
            // ),
            // (
            //     "ex_fluxx".to_string(),
            //     vec![
            //         ScopeData::new("y", |s: &mut Simulation| s.base_flux_observer.y),
            //         ScopeData::new("y_lp", |s: &mut Simulation| s.base_flux_observer.omega_l),
            //     ],
            // ),
            (
                "rs".to_string(),
                vec![
                    ScopeData::new("rs", |s: &mut Simulation| s.motor.rs),
                    // ScopeData::new("grad_rs", |s: &mut Simulation| s.grad_observer.rs),
                    // ScopeData::new("tune_rs", |s: &mut Simulation| s.res_tune.output_res),
                    // ScopeData::new("ss_observer", |s: &mut Simulation| s.ss_observer.rs_est),
                    // ScopeData::new("super_flux_observer", |s: &mut Simulation| {
                    //     s.super_flux_observer.rs
                    // }),
                    // ScopeData::new("ss2_observer", |s: &mut Simulation| s.ss2_observer.rs_est),
                    // ScopeData::new("all_observer", |s: &mut Simulation| {
                    //     s.all_observer.any_observer.resistor
                    // }),
                ],
            ),
            (
                "irs".to_string(),
                vec![ScopeData::new("tune_c", |s: &mut Simulation| {
                    s.res_tune.current_lp[0]
                })],
            ),
            (
                "vrs".to_string(),
                vec![ScopeData::new("tune_v", |s: &mut Simulation| {
                    s.res_tune.voltage_lp[0]
                })],
            ),
            (
                "l0".to_string(),
                vec![
                    ScopeData::new("l0", |s: &mut Simulation| {
                        (s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5
                    }),
                    // ScopeData::new("grad_l0", |s: &mut Simulation| s.grad_observer.l0),
                    // ScopeData::new("tune_l0", |s: &mut Simulation| s.inductance_tune.output_l0),
                    // ScopeData::new("ss_observer", |s: &mut Simulation| s.ss_observer.l0_est),
                    // ScopeData::new("super_flux_observer", |s: &mut Simulation| {
                    //     s.super_flux_observer.inductance
                    // }),
                    // ScopeData::new("ss2_observer", |s: &mut Simulation| s.ss2_observer.l0_est),
                    // ScopeData::new("all_observer", |s: &mut Simulation| {
                    //     s.all_observer.any_observer.inductor
                    // }),
                ],
            ),
            (
                "l1".to_string(),
                vec![
                    ScopeData::new("l1", |s: &mut Simulation| {
                        (s.motor.inductance_dq[0] - s.motor.inductance_dq[1]) * 0.5
                    }),
                    // ScopeData::new("grad_l1", |s: &mut Simulation| s.grad_observer.l1),
                    // ScopeData::new("ss_observer", |s: &mut Simulation| s.ss_observer.l1_est),
                    // ScopeData::new("ss2_observer", |s: &mut Simulation| s.ss2_observer.l1_est),
                ],
            ),
            (
                "flux".to_string(),
                vec![
                    ScopeData::new("flux", |s: &mut Simulation| s.motor.flux),
                    // ScopeData::new("grad_flux", |s: &mut Simulation| s.grad_observer.flux),
                    // ScopeData::new("ss_observer", |s: &mut Simulation| s.ss_observer.flux_est),
                    // ScopeData::new("super_flux", |s: &mut Simulation| {
                    //     s.super_flux_observer.flux
                    // }),
                    // ScopeData::new("ss2_observer", |s: &mut Simulation| s.ss2_observer.flux_est),
                    // ScopeData::new("all_observer", |s: &mut Simulation| {
                    //     s.all_observer.any_observer.flux
                    // }),
                ],
            ),
            (
                "direct_flux".to_string(),
                vec![
                    ScopeData::new("true_R", |s: &mut Simulation| {
                        s.motor.rs / ((s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5)
                    }),
                    ScopeData::new("R", |s: &mut Simulation| s.direct_flux_observer.r),
                    ScopeData::new("R_fit", |s: &mut Simulation| {
                        s.direct_flux_observer.r_fit
                    }),
                    ScopeData::new("radius", |s: &mut Simulation| {
                        s.direct_flux_observer.radius
                    }),
                    ScopeData::new("radius_fit", |s: &mut Simulation| {
                        s.direct_flux_observer.radius_fit
                    }),
                    ScopeData::new("obs", |s: &mut Simulation| s.direct_flux_observer.obs),
                    ScopeData::new("steps", |s: &mut Simulation| {
                        s.direct_flux_observer.steps as f64
                    }),
                ],
            ),
            (
                "direct_flux_mag".to_string(),
                vec![
                    ScopeData::new("flux_mag", |s: &mut Simulation| {
                        s.direct_flux_observer.flux_mag
                    }),
                    ScopeData::new("radius", |s: &mut Simulation| {
                        s.direct_flux_observer.radius
                    }),
                ],
            ),
            (
                "direct_flux_valid".to_string(),
                vec![
                    ScopeData::new("valid", |s: &mut Simulation| {
                        if s.direct_flux_observer.valid {
                            1.0
                        } else {
                            0.0
                        }
                    }),
                    ScopeData::new("weight", |s: &mut Simulation| {
                        s.direct_flux_observer.weight
                    }),
                    ScopeData::new("data_arc", |s: &mut Simulation| {
                        s.direct_flux_observer.data_arc
                    }),
                    ScopeData::new("sigma_rel", |s: &mut Simulation| {
                        s.direct_flux_observer.sigma_rel
                    }),
                    ScopeData::new("sigma_k", |s: &mut Simulation| {
                        s.direct_flux_observer.sigma_k
                    }),
                ],
            ),
            (
                "high_z_flux".to_string(),
                vec![
                    ScopeData::new("radius", |s: &mut Simulation| {
                        s.high_z_flux_observer.radius * s.high_z_flux_observer.unit_len
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                    ScopeData::new("flux_mag", |s: &mut Simulation| {
                        let o = &s.high_z_flux_observer;
                        o.flux().sub(o.center()).norm() * o.unit_len
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                    ScopeData::new("center", |s: &mut Simulation| {
                        let o = &s.high_z_flux_observer;
                        o.center().norm() * o.unit_len
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                    ScopeData::new("center_local", |s: &mut Simulation| {
                        let o = &s.high_z_flux_observer;
                        o.center_local.norm() * o.unit_len
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                    ScopeData::new("weight", |s: &mut Simulation| {
                        s.high_z_flux_observer.weight
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                    ScopeData::new("valid", |s: &mut Simulation| {
                        if s.high_z_flux_observer.valid {
                            1.0
                        } else {
                            0.0
                        }
                    })
                    .visible_if(|s: &mut Simulation| s.use_observer == HIGH_Z_OBSERVER),
                ],
            ),
        ];
    }

    fn update(&mut self) {
        self.simulation.update();
        self.scope
            .update(self.simulation.timer.time, &mut self.simulation);
        // 切换观测器时清空轨迹：两个观测器的磁链不在同一帧里。
        if self.simulation.use_observer != self.last_plot_observer {
            self.last_plot_observer = self.simulation.use_observer;
            self.flux_trace.clear();
        }
        // 磁链圆轨迹：直接观测器用“相对快速环圆心”的 flux；高阻观测器用
        // 积分磁链 p 的绝对值（帧固定为积分起点，避免圆心更新时轨迹跳变）。
        let f = if self.simulation.use_observer == HIGH_Z_OBSERVER {
            let o = &self.simulation.high_z_flux_observer;
            let p = o.flux().mul(o.unit_len);
            [p.x, p.y]
        } else {
            let o = &self.simulation.direct_flux_observer;
            [o.flux.x, o.flux.y]
        };
        if f[0].is_finite() && f[1].is_finite() {
            self.flux_trace.push(PlotPoint::new(f[0], f[1]));
            if self.flux_trace.len() > 4096 {
                self.flux_trace.drain(..1024);
            }
        }
    }

    /// 磁链圆。
    ///
    /// * `direct_flux_observer`：快速环积分出的磁链向量，拟合圆心为原点；
    /// * `high_z_flux_observer`：电压积分 `p`（固定帧），参考圆以拟合圆心
    ///   `center` 为圆心、半径 `radius`（先验或拟合范围）× `unit_len`。
    fn flux_plot(&mut self, ui: &mut Ui) {
        let (radius, current, center_used, circle_center, title) =
            if self.simulation.use_observer == HIGH_Z_OBSERVER {
                let o = &self.simulation.high_z_flux_observer;
                let unit = o.unit_len;
                let p = o.flux().mul(unit);
                let c = o.center().mul(unit);
                (
                    o.radius * unit,
                    [p.x, p.y],
                    [c.x, c.y],
                    [c.x, c.y],
                    "磁链圆（高阻观测器）",
                )
            } else {
                let o = &self.simulation.direct_flux_observer;
                (
                    o.radius,
                    [o.flux.x, o.flux.y],
                    [o.center_used.x, o.center_used.y],
                    [0.0, 0.0],
                    "磁链圆（圆心为原点）",
                )
            };
        // 拟合半径画成参考圆。
        let mut circle: Vec<[f64; 2]> = Vec::new();
        if radius > 0.0 && radius.is_finite() {
            const N: usize = 180;
            for k in 0..=N {
                let a = std::f64::consts::TAU * k as f64 / N as f64;
                circle.push([
                    circle_center[0] + radius * a.cos(),
                    circle_center[1] + radius * a.sin(),
                ]);
            }
        }
        CollapsingHeader::new(title)
            .default_open(true)
            .show(ui, |ui| {
                Plot::new("flux_circle")
                    .height(300.0)
                    .data_aspect(1.0)
                    .legend(Legend::default())
                    .label_formatter(default_label_formatter)
                    .show(ui, |ui| {
                        ui.line(Line::new("flux", self.flux_trace.as_slice()));
                        if !circle.is_empty() {
                            ui.line(Line::new("radius", circle).color(egui::Color32::DARK_GRAY));
                        }
                        ui.points(
                            Points::new("now", current)
                                .radius(4.0)
                                .color(egui::Color32::RED),
                        );
                        // 直接观测器：交班实际使用的圆心（最新样本 + 拟合半径推出）；
                        // 高阻观测器：拟合圆心。
                        ui.points(
                            Points::new("center", center_used)
                                .radius(5.0)
                                .color(egui::Color32::from_rgb(30, 120, 255)),
                        );
                    });
            });
    }
}

impl App for Application {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut Frame) {
        ui.request_repaint();
        if self.run {
            for _ in 0..33 {
                self.update();
            }
        }

        CentralPanel::default().show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Reset").clicked() {
                    self.reset();
                }
                if ui.button("Dump").clicked() {
                    // let max_sample = self
                    //     .simulation
                    //     .ss_observer
                    //     .samples
                    //     .values()
                    //     .max_by(|a, b| f64::total_cmp(&a.power, &b.power));
                    // println!("{:#?}", max_sample);

                    // println!("{:#?}", self.simulation.super_flux_observer);
                    println!("{:#?}", self.simulation.all_observer);
                }
                ui.checkbox(&mut self.run, "Run");
                ui.label("static_friction_torque");
                Slider::new(
                    &mut self.simulation.motion_load.static_friction_torque,
                    0.0..=10.0,
                )
                .logarithmic(true)
                .ui(ui);
                ui.label("cmd_speed");
                Slider::new(&mut self.simulation.cmd_speed, -5000.0..=5000.0).ui(ui);
                ui.checkbox(&mut self.simulation.set_id, "id");
                Slider::new(&mut self.simulation.id, -2.0..=2.0).ui(ui);
            });
            ui.horizontal(|ui| {
                ui.label("use_observer");
                Slider::new(&mut self.simulation.use_observer, 0..=13).ui(ui);
                ui.label(match self.simulation.use_observer {
                    0 => "sensor_observer",
                    1 => "grad_observer",
                    2 => "flux_observer",
                    3 => "mix_observer",
                    4 => "mp_observer",
                    5 => "ds_observer",
                    6 => "ex_flux_observer",
                    7 => "base_flux_observer",
                    8 => "ss_observer",
                    9 => "super_flux_observer",
                    10 => "ss2_observer",
                    11 => "all_observer",
                    12 => "direct_flux_observer",
                    13 => "high_z_flux_observer",
                    _ => "sensor_observer",
                });
                ui.checkbox(
                    &mut self.simulation.high_z_flux_observer.use_prior_radius,
                    "高阻：先验半径",
                );
                ui.label("高阻占空比");
                Slider::new(&mut self.simulation.high_z_duty, 0.05..=0.5).ui(ui);
                if ui
                    .checkbox(&mut self.simulation.low_noise, "1 mA 噪声")
                    .changed()
                {
                    let std = if self.simulation.low_noise {
                        0.001
                    } else {
                        0.01
                    };
                    self.simulation.set_current_noise(std);
                }
                if ui
                    .checkbox(&mut self.simulation.slow_speed_loop, "慢速环")
                    .changed()
                {
                    let (kp, ki) = if self.simulation.slow_speed_loop {
                        (0.02, 0.02)
                    } else {
                        (0.1, 0.1)
                    };
                    self.simulation.speed_kp = kp;
                    self.simulation.speed_ki = ki;
                }
            });
            ui.horizontal(|ui| {
                ui.label("斜率滤波带宽");
                Slider::new(&mut self.simulation.slope_lpf_hz, 100.0..=1000.0)
                    .logarithmic(true)
                    .suffix(" Hz")
                    .ui(ui);
                ui.checkbox(
                    &mut self.simulation.direct_flux_observer.pll_enable,
                    "direct PLL",
                );
                ui.label("PLL 带宽");
                let mut pll_hz = self.simulation.direct_flux_observer.pll_wn
                    / std::f64::consts::TAU;
                if Slider::new(&mut pll_hz, 10.0..=300.0)
                    .logarithmic(true)
                    .suffix(" Hz")
                    .ui(ui)
                    .changed()
                {
                    self.simulation.direct_flux_observer.pll_wn =
                        pll_hz * std::f64::consts::TAU;
                }
            });
            ScrollArea::vertical().show(ui, |ui| {
                self.flux_plot(ui);
                self.scope.ui(ui);
            });
        });
    }
}

#[derive(Default)]
struct Metrics {
    n: usize,
    valid: usize,
    sum_err: f64,
    sum_err2: f64,
    max_err: f64,
    sum_speed_err2: f64,
}

impl Metrics {
    fn push(&mut self, err: f64, speed_err: f64, valid: bool) {
        self.n += 1;
        if valid {
            self.valid += 1;
        }
        self.sum_err += err;
        self.sum_err2 += err * err;
        self.max_err = self.max_err.max(err.abs());
        self.sum_speed_err2 += speed_err * speed_err;
    }

    fn rms(&self) -> f64 {
        if self.n == 0 {
            return f64::NAN;
        }
        (self.sum_err2 / self.n as f64).sqrt()
    }

    fn bias(&self) -> f64 {
        if self.n == 0 {
            return f64::NAN;
        }
        self.sum_err / self.n as f64
    }

    fn speed_rms(&self) -> f64 {
        if self.n == 0 {
            return f64::NAN;
        }
        (self.sum_speed_err2 / self.n as f64).sqrt()
    }

    fn valid_percent(&self) -> f64 {
        if self.n == 0 {
            return f64::NAN;
        }
        100.0 * self.valid as f64 / self.n as f64
    }
}

fn arg_value<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
}

fn arg_f64(args: &[String], key: &str, default: f64) -> f64 {
    arg_value(args, key)
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}

/// 无界面批处理：用与 GUI 完全相同的 `Simulation` 跑一段，打印磁链圆观测器指标。
fn run_headless(args: &[String]) {
    let seconds = arg_f64(args, "--seconds", 10.0);
    let speed = arg_f64(args, "--speed", 20.0);
    let r_scale = arg_f64(args, "--r-scale", 1.0);
    let plant_step = arg_f64(args, "--plant-step", 5.0e-6);
    let noise_scale = arg_f64(args, "--slope-noise", 1.0);
    // 按用户说明：零矢量斜率对应 v=0，不受死区电压影响，默认不注入。
    let dead = args.iter().any(|a| a == "--inject-dead");
    let dead_scale = arg_f64(args, "--dead-scale", 1.0);
    let current_noise = arg_value(args, "--current-noise").and_then(|s| s.parse::<f64>().ok());
    let speed_ramp = arg_f64(args, "--speed-ramp", 1000.0);
    // 调试用速度阶跃：`--speed2/--speed3` + 生效时刻，用来复现“先加速再减速”。
    let speed2 = arg_f64(args, "--speed2", f64::NAN);
    let speed2_time = arg_f64(args, "--speed2-time", 1.0);
    let speed3 = arg_f64(args, "--speed3", f64::NAN);
    let speed3_time = arg_f64(args, "--speed3-time", 3.0);
    let speed_kp = arg_f64(args, "--speed-kp", 0.1);
    let speed_ki = arg_f64(args, "--speed-ki", 0.1);
    let load = arg_value(args, "--load").and_then(|s| s.parse::<f64>().ok());
    // 观测器只做旁观（电流环仍用传感器角），用来单独评价观测器。
    let observe_only = args.iter().any(|a| a == "--observe-only");
    let handover = arg_f64(args, "--handover", 0.0);
    let tau = arg_f64(args, "--tau", 1.0);
    let hp_rel = arg_f64(args, "--hp-rel", 200.0);
    let pll = args.iter().any(|a| a == "--pll");
    let pll_hz = arg_f64(args, "--pll-hz", 50.0);
    let obs_min = arg_f64(args, "--obs-min", 5.0e-3);
    let k_prior = arg_f64(args, "--k-prior", 0.02);
    let k_step = arg_f64(args, "--k-step", 0.02);
    let max_sigma_rel = arg_f64(args, "--max-sigma-rel", 0.02);
    let k_sigma = arg_f64(args, "--k-sigma", 0.0);
    let hard_sigma = args.iter().any(|a| a == "--hard-sigma");
    let fit_period = arg_f64(args, "--fit-period", 0.01);
    let warmup_fraction = arg_f64(args, "--warmup", 0.5).clamp(0.0, 0.99);
    let verbose = args.iter().any(|a| a == "--verbose");
    let closed_loop_idx = 12;

    let dt = 1.0 / 20e3;
    let mut sim = Simulation::new();
    sim.delta_time = dt;
    sim.timer = Timer::new(vec![plant_step, dt]);
    sim.cmd_speed = speed;
    sim.target_speed = speed;
    sim.set_id = true;
    sim.id = 0.0;
    sim.slope_noise_scale = noise_scale;
    sim.slope_dead_voltage = dead;
    sim.slope_dead_scale = dead_scale;
    sim.power_bridge.dead_scale = dead_scale;
    sim.speed_ramp_rate = speed_ramp;
    sim.speed_kp = speed_kp;
    sim.speed_ki = speed_ki;
    if let Some(std) = current_noise {
        sim.current_sample_noise = [Noise::new(std, 1), Noise::new(std, 2), Noise::new(std, 3)];
        sim.current_noise_std = std;
    }
    if let Some(torque) = load {
        sim.motion_load.static_friction_torque = torque;
    }
    if !observe_only {
        // 默认从 0 秒就闭环；--handover 可先让传感器闭环把转子带到低速，
        // 再交班给观测器。
        sim.use_observer = if handover > 0.0 { 0 } else { closed_loop_idx };
    }

    let mean_l = (sim.motor.inductance_dq[0] + sim.motor.inductance_dq[1]) * 0.5;
    let nominal_r = sim.motor.rs / mean_l;
    let nominal_radius = sim.motor.flux / mean_l;
    let theta0 = angle_normal(sim.motion_load.angle * sim.motor.pole_pairs);

    // 先把拟合参数写进 observer，再 set_initial——Tracker 在 set_initial 里
    // 用这份配置构造，顺序反了会拿到默认 tau。
    sim.direct_flux_observer.fit.tau = tau;
    sim.direct_flux_observer.fit.hp_rel = hp_rel;
    sim.direct_flux_observer.pll_enable = pll;
    sim.direct_flux_observer.pll_wn = std::f64::consts::TAU * pll_hz;
    sim.direct_flux_observer.fit.obs_min = obs_min;
    sim.direct_flux_observer.fit.k_prior_rel = k_prior;
    sim.direct_flux_observer.fit.k_step_rel = k_step;
    sim.direct_flux_observer.fit.max_sigma_rel = max_sigma_rel;
    sim.direct_flux_observer.fit.k_sigma = k_sigma;
    sim.direct_flux_observer.fit.gate_sigma_rel = hard_sigma;
    sim.direct_flux_observer.fit_period = fit_period;
    sim.direct_flux_observer
        .set_initial(theta0, 0.0, r_scale * nominal_r, nominal_radius);

    let steps = (seconds / dt).round() as usize;
    let warmup_steps = (steps as f64 * warmup_fraction) as usize;

    let mut startup = Metrics::default();
    let mut steady = Metrics::default();
    let mut r_min = f64::INFINITY;
    let mut r_max = f64::NEG_INFINITY;
    let mut last_print = 0.0;

    for k in 0..steps {
        let t_step = k as f64 * dt;
        if speed2.is_finite() && t_step >= speed2_time {
            sim.cmd_speed = speed2;
        }
        if speed3.is_finite() && t_step >= speed3_time {
            sim.cmd_speed = speed3;
        }
        if !observe_only && handover > 0.0 {
            sim.use_observer = if (k as f64) * dt >= handover {
                closed_loop_idx
            } else {
                0
            };
        }
        sim.update();
        let t = (k + 1) as f64 * dt;
        let true_angle = angle_normal(sim.motion_load.angle * sim.motor.pole_pairs);
        let true_speed = sim.motion_load.speed * sim.motor.pole_pairs;

        let (out, obs_r, obs_radius, obs_index, valid) = {
            let o = &sim.direct_flux_observer;
            (
                sim.direct_flux_observer_output.clone(),
                o.r,
                o.radius,
                o.obs,
                o.valid,
            )
        };
        let err = angle_normal(out.electrical_angle - true_angle);
        let speed_err = out.electrical_speed - true_speed;

        if k >= warmup_steps {
            steady.push(err, speed_err, valid);
        } else {
            startup.push(err, speed_err, valid);
        }
        if valid {
            r_min = r_min.min(obs_r);
            r_max = r_max.max(obs_r);
        }

        if verbose && t - last_print >= seconds / 20.0 {
            last_print = t;
            println!(
                "t={t:6.3}s err={:8.3}deg r={:8.2} radius={:7.3} obs={:.2e} valid={}",
                err.to_degrees(),
                obs_r,
                obs_radius,
                obs_index,
                valid,
            );
        }
    }

    println!("=== direct flux observer (headless) ===");
    println!(
        "seconds={seconds} speed={speed} r0_scale={r_scale} plant_step={plant_step:.1e} \
         slope_noise={noise_scale} current_noise={:.4} speed_ramp={speed_ramp} \
         tau={tau} hp_rel={hp_rel} pll={pll} pll_hz={pll_hz} obs_min={obs_min:.1e} \
         prior={k_prior} step={k_step} \
         sigma_rel={max_sigma_rel:.3} k_sigma={k_sigma} hard_sigma={hard_sigma} \
         fit_period={:.4} speed_kp={speed_kp} speed_ki={speed_ki} handover={handover}",
        current_noise.unwrap_or(0.01),
        sim.direct_flux_observer.fit_period,
    );
    println!("true R={nominal_r:.3} s^-1, true radius={nominal_radius:.4} A");
    println!(
        "startup: n={} valid={:.1}% rms={:.3}deg max={:.3}deg bias={:.3}deg",
        startup.n,
        startup.valid_percent(),
        startup.rms().to_degrees(),
        startup.max_err.to_degrees(),
        startup.bias().to_degrees()
    );
    println!(
        "steady : n={} valid={:.1}% rms={:.3}deg max={:.3}deg bias={:.3}deg speed_rms={:.3}rad/s",
        steady.n,
        steady.valid_percent(),
        steady.rms().to_degrees(),
        steady.max_err.to_degrees(),
        steady.bias().to_degrees(),
        steady.speed_rms()
    );
    {
        let o = &sim.direct_flux_observer;
        println!(
            "observer: r={:.3} ({:+.2}%) r_fit={:.3} radius={:.4} ({:+.2}%) radius_fit={:.4} obs={:.3e} sigma_rel={:.3e} sigma_k={:.3e} p1={:.2e} p2={:.2e} weight={:.0} steps={} fits={} updates={} valid={} slope_dt={:.2}us",
            o.r,
            100.0 * (o.r - nominal_r) / nominal_r,
            o.r_fit,
            o.radius,
            100.0 * (o.radius - nominal_radius) / nominal_radius,
            o.radius_fit,
            o.obs,
            o.sigma_rel,
            o.sigma_k,
            o.p1,
            o.p2,
            o.weight,
            o.steps,
            o.fit_updates,
            o.updates,
            o.valid,
            sim.last_slope_dt * 1.0e6,
        );
    }
    println!("R range over valid samples: {r_min:.3} .. {r_max:.3}");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--headless") {
        run_headless(&args);
        return;
    }
    let options = NativeOptions {
        ..Default::default()
    };
    eframe::run_native(
        "example1",
        options,
        Box::new(|cc| Ok(Box::new(Application::new(cc)?))),
    )
    .unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 高阻模式：电流强制为 0、端电压为反电势，观测器靠积分电压跟角度；
    /// 切换时 `update` 会调用 `high_z_flux_observer.reset()`。
    #[test]
    fn high_z_mode_forces_zero_current_and_tracks_angle() {
        let mut sim = Simulation::new();
        // 1 mA：高阻的斜率窗口只有 9.5 µs，10 mA 下角度噪声本来就大；
        // 这个测试验证的是接线/零电流/角度，不是噪声性能。
        sim.set_current_noise(0.001);
        sim.motion_load.speed = 100.0 / sim.motor.pole_pairs;
        sim.use_observer = HIGH_Z_OBSERVER;

        // 0.5 s：5000 次 100 µs 的 update（控制环 20 kHz）。
        for _ in 0..5000 {
            sim.update();
        }

        let i_max = sim
            .motor
            .current_dq
            .iter()
            .fold(0.0_f64, |m, i| m.max(i.abs()));
        let theta_true = angle_normal(sim.motion_load.angle * sim.motor.pole_pairs);
        let theta_est = sim.high_z_flux_observer_output.electrical_angle;
        let err = angle_normal(theta_est - theta_true).abs();
        eprintln!(
            "high-z test: i_max={i_max:.3e} theta={theta_est:.4} true={theta_true:.4} \
             err={:.3}deg radius={:.4} valid={} w={:.0}",
            err.to_degrees(),
            sim.high_z_flux_observer.radius,
            sim.high_z_flux_observer.valid,
            sim.high_z_flux_observer.weight
        );
        assert!(i_max < 1.0e-9, "高阻下电流必须为 0，实际 {i_max}");
        assert!(sim.high_z_flux_observer.valid);
        assert!(err < 0.1, "angle error {err} rad");
        assert!((sim.high_z_flux_observer.radius - 1.0).abs() < 1.0e-9);
    }
}
