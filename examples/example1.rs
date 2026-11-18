use std::io::Write;

use eframe::{App, CreationContext, Frame, NativeOptions};
use egui::{CentralPanel, CollapsingHeader, Context, ScrollArea, Slider, Ui, Widget};
use egui_plot::{Legend, Line, Plot, PlotPoint, default_label_formatter};
use moto_sim::model::fan::{FAN_LOAD, FAN_MOTOR};
use moto_sim::model::zq10y_dead_mapping;
use moto_sim::simulation::controller::current_regulator::three_phase_pi_current_regulator::ThreePhasePICurrentRegulator;
use moto_sim::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use moto_sim::simulation::controller::inline_tune::{InductanceTune, ResTune};
use moto_sim::simulation::controller::observer::any_observer::{AllObserver, AnyObserver};
use moto_sim::simulation::controller::observer::base_flux_observer::BaseFluxObserver;
use moto_sim::simulation::controller::observer::ds_observer::DsObserver;
use moto_sim::simulation::controller::observer::ex_flux_observer::ExFluxObserver;
use moto_sim::simulation::controller::observer::flux_circle_observer::FluxCircleObserver;
use moto_sim::simulation::controller::observer::flux_observer::FluxObserver;
use moto_sim::simulation::controller::observer::grad_observer::GradObserver;
use moto_sim::simulation::controller::observer::direct_flux_observer::DirectFluxObserver;
use moto_sim::simulation::controller::observer::kasa_flux_observer::KasaFluxObserver;
use moto_sim::simulation::controller::observer::mix_observer::MixObserver;
use moto_sim::simulation::controller::observer::mp_observer::MpObserver;
use moto_sim::simulation::controller::observer::sensor_observer::SensorObserver;
use moto_sim::simulation::controller::observer::ss_observer::{SSObserver, SSample};
use moto_sim::simulation::controller::observer::ss2_observer::{S2Sample, SS2Observer};
use moto_sim::simulation::controller::observer::super_flux_observer::{
    SuperFluxObserver, SuperFluxSample,
};
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
    slope_sample_noise: [Noise; 3],
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
    /// GUI 演示用：把电流采样噪声切到 1 mA（PAPER.md 的假设值）。
    pub flux_circle_low_noise: bool,
    /// GUI 演示用：闭环磁链观测器时把速度环降慢，避免与 PLL 相位滞后耦合。
    pub flux_circle_slow_speed_loop: bool,

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
    pub flux_circle_observer_output: ObserverOutput,
    pub kasa_flux_observer_output: ObserverOutput,
    pub direct_flux_observer_output: ObserverOutput,

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
    pub flux_circle_observer: FluxCircleObserver,
    pub kasa_flux_observer: KasaFluxObserver,
    pub direct_flux_observer: DirectFluxObserver,

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

        // 磁链圆观测器：PAPER.md 的算法。初始角度取本仿真已知的转子位置
        // （等价于启动前的 ALIGN），R 与半径取电机铭牌值。
        let mean_inductance = (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5;
        let nominal_r = motor.rs / mean_inductance;
        let nominal_radius = motor.flux / mean_inductance;
        let initial_electrical_angle = angle_normal(std::f64::consts::PI * 0.5 * motor.pole_pairs);
        let mut flux_circle_observer = FluxCircleObserver::new();
        flux_circle_observer.set_initial(initial_electrical_angle, 0.0, nominal_r, nominal_radius);
        flux_circle_observer.pll_wn_min = 50.0;

        // 流式 Kåsa 观测器：同样的启动状态，拟合换成指数遗忘的递归矩。
        let mut kasa_flux_observer = KasaFluxObserver::new();
        kasa_flux_observer.set_initial(
            initial_electrical_angle,
            0.0,
            nominal_r,
            nominal_radius,
        );
        kasa_flux_observer.pll_wn_min = 50.0;
        kasa_flux_observer.fit.tau = 1.0;

        // 直接磁链积分观测器（无 PLL，快速环反正切 + 10 ms 圆拟合）。
        let mut direct_flux_observer = DirectFluxObserver::new();
        direct_flux_observer.set_initial(
            initial_electrical_angle,
            0.0,
            nominal_r * 0.2,
            nominal_radius,
        );
        direct_flux_observer.fit.tau = 10.0;

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
            slope_sample_noise: [
                Noise::new(1.0, 11),
                Noise::new(1.0, 12),
                Noise::new(1.0, 13),
            ],
            slope_noise_scale: 1.0,
            // 零矢量斜率是 v=0 时的 di/dt，死区/器件压降不进入这一测量，
            // 因此默认不注入。保留开关仅用于对照“平均模型误解”的敏感性。
            slope_dead_voltage: false,
            slope_dead_scale: 1.0,
            current_noise_std: 0.01,
            last_slope_dt: 0.0,
            flux_circle_low_noise: false,
            flux_circle_slow_speed_loop: false,
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
            flux_circle_observer,
            kasa_flux_observer,
            direct_flux_observer,
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

                    // ---- 零矢量电流斜率传感器模型 ----
                    // 本仿真的功率桥是平均模型，没有真实载波/开关状态，所以这里
                    // 用电机 dq ODE 在 v=0 时的导数合成“双采样斜率”，再按零矢量
                    // 时长注入噪声。它不是从真实 PWM 波形里测出来的。
                    let duty = self.power_bridge_input.output_duty;
                    let duty_max = duty.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    let duty_min = duty.iter().copied().fold(f64::INFINITY, f64::min);
                    let zero_vector_fraction = (1.0 - 0.5 * (duty_max - duty_min)).clamp(0.0, 1.0);
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

                        let mut slope_3ph = inverse_clarke(slope_ab);
                        let sigma_s = std::f64::consts::SQRT_2
                            * self.current_noise_std
                            * self.slope_noise_scale
                            / slope_dt;
                        for i in 0..3 {
                            self.slope_sample_noise[i].update(delta_time);
                            slope_3ph[i] += self.slope_sample_noise[i].value * sigma_s;
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
                    self.flux_circle_observer_output = self
                        .flux_circle_observer
                        .update(delta_time, &self.observer_input);
                    self.kasa_flux_observer_output = self
                        .kasa_flux_observer
                        .update(delta_time, &self.observer_input);
                    self.direct_flux_observer_output = self
                        .direct_flux_observer
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
                        12 => &self.flux_circle_observer_output,
                        13 => &self.kasa_flux_observer_output,
                        14 => &self.direct_flux_observer_output,
                        _ => &self.sensor_observer_output,
                    };
                    self.current_regulator_input.electrical_speed = use_observer.electrical_speed;
                    self.current_regulator_input.electrical_angle = use_observer.electrical_angle;
                    self.current_regulator_input.continuous_speed = use_observer.continuous_speed;
                    self.current_regulator_input.current = sample_current;

                    let max_current = 2.0;
                    let speed_error =
                        self.target_speed - self.current_regulator_input.electrical_speed;
                    let speed_p = speed_error * self.speed_kp;
                    self.speed_i += speed_error * self.speed_ki * delta_time;
                    self.speed_i = self.speed_i.clamp(-max_current, max_current);
                    let output_torque = (self.speed_i + speed_p).clamp(-max_current, max_current);
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
                            id + 0.1
                                * f64::cos(
                                    use_observer.electrical_angle - self.res_tune.tune_angle,
                                ),
                            output_torque,
                        ],
                        use_observer.electrical_angle,
                    );
                    self.current_regulator_input.command_current = inverse_clarke(command_current);
                    self.current_regulator_output = self
                        .current_regulator
                        .update(delta_time, &self.current_regulator_input);

                    let mut command_voltage = self.current_regulator_output.command_voltage;

                    let inject_voltage = inverse_clarke(rotate(
                        [
                            10.0 * f64::sin(5000.0 * self.timer.time),
                            0.0 * f64::sin(5000.0 * self.timer.time),
                        ],
                        self.current_regulator_input.electrical_angle,
                    ));
                    let inject_voltage =
                        inverse_clarke(rotate([0.0, 0.0], 5000.0 * self.timer.time));
                    command_voltage[0] += inject_voltage[0];
                    command_voltage[1] += inject_voltage[1];
                    command_voltage[2] += inject_voltage[2];

                    self.power_bridge_input.output_duty =
                        command_voltage.map(|i| nn(i * 2.0 / sample_bus_voltage));
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
}

impl<S> ScopeData<S> {
    pub fn new<F: FnMut(&mut S) -> f64 + 'static>(name: &str, f: F) -> Self {
        Self {
            name: name.to_string(),
            data: Vec::new(),
            update: Box::new(f),
        }
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
                                ui.line(Line::new(&d.name, d.data.as_slice()));
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
}

impl Application {
    pub fn new(cc: &CreationContext) -> anyhow::Result<Self> {
        load_chinese_font(&cc.egui_ctx);
        let mut this = Self {
            simulation: Simulation::new(),
            run: false,
            scope: Scope::new(),
        };
        this.reset();
        Ok(this)
    }

    pub fn reset(&mut self) {
        self.run = false;
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
                    ScopeData::new("flux_circle_angle", |s: &mut Simulation| {
                        s.flux_circle_observer_output.electrical_angle
                    }),
                    ScopeData::new("kasa_angle", |s: &mut Simulation| {
                        s.kasa_flux_observer_output.electrical_angle
                    }),
                    ScopeData::new("direct_angle", |s: &mut Simulation| {
                        s.direct_flux_observer_output.electrical_angle
                    }),
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
                    // ScopeData::new("flux_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.flux_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("grad_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.grad_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
                    // ScopeData::new("mix_angle", |s: &mut Simulation| {
                    //     angle_normal(
                    //         s.mix_observer_output.electrical_angle
                    //             - s.motion_load.angle * s.motor.pole_pairs,
                    //     )
                    // }),
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
                    ScopeData::new("flux_circle_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.flux_circle_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    ScopeData::new("kasa_angle", |s: &mut Simulation| {
                        angle_normal(
                            s.kasa_flux_observer_output.electrical_angle
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
                    // 未过 PLL 的圆上测量角误差：用来区分“几何错了”还是“PLL 滞后”。
                    ScopeData::new("kasa_meas", |s: &mut Simulation| {
                        angle_normal(
                            s.kasa_flux_observer.theta_meas
                                - s.motion_load.angle * s.motor.pole_pairs,
                        )
                    }),
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
                    ScopeData::new("flux_circle_speed", |s: &mut Simulation| {
                        s.flux_circle_observer_output.electrical_speed
                    }),
                    ScopeData::new("kasa_speed", |s: &mut Simulation| {
                        s.kasa_flux_observer_output.electrical_speed
                    }),
                    ScopeData::new("direct_speed", |s: &mut Simulation| {
                        s.direct_flux_observer_output.electrical_speed
                    }),
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
                "flux_circle_r".to_string(),
                vec![
                    ScopeData::new("true_R", |s: &mut Simulation| {
                        s.motor.rs / ((s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5)
                    }),
                    ScopeData::new("estimated_R", |s: &mut Simulation| s.flux_circle_observer.r),
                ],
            ),
            (
                "flux_circle_radius".to_string(),
                vec![
                    ScopeData::new("true_radius", |s: &mut Simulation| {
                        s.motor.flux / ((s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5)
                    }),
                    ScopeData::new("estimated_radius", |s: &mut Simulation| {
                        s.flux_circle_observer.radius
                    }),
                ],
            ),
            (
                "flux_circle_observability".to_string(),
                vec![
                    ScopeData::new("gamma", |s: &mut Simulation| s.flux_circle_observer.gamma),
                    ScopeData::new("fit_count", |s: &mut Simulation| {
                        s.flux_circle_observer.fit_count as f64
                    }),
                    ScopeData::new("fit_fail", |s: &mut Simulation| {
                        s.flux_circle_observer.fit_fail as f64
                    }),
                ],
            ),
            (
                "flux_circle_valid".to_string(),
                vec![
                    ScopeData::new("valid", |s: &mut Simulation| {
                        if s.flux_circle_observer.valid {
                            1.0
                        } else {
                            0.0
                        }
                    }),
                    ScopeData::new("slope_dt_us", |s: &mut Simulation| s.last_slope_dt * 1.0e6),
                ],
            ),
            (
                "kasa_flux_r".to_string(),
                vec![
                    ScopeData::new("true_R", |s: &mut Simulation| {
                        s.motor.rs / ((s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5)
                    }),
                    ScopeData::new("estimated_R", |s: &mut Simulation| s.kasa_flux_observer.r),
                ],
            ),
            (
                "kasa_flux_radius".to_string(),
                vec![
                    ScopeData::new("true_radius", |s: &mut Simulation| {
                        s.motor.flux / ((s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5)
                    }),
                    ScopeData::new("estimated_radius", |s: &mut Simulation| {
                        s.kasa_flux_observer.radius
                    }),
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
                ],
            ),
            (
                "kasa_flux_observability".to_string(),
                vec![
                    ScopeData::new("obs", |s: &mut Simulation| s.kasa_flux_observer.obs),
                    ScopeData::new("sigma_rel", |s: &mut Simulation| {
                        s.kasa_flux_observer.sigma_rel
                    }),
                    ScopeData::new("data_arc", |s: &mut Simulation| {
                        s.kasa_flux_observer.data_arc
                    }),
                    ScopeData::new("weight", |s: &mut Simulation| s.kasa_flux_observer.weight),
                    ScopeData::new("use_fit", |s: &mut Simulation| {
                        if s.kasa_flux_observer.use_fit {
                            1.0
                        } else {
                            0.0
                        }
                    }),
                    ScopeData::new("scan_done", |s: &mut Simulation| {
                        if s.kasa_flux_observer.scan_done {
                            1.0
                        } else {
                            0.0
                        }
                    }),
                ],
            ),
            (
                "kasa_flux_newton".to_string(),
                vec![
                    ScopeData::new("p1", |s: &mut Simulation| s.kasa_flux_observer.p1),
                    ScopeData::new("p2", |s: &mut Simulation| s.kasa_flux_observer.p2),
                    ScopeData::new("steps", |s: &mut Simulation| {
                        s.kasa_flux_observer.steps as f64
                    }),
                ],
            ),
            (
                "kasa_flux_mag".to_string(),
                vec![
                    ScopeData::new("flux_mag", |s: &mut Simulation| {
                        s.kasa_flux_observer.flux_mag
                    }),
                    ScopeData::new("radius", |s: &mut Simulation| s.kasa_flux_observer.radius),
                ],
            ),
            (
                "kasa_flux_valid".to_string(),
                vec![
                    ScopeData::new("valid", |s: &mut Simulation| {
                        if s.kasa_flux_observer.valid {
                            1.0
                        } else {
                            0.0
                        }
                    }),
                    ScopeData::new("slope_dt_us", |s: &mut Simulation| s.last_slope_dt * 1.0e6),
                ],
            ),
        ];
    }

    fn update(&mut self) {
        self.simulation.update();
        self.scope
            .update(self.simulation.timer.time, &mut self.simulation);
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
                ui.label("use_observer");
                Slider::new(&mut self.simulation.use_observer, 0..=14).ui(ui);
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
                    12 => "flux_circle_observer",
                    13 => "kasa_flux_observer",
                    14 => "direct_flux_observer",
                    _ => "sensor_observer",
                });
                if ui
                    .checkbox(&mut self.simulation.flux_circle_low_noise, "flux 1mA")
                    .changed()
                {
                    let std = if self.simulation.flux_circle_low_noise {
                        0.001
                    } else {
                        0.01
                    };
                    self.simulation.set_current_noise(std);
                }
                if ui
                    .checkbox(
                        &mut self.simulation.flux_circle_slow_speed_loop,
                        "flux slow loop",
                    )
                    .changed()
                {
                    let (kp, ki) = if self.simulation.flux_circle_slow_speed_loop {
                        (0.02, 0.02)
                    } else {
                        (0.1, 0.1)
                    };
                    self.simulation.speed_kp = kp;
                    self.simulation.speed_ki = ki;
                }
            });
            ScrollArea::vertical().show(ui, |ui| {
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
    let use_flux_circle = args.iter().any(|a| a == "--use-flux-circle");
    let use_kasa = args.iter().any(|a| a == "--use-kasa");
    let use_direct = args.iter().any(|a| a == "--use-direct");
    // 观测器只做旁观（电流环仍用传感器角），用来单独评价观测器。
    let observe_only = args.iter().any(|a| a == "--observe-only");
    let handover = arg_f64(args, "--handover", 0.0);
    let no_fit = args.iter().any(|a| a == "--no-fit");
    let wn_min = arg_f64(args, "--wn-min", 50.0);
    let wn_min_run = arg_f64(args, "--wn-min-run", 1.0);
    let narrow_time = arg_f64(args, "--narrow-time", 2.0);
    let gamma_min = arg_f64(args, "--gamma-min", 0.05);
    let max_sigma_rel = arg_f64(args, "--max-sigma-rel", 0.02);
    let window = arg_f64(args, "--window", 1.0);
    let no_drift = args.iter().any(|a| a == "--no-drift");
    let kasa_tau = arg_f64(args, "--kasa-tau", 1.0);
    let kasa_scan_time = arg_f64(args, "--kasa-scan-time", 0.2);
    let kasa_min_arc = arg_f64(args, "--kasa-min-arc", std::f64::consts::TAU);
    let kasa_inner = arg_f64(args, "--kasa-inner", 2.0).max(0.0) as usize;
    let kasa_obs_min = arg_f64(args, "--kasa-obs-min", 5.0e-3);
    let kasa_prior = arg_f64(args, "--kasa-prior", 0.02);
    let kasa_step = arg_f64(args, "--kasa-step", 0.02);
    let kasa_max_sigma = arg_f64(args, "--kasa-max-sigma-rel", 0.02);
    let kasa_scan_gain = arg_f64(args, "--kasa-scan-gain", 0.5);
    let warmup_fraction = arg_f64(args, "--warmup", 0.5).clamp(0.0, 0.99);
    let csv = arg_value(args, "--csv").map(|s| s.to_string());
    let verbose = args.iter().any(|a| a == "--verbose");
    let closed_loop_idx = if use_direct {
        14
    } else if use_kasa {
        13
    } else {
        12
    };

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
    if (use_flux_circle || use_kasa || use_direct) && !observe_only {
        // 默认从 0 秒就闭环；--handover 可先让传感器闭环把转子带到低速，
        // 再交班给观测器（对应 PAPER.md §7 的 RUN 阶段）。
        sim.use_observer = if handover > 0.0 { 0 } else { closed_loop_idx };
    }

    let mean_l = (sim.motor.inductance_dq[0] + sim.motor.inductance_dq[1]) * 0.5;
    let nominal_r = sim.motor.rs / mean_l;
    let nominal_radius = sim.motor.flux / mean_l;
    let theta0 = angle_normal(sim.motion_load.angle * sim.motor.pole_pairs);
    sim.flux_circle_observer
        .set_initial(theta0, 0.0, r_scale * nominal_r, nominal_radius);
    sim.flux_circle_observer.pll_wn_min = wn_min;
    sim.flux_circle_observer.pll_wn_min_run = wn_min_run;
    sim.flux_circle_observer.pll_narrow_time = narrow_time;
    sim.flux_circle_observer.gamma_min = gamma_min;
    sim.flux_circle_observer.max_sigma_rel = max_sigma_rel;
    sim.flux_circle_observer.min_window = window;
    sim.flux_circle_observer.fit_cfg.fit_drift = !no_drift;
    sim.flux_circle_observer.fit_enabled = !no_fit;

    sim.kasa_flux_observer
        .set_initial(theta0, 0.0, r_scale * nominal_r, nominal_radius);
    sim.kasa_flux_observer.pll_wn_min = wn_min;
    sim.kasa_flux_observer.pll_wn_min_run = wn_min_run;
    sim.kasa_flux_observer.pll_narrow_time = narrow_time;
    sim.kasa_flux_observer.fit.tau = kasa_tau;
    sim.kasa_flux_observer.fit.inner_iters = kasa_inner;
    sim.kasa_flux_observer.fit.obs_min = kasa_obs_min;
    sim.kasa_flux_observer.fit.k_prior_rel = kasa_prior;
    sim.kasa_flux_observer.fit.k_step_rel = kasa_step;
    sim.kasa_flux_observer.fit.max_sigma_rel = kasa_max_sigma;
    sim.kasa_flux_observer.fit.scan_gain = kasa_scan_gain;
    sim.kasa_flux_observer.scan_time = kasa_scan_time;
    sim.kasa_flux_observer.min_arc = kasa_min_arc;
    sim.kasa_flux_observer.fit_enabled = !no_fit;

    sim.direct_flux_observer
        .set_initial(theta0, 0.0, r_scale * nominal_r, nominal_radius);
    sim.direct_flux_observer.fit.tau = kasa_tau;
    sim.direct_flux_observer.fit.obs_min = kasa_obs_min;
    sim.direct_flux_observer.fit.k_step_rel = kasa_step;

    let steps = (seconds / dt).round() as usize;
    let warmup_steps = (steps as f64 * warmup_fraction) as usize;

    let mut csv_writer = if use_direct {
        if csv.is_some() {
            eprintln!("--csv 还没有为 --use-direct 接线，忽略导岀。");
        }
        None
    } else {
        csv.as_ref().map(|path| {
        let file = std::fs::File::create(path).expect("create csv");
        let mut writer = std::io::BufWriter::new(file);
        if use_kasa {
            writeln!(
                writer,
                "t,true_angle,est_angle,err,theta_meas,err_meas,true_speed,est_speed,id,iq,r,radius,obs,sigma_rel,valid,slope_dt_us,p_x,p_y,c_x,c_y,scan_done,steps,weight"
            )
            .unwrap();
        } else {
            writeln!(
                writer,
                "t,true_angle,est_angle,err,theta_meas,err_meas,true_speed,est_speed,id,iq,r,radius,gamma,valid,slope_dt_us,u_x,u_y,c_x,c_y,fit_count,fit_fail,fit_gamma,fit_arc,fit_sigma"
            )
            .unwrap();
        }
        writer
    })
    };

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
        if (use_flux_circle || use_kasa || use_direct) && !observe_only && handover > 0.0 {
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

        let (out, obs_r, obs_radius, obs_index, valid, theta_meas) = if use_direct {
            let o = &sim.direct_flux_observer;
            (
                sim.direct_flux_observer_output.clone(),
                o.r,
                o.radius,
                o.obs,
                o.valid,
                o.theta_meas,
            )
        } else if use_kasa {
            let o = &sim.kasa_flux_observer;
            (
                sim.kasa_flux_observer_output.clone(),
                o.r,
                o.radius,
                o.obs,
                o.valid,
                o.theta_meas,
            )
        } else {
            let o = &sim.flux_circle_observer;
            (
                sim.flux_circle_observer_output.clone(),
                o.r,
                o.radius,
                o.gamma,
                o.valid,
                o.theta_meas,
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

        if let Some(writer) = csv_writer.as_mut() {
            if k % 10 == 0 {
                let err_meas = angle_normal(theta_meas - true_angle);
                if use_kasa {
                    let o = &sim.kasa_flux_observer;
                    let (p, c) = o.debug_state();
                    writeln!(
                        writer,
                        "{t:.6},{true_angle:.6},{est:.6},{err:.6},{theta_meas:.6},{err_meas:.6},{true_speed:.6},{est_speed:.6},{id:.6},{iq:.6},{r:.6},{radius:.6},{obs:.3e},{sigma_rel:.3e},{valid},{slope_dt_us:.3e},{p_x:.6},{p_y:.6},{c_x:.6},{c_y:.6},{scan_done},{steps},{weight:.2}",
                        est = out.electrical_angle,
                        est_speed = out.electrical_speed,
                        id = sim.motor.current_dq[0],
                        iq = sim.motor.current_dq[1],
                        r = obs_r,
                        radius = obs_radius,
                        obs = obs_index,
                        sigma_rel = o.sigma_rel,
                        valid = if valid { 1 } else { 0 },
                        slope_dt_us = sim.last_slope_dt * 1.0e6,
                        p_x = p.x,
                        p_y = p.y,
                        c_x = c.x,
                        c_y = c.y,
                        scan_done = if o.scan_done { 1 } else { 0 },
                        steps = o.steps,
                        weight = o.weight,
                    )
                    .unwrap();
                } else {
                    let fit = sim.flux_circle_observer.fit;
                    let (fg, fa, fs) = fit.map(|f| (f.gamma, f.arc, f.sigma)).unwrap_or((
                        f64::NAN,
                        f64::NAN,
                        f64::NAN,
                    ));
                    let (_, _, u, c) = sim.flux_circle_observer.debug_state();
                    writeln!(
                        writer,
                        "{t:.6},{true_angle:.6},{est:.6},{err:.6},{theta_meas:.6},{err_meas:.6},{true_speed:.6},{est_speed:.6},{id:.6},{iq:.6},{r:.6},{radius:.6},{gamma:.3e},{valid},{slope_dt_us:.3e},{u_x:.6},{u_y:.6},{c_x:.6},{c_y:.6},{fit_count},{fit_fail},{fg:.3e},{fa:.3e},{fs:.3e}",
                        est = out.electrical_angle,
                        est_speed = out.electrical_speed,
                        id = sim.motor.current_dq[0],
                        iq = sim.motor.current_dq[1],
                        r = obs_r,
                        radius = obs_radius,
                        gamma = obs_index,
                        valid = if valid { 1 } else { 0 },
                        slope_dt_us = sim.last_slope_dt * 1.0e6,
                        u_x = u.x,
                        u_y = u.y,
                        c_x = c.x,
                        c_y = c.y,
                        fit_count = sim.flux_circle_observer.fit_count,
                        fit_fail = sim.flux_circle_observer.fit_fail,
                    )
                    .unwrap();
                }
            }
        }

        if verbose && t - last_print >= seconds / 20.0 {
            last_print = t;
            println!(
                "t={t:6.3}s err={:8.3}deg r={:8.2} radius={:7.3} index={:.2e} valid={} scan={}",
                err.to_degrees(),
                obs_r,
                obs_radius,
                obs_index,
                valid,
                sim.kasa_flux_observer.scan_done,
            );
        }
    }

    if use_kasa {
        println!("=== kasa streaming observer (headless) ===");
        println!(
            "seconds={seconds} speed={speed} r0_scale={r_scale} plant_step={plant_step:.1e} \
             slope_noise={noise_scale} dead_injection={dead} dead_scale={dead_scale} \
             current_noise={:.4} speed_ramp={speed_ramp} wn_min={wn_min}->{wn_min_run} \
             tau={kasa_tau} scan_time={kasa_scan_time} min_arc={kasa_min_arc:.2} inner={kasa_inner} \
             obs_min={kasa_obs_min:.1e} prior={kasa_prior} step={kasa_step} \
             sigma_rel={kasa_max_sigma:.3} scan_gain={kasa_scan_gain:.2} \
             narrow_time={narrow_time} speed_kp={speed_kp} speed_ki={speed_ki} handover={handover}",
            current_noise.unwrap_or(0.01),
        );
    } else if use_direct {
        println!("=== direct flux observer (headless) ===");
        println!(
            "seconds={seconds} speed={speed} r0_scale={r_scale} plant_step={plant_step:.1e} \
             slope_noise={noise_scale} current_noise={:.4} speed_ramp={speed_ramp} \
             tau={kasa_tau} obs_min={kasa_obs_min:.1e} prior={kasa_prior} step={kasa_step} \
             sigma_rel={kasa_max_sigma:.3} fit_period={:.4} speed_kp={speed_kp} \
             speed_ki={speed_ki} handover={handover}",
            current_noise.unwrap_or(0.01),
            sim.direct_flux_observer.fit_period,
        );
    } else {
        println!("=== flux circle observer (headless) ===");
        println!(
            "seconds={seconds} speed={speed} r0_scale={r_scale} plant_step={plant_step:.1e} \
             slope_noise={noise_scale} dead_injection={dead} dead_scale={dead_scale} \
             current_noise={:.4} speed_ramp={speed_ramp} wn_min={wn_min}->{wn_min_run} \
             gamma_min={gamma_min} window={window} drift={} narrow_time={narrow_time} \
             speed_kp={speed_kp} speed_ki={speed_ki} closed_loop={use_flux_circle} handover={handover}",
            current_noise.unwrap_or(0.01),
            !no_drift
        );
    }
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
    if use_kasa {
        let o = &sim.kasa_flux_observer;
        println!(
            "observer: r={:.3} ({:+.2}%) radius={:.4} ({:+.2}%) obs={:.3e} sigma_rel={:.3e} weight={:.0} scan={} tries={} p2={:.2e} slope_dt={:.2}us",
            o.r,
            100.0 * (o.r - nominal_r) / nominal_r,
            o.radius,
            100.0 * (o.radius - nominal_radius) / nominal_radius,
            o.obs,
            o.sigma_rel,
            o.weight,
            o.scan_done,
            o.scan_tries,
            o.p2,
            sim.last_slope_dt * 1.0e6,
        );
        println!(
            "use_fit={} fit_updates={} updates={} valid={}",
            o.use_fit, o.fit_updates, o.updates, o.valid
        );
    } else if use_direct {
        let o = &sim.direct_flux_observer;
        println!(
            "observer: r={:.3} ({:+.2}%) r_fit={:.3} radius={:.4} ({:+.2}%) radius_fit={:.4} obs={:.3e} sigma_rel={:.3e} weight={:.0} steps={} fits={} updates={} valid={} slope_dt={:.2}us",
            o.r,
            100.0 * (o.r - nominal_r) / nominal_r,
            o.r_fit,
            o.radius,
            100.0 * (o.radius - nominal_radius) / nominal_radius,
            o.radius_fit,
            o.obs,
            o.sigma_rel,
            o.weight,
            o.steps,
            o.fit_updates,
            o.updates,
            o.valid,
            sim.last_slope_dt * 1.0e6,
        );
    } else {
        println!(
            "observer: r={:.3} ({:+.2}%) radius={:.4} ({:+.2}%) gamma={:.3e} fits={} fail={} slope_dt={:.2}us",
            sim.flux_circle_observer.r,
            100.0 * (sim.flux_circle_observer.r - nominal_r) / nominal_r,
            sim.flux_circle_observer.radius,
            100.0 * (sim.flux_circle_observer.radius - nominal_radius) / nominal_radius,
            sim.flux_circle_observer.gamma,
            sim.flux_circle_observer.fit_count,
            sim.flux_circle_observer.fit_fail,
            sim.last_slope_dt * 1.0e6,
        );
        if let Some(f) = sim.flux_circle_observer.fit {
            println!(
                "last fit: r={:.3} radius={:.3} gamma={:.3e} arc={:.2}rad sigma={:.3e} inlier={:.3} n={}",
                f.r, f.radius, f.gamma, f.arc, f.sigma, f.inlier_frac, f.n_obs
            );
        }
    }
    println!("R range over valid samples: {r_min:.3} .. {r_max:.3}");
    if let Some(path) = csv {
        println!("csv: {path}");
    }
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
