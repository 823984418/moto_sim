use eframe::{App, CreationContext, Frame, NativeOptions};
use egui::{CentralPanel, CollapsingHeader, Context, ScrollArea, Slider, Ui, Widget};
use egui_plot::{Line, Plot, PlotPoint};
use moto_sim::model::fan::{FAN_LOAD, FAN_MOTOR};
use moto_sim::simulation::controller::current_regulator::three_phase_pi_current_regulator::ThreePhasePICurrentRegulator;
use moto_sim::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use moto_sim::simulation::controller::observer::flux_observer::FluxObserver;
use moto_sim::simulation::controller::observer::grad_observer::GradObserver;
use moto_sim::simulation::controller::observer::mix_observer::MixObserver;
use moto_sim::simulation::controller::observer::sensor_observer::SensorObserver;
use moto_sim::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use moto_sim::simulation::motion_load::ideal_motion_load::IdealMotionLoad;
use moto_sim::simulation::motion_load::{MotionLoad, MotionLoadInput, MotionLoadOutput};
use moto_sim::simulation::motor::pmsm::PermanentMagnetSynchronousMotor;
use moto_sim::simulation::motor::{Motor, MotorInput, MotorOutput};
use moto_sim::simulation::noise::Noise;
use moto_sim::simulation::power_bridge::ideal_power_bridge::IdealPowerBridge;
use moto_sim::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};
use moto_sim::simulation::{angle_normal, clarke, inverse_clarke, rotate};
use moto_sim::ui::font::load_chinese_font;
use moto_sim::util::Timer;

#[derive(Debug, Default, Clone)]
pub struct Simulation {
    pub delta_time: f64,
    pub timer: Timer,
    pub target_speed: f64,
    pub use_observer: usize,

    sensor_noise: Noise,
    current_noise: [Noise; 3],
    voltage_noise: [Noise; 3],
    current_offset: [f64; 3],
    voltage_offset: [f64; 3],

    speed_i: f64,

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
    current_regulator_input: CurrentRegulatorInput<3>,
    current_regulator_output: CurrentRegulatorOutput<3>,

    pub power_bridge: IdealPowerBridge,
    pub motor: PermanentMagnetSynchronousMotor,
    pub motion_load: IdealMotionLoad,
    pub sensor_observer: SensorObserver,
    pub grad_observer: GradObserver,
    pub flux_observer: FluxObserver,
    pub mix_observer: MixObserver,
    pub current_regulator: ThreePhasePICurrentRegulator,
}

impl Simulation {
    pub fn new() -> Self {
        let motor = PermanentMagnetSynchronousMotor { ..FAN_MOTOR };
        let sensor_observer = SensorObserver {
            pole_pairs: motor.pole_pairs,
            pll_kp: 100.0,
            pll_ki: 1000.0,
            ..Default::default()
        };
        let flux_observer = FluxObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            speed_lp_factor: 100.0,
            position_factor: 100.0,
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
            theta_error_kp: 10.0,
            theta_error_ki: 10.0,
            speed_error_kp: 0.0,
            speed_error_ki: 100.0,
            ..Default::default()
        };
        let grad_observer = GradObserver {
            rs: motor.rs,
            l0: (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            l1: (motor.inductance_dq[0] - motor.inductance_dq[1]) * 0.5,
            flux: motor.flux,
            kr: 0.0,
            kl0: 0.0,
            kflux: 0.0,
            kl1_ds: 0.0,
            kl1_de: 0.0,

            kspeed_l1: 100.0,
            kspeed_flux: 100.0,
            kspeed_kangle: 10.0,
            kangle_l1_di: 100.0,
            kangle_l1_ds: 100.0,
            kangle_flux: 100.0,

            angle: std::f64::consts::PI * 0.1,
            ..Default::default()
        };
        Self {
            delta_time: 0.001,
            timer: Timer::new(vec![1e-7, 1.0 / 20e3]),
            sensor_noise: Noise::new(0.01, 0),
            current_noise: [
                Noise::new(0.01, 1),
                Noise::new(0.01, 2),
                Noise::new(0.01, 3),
            ],
            voltage_noise: [
                Noise::new(0.01, 1),
                Noise::new(0.01, 2),
                Noise::new(0.01, 3),
            ],
            current_offset: [0.01, 0.02, 0.03],
            voltage_offset: [0.02, -0.01, 0.01],
            motion_load: IdealMotionLoad { ..FAN_LOAD },
            sensor_observer,
            grad_observer,
            flux_observer,
            mix_observer,
            current_regulator: ThreePhasePICurrentRegulator {
                kp: [
                    0.8e3 * motor.inductance_dq[0],
                    0.8e3 * motor.inductance_dq[1],
                ],
                ki: [0.8e3 * motor.rs, 0.8e3 * motor.rs],
                ..Default::default()
            },
            motor,
            ..Default::default()
        }
    }

    pub fn update(&mut self) {
        self.timer.time += self.delta_time;
        while let Some(i) = self.timer.check_time() {
            let delta_time = self.timer.event_step[i];
            match i {
                0 => {
                    self.sensor_noise.update(delta_time);
                    for i in 0..3 {
                        self.current_noise[i].update(delta_time);
                        self.voltage_noise[i].update(delta_time);
                    }

                    self.motor_input.speed = self.motion_load_output.speed;
                    self.motor_input.angle = self.motion_load_output.angle;
                    for i in 0..3 {
                        self.motor_input.voltage[i] = self.power_bridge_output.voltage[i]
                            + self.voltage_offset[i]
                            + self.voltage_noise[i].value;
                    }
                    self.motion_load_input.torque = self.motor_output.torque;

                    self.power_bridge_output = self
                        .power_bridge
                        .update(delta_time, &self.power_bridge_input);
                    self.motor_output = self.motor.update(delta_time, &self.motor_input);
                    self.motion_load_output =
                        self.motion_load.update(delta_time, &self.motion_load_input);
                }
                1 => {
                    self.sensor_observer.sensor_angle =
                        self.motion_load.angle + self.sensor_noise.value;

                    // 实际需要电压重建
                    self.observer_input.voltage = self.power_bridge_output.voltage;
                    self.observer_input.current = self.motor_output.current;

                    self.sensor_observer_output = self
                        .sensor_observer
                        .update(delta_time, &self.observer_input);
                    self.grad_observer_output =
                        self.grad_observer.update(delta_time, &self.observer_input);
                    self.flux_observer_output =
                        self.flux_observer.update(delta_time, &self.observer_input);
                    self.mix_observer_output =
                        self.mix_observer.update(delta_time, &self.observer_input);

                    let use_observer = match self.use_observer {
                        0 => &self.sensor_observer_output,
                        1 => &self.grad_observer_output,
                        2 => &self.flux_observer_output,
                        3 => &self.mix_observer_output,
                        _ => &self.sensor_observer_output,
                    };
                    self.current_regulator_input.electrical_speed = use_observer.electrical_speed;
                    self.current_regulator_input.electrical_angle = use_observer.electrical_angle;
                    for i in 0..3 {
                        self.current_regulator_input.current[i] = self.motor_output.current[i]
                            + self.current_offset[i]
                            + self.current_noise[i].value;
                    }

                    let speed_error =
                        self.target_speed - self.current_regulator_input.electrical_speed;
                    self.speed_i += speed_error * 0.01 * delta_time;
                    let output_torque = self.speed_i + speed_error * 0.1;
                    let command_current = rotate(
                        [
                            0.0 * f64::cos(use_observer.electrical_angle) + 0.0,
                            output_torque,
                        ],
                        use_observer.electrical_angle,
                    );
                    self.current_regulator_input.command_current = inverse_clarke(command_current);
                    self.current_regulator_output = self
                        .current_regulator
                        .update(delta_time, &self.current_regulator_input);

                    self.power_bridge_input.command_voltage =
                        self.current_regulator_output.command_voltage;

                    let inject_voltage = inverse_clarke(rotate(
                        [0.0 * f64::sin(5000.0 * self.timer.time), 0.0],
                        self.current_regulator_input.electrical_angle,
                    ));
                    self.power_bridge_input.command_voltage[0] += inject_voltage[0];
                    self.power_bridge_input.command_voltage[1] += inject_voltage[1];
                    self.power_bridge_input.command_voltage[2] += inject_voltage[2];
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
                d.data
                    .drain(..d.data.len().saturating_sub(self.sample_count));
            }
        }
    }

    pub fn ui(&mut self, ui: &mut Ui) {
        let auto_bounds = ui.button("auto").clicked();
        ScrollArea::vertical().show(ui, |ui| {
            for (group_name, datas) in &self.data {
                CollapsingHeader::new(group_name)
                    .default_open(true)
                    .show(ui, |ui| {
                        Plot::new(group_name).height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
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
                    ScopeData::new("sensor_angle", |s: &mut Simulation| {
                        s.sensor_observer_output.electrical_angle
                    }),
                    ScopeData::new("flux_angle", |s: &mut Simulation| {
                        s.flux_observer_output.electrical_angle
                    }),
                    ScopeData::new("grad_angle", |s: &mut Simulation| {
                        s.grad_observer_output.electrical_angle
                    }),
                    ScopeData::new("mix_angle", |s: &mut Simulation| {
                        s.mix_observer_output.electrical_angle
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
                ],
            ),
            (
                "speed".to_string(),
                vec![
                    ScopeData::new("real_speed", |s: &mut Simulation| {
                        s.motion_load.speed * s.motor.pole_pairs
                    }),
                    ScopeData::new("sensor_speed", |s: &mut Simulation| {
                        s.sensor_observer_output.electrical_speed
                    }),
                    ScopeData::new("flux_speed", |s: &mut Simulation| {
                        s.flux_observer_output.electrical_speed
                    }),
                    ScopeData::new("grad_speed", |s: &mut Simulation| {
                        s.grad_observer_output.electrical_speed
                    }),
                    ScopeData::new("mix_speed", |s: &mut Simulation| {
                        s.mix_observer_output.electrical_speed
                    }),
                ],
            ),
            (
                "speed_error".to_string(),
                vec![
                    ScopeData::new("sensor_speed", |s: &mut Simulation| {
                        s.sensor_observer_output.electrical_speed
                            - s.motion_load.speed * s.motor.pole_pairs
                    }),
                    ScopeData::new("flux_speed", |s: &mut Simulation| {
                        s.flux_observer_output.electrical_speed
                            - s.motion_load.speed * s.motor.pole_pairs
                    }),
                    ScopeData::new("grad_speed", |s: &mut Simulation| {
                        s.grad_observer_output.electrical_speed
                            - s.motion_load.speed * s.motor.pole_pairs
                    }),
                    ScopeData::new("mix_speed", |s: &mut Simulation| {
                        s.mix_observer_output.electrical_speed
                            - s.motion_load.speed * s.motor.pole_pairs
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
                            clarke(s.power_bridge_output.voltage),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[0]
                    }),
                    ScopeData::new("uq", |s: &mut Simulation| {
                        rotate(
                            clarke(s.power_bridge_output.voltage),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[1]
                    }),
                ],
            ),
            (
                "sidq".to_string(),
                vec![
                    ScopeData::new("sid", |s: &mut Simulation| {
                        rotate(
                            clarke(s.current_regulator_input.current),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[0]
                    }),
                    ScopeData::new("siq", |s: &mut Simulation| {
                        rotate(
                            clarke(s.current_regulator_input.current),
                            -s.motion_load.angle * s.motor.pole_pairs,
                        )[1]
                    }),
                ],
            ),
            (
                "sdq".to_string(),
                vec![
                    ScopeData::new("sd", |s: &mut Simulation| s.mix_observer.sync_speed_lp[0]),
                    ScopeData::new("sq", |s: &mut Simulation| s.mix_observer.sync_speed_lp[1]),
                ],
            ),
            (
                "rs".to_string(),
                vec![
                    ScopeData::new("rs", |s: &mut Simulation| s.motor.rs),
                    ScopeData::new("grad_rs", |s: &mut Simulation| s.grad_observer.rs),
                ],
            ),
            (
                "l0".to_string(),
                vec![
                    ScopeData::new("l0", |s: &mut Simulation| {
                        (s.motor.inductance_dq[0] + s.motor.inductance_dq[1]) * 0.5
                    }),
                    ScopeData::new("grad_l0", |s: &mut Simulation| s.grad_observer.l0),
                ],
            ),
            (
                "l1".to_string(),
                vec![
                    ScopeData::new("l1", |s: &mut Simulation| {
                        (s.motor.inductance_dq[0] - s.motor.inductance_dq[1]) * 0.5
                    }),
                    ScopeData::new("grad_l1", |s: &mut Simulation| s.grad_observer.l1),
                ],
            ),
            (
                "flux".to_string(),
                vec![
                    ScopeData::new("flux", |s: &mut Simulation| s.motor.flux),
                    ScopeData::new("grad_flux", |s: &mut Simulation| s.grad_observer.flux),
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
    fn update(&mut self, ctx: &Context, frame: &mut Frame) {
        ctx.request_repaint();
        if self.run {
            for _ in 0..33 {
                self.update();
            }
        }
        CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Reset").clicked() {
                    self.reset();
                }
                ui.checkbox(&mut self.run, "Run");
                ui.label("target_speed");
                Slider::new(&mut self.simulation.target_speed, -300.0..=300.0).ui(ui);
                ui.label("use_observer");
                Slider::new(&mut self.simulation.use_observer, 0..=3).ui(ui);
            });
            ScrollArea::vertical().show(ui, |ui| {
                self.scope.ui(ui);
            });
        });
    }
}

fn main() {
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
