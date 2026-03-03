use eframe::{App, CreationContext, Frame, NativeOptions};
use egui::{CentralPanel, CollapsingHeader, Context, ScrollArea};
use egui_plot::{Line, Plot, PlotPoint};
use moto_sim::model::fan::{FAN_LOAD, FAN_MOTOR};
use moto_sim::simulation::controller::current_regulator::three_phase_pi_current_regulator::ThreePhasePICurrentRegulator;
use moto_sim::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use moto_sim::simulation::controller::observer::flux_observer::FluxObserver;
use moto_sim::simulation::controller::observer::grad_observer::GradObserver;
use moto_sim::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use moto_sim::simulation::motion_load::ideal_motion_load::IdealMotionLoad;
use moto_sim::simulation::motion_load::{MotionLoad, MotionLoadInput, MotionLoadOutput};
use moto_sim::simulation::motor::pmsm::PermanentMagnetSynchronousMotor;
use moto_sim::simulation::motor::{Motor, MotorInput, MotorOutput};
use moto_sim::simulation::power_bridge::ideal_power_bridge::IdealPowerBridge;
use moto_sim::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};
use moto_sim::simulation::{angle_normal, inverse_clarke, rotate};
use moto_sim::ui::font::load_chinese_font;
use moto_sim::util::Timer;

#[derive(Debug, Default, Clone)]
pub struct Simulation {
    pub delta_time: f64,
    pub timer: Timer,

    power_bridge_input: PowerBridgeInput<3>,
    power_bridge_output: PowerBridgeOutput<3>,
    motor_input: MotorInput<3>,
    motor_output: MotorOutput<3>,
    motion_load_input: MotionLoadInput,
    motion_load_output: MotionLoadOutput,
    observer_input: ObserverInput<3>,
    observer_output: ObserverOutput,
    current_regulator_input: CurrentRegulatorInput<3>,
    current_regulator_output: CurrentRegulatorOutput<3>,

    pub power_bridge: IdealPowerBridge,
    pub motor: PermanentMagnetSynchronousMotor,
    pub motion_load: IdealMotionLoad,
    pub observer: GradObserver,
    pub current_regulator: ThreePhasePICurrentRegulator,
}

impl Simulation {
    pub fn new() -> Self {
        let motor = PermanentMagnetSynchronousMotor { ..FAN_MOTOR };
        let observer = FluxObserver {
            rs: motor.rs,
            flux: motor.flux,
            inductance_dq: motor.inductance_dq,
            speed_lp_factor: 100.0,
            position_factor: 100.0,
            position: [0.0, 0.0],
            ..Default::default()
        };
        let observer = GradObserver {
            rs: motor.rs,
            l0: (motor.inductance_dq[0] + motor.inductance_dq[1]) * 0.5,
            l1: (motor.inductance_dq[0] - motor.inductance_dq[1]) * 0.5,
            flux: motor.flux,
            kr: 0.0,
            kl0: 0.0001,
            kflux: 0.0,
            kl1_ds: 0.0,
            kl1_de: 0.0,

            kspeed_l1: 1.0,
            kspeed_flux: 10000.0,
            kspeed_kangle: 1.0,
            kangle_l1_di: 1000.0,
            kangle_l1_ds: 1000.0,
            kangle_flux: 10.0,

            angle: std::f64::consts::PI * 0.2,
            ..Default::default()
        };
        Self {
            delta_time: 0.0001,
            timer: Timer::new(vec![1e-7, 1.0 / 20e3]),
            motion_load: IdealMotionLoad { ..FAN_LOAD },
            observer,
            current_regulator: ThreePhasePICurrentRegulator {
                kp: [10e3 * motor.inductance_dq[0], 10e3 * motor.inductance_dq[1]],
                ki: [10e3 * motor.rs, 10e3 * motor.rs],
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
                    self.motor_input.speed = self.motion_load_output.speed;
                    self.motor_input.angle = self.motion_load_output.angle;
                    self.motor_input.voltage = self.power_bridge_output.voltage;
                    self.motion_load_input.torque = self.motor_output.torque;

                    self.power_bridge_output = self
                        .power_bridge
                        .update(delta_time, &self.power_bridge_input);
                    self.motor_output = self.motor.update(delta_time, &self.motor_input);
                    self.motion_load_output =
                        self.motion_load.update(delta_time, &self.motion_load_input);
                }
                1 => {
                    // 仅有感
                    // self.observer.sensor_angle = self.motion_load.angle;

                    // 实际需要电压重建
                    self.observer_input.voltage = self.power_bridge_output.voltage;
                    self.observer_input.current = self.motor_output.current;

                    // self.observer.angle = self.motion_load_output.angle * self.motor.pole_pairs;
                    // self.observer.speed = self.motion_load_output.speed * self.motor.pole_pairs;
                    self.observer_output = self.observer.update(delta_time, &self.observer_input);

                    self.current_regulator_input.current = self.motor_output.current;
                    self.current_regulator_input.electrical_speed =
                        self.observer_output.electrical_speed;
                    self.current_regulator_input.electrical_angle =
                        self.observer_output.electrical_angle;

                    self.current_regulator_input.command_current =
                        inverse_clarke(rotate([0.0, 1.0], self.observer_output.electrical_angle));
                    self.current_regulator_output = self
                        .current_regulator
                        .update(delta_time, &self.current_regulator_input);

                    self.power_bridge_input.command_voltage =
                        self.current_regulator_output.command_voltage;
                    self.power_bridge_input.command_voltage[0] +=
                        f64::sin(self.timer.time * std::f64::consts::TAU * 1000.0) * 10.0;
                    self.power_bridge_input.command_voltage[1] +=
                        f64::cos(self.timer.time * std::f64::consts::TAU * 1000.0) * 10.0;
                }
                _ => unreachable!(),
            }
        }
    }
}

pub struct Application {
    simulation: Simulation,
    angle: Vec<PlotPoint>,
    observer_angle: Vec<PlotPoint>,
    speed: Vec<PlotPoint>,
    observer_speed: Vec<PlotPoint>,
    run: bool,
    id: Vec<PlotPoint>,
    iq: Vec<PlotPoint>,
    fx: Vec<PlotPoint>,
    fy: Vec<PlotPoint>,
    rs: Vec<PlotPoint>,
    l0: Vec<PlotPoint>,
    l1: Vec<PlotPoint>,
    flux: Vec<PlotPoint>,
}

impl Application {
    pub fn new(cc: &CreationContext) -> anyhow::Result<Self> {
        load_chinese_font(&cc.egui_ctx);
        Ok(Self {
            simulation: Simulation::new(),
            angle: Vec::new(),
            observer_angle: Vec::new(),
            speed: Vec::new(),
            observer_speed: Vec::new(),
            run: false,
            id: Vec::new(),
            iq: Vec::new(),
            fx: Vec::new(),
            fy: Vec::new(),
            rs: Vec::new(),
            l0: Vec::new(),
            l1: Vec::new(),
            flux: Vec::new(),
        })
    }

    fn update(&mut self) {
        self.simulation.update();

        let sample = 10000;

        self.angle.drain(..self.angle.len().saturating_sub(sample));
        self.observer_angle
            .drain(..self.observer_angle.len().saturating_sub(sample));
        self.speed.drain(..self.speed.len().saturating_sub(sample));
        self.observer_speed
            .drain(..self.observer_speed.len().saturating_sub(sample));
        self.id.drain(..self.id.len().saturating_sub(sample));
        self.iq.drain(..self.iq.len().saturating_sub(sample));
        self.fx.drain(..self.fx.len().saturating_sub(sample));
        self.fy.drain(..self.fy.len().saturating_sub(sample));
        self.rs.drain(..self.rs.len().saturating_sub(sample));
        self.l0.drain(..self.l0.len().saturating_sub(sample));
        self.l1.drain(..self.l1.len().saturating_sub(sample));
        self.flux.drain(..self.flux.len().saturating_sub(sample));

        self.angle.push(PlotPoint::new(
            self.simulation.timer.time,
            angle_normal(self.simulation.motion_load.angle * self.simulation.motor.pole_pairs),
        ));
        self.observer_angle.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer_output.electrical_angle,
        ));
        self.speed.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.motion_load.speed * self.simulation.motor.pole_pairs,
        ));
        self.observer_speed.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer_output.electrical_speed,
        ));

        self.id.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.motor.current_dq[0],
        ));
        self.iq.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.motor.current_dq[1],
        ));

        self.fx.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.error[0],
        ));
        self.fy.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.error[1],
        ));
        self.rs.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.rs,
        ));
        self.l0.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.l0,
        ));
        self.l1.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.l1,
        ));
        self.flux.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.flux,
        ));
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
        let mut auto_bounds = false;
        CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                auto_bounds = ui.button("auto").clicked();
                ui.checkbox(&mut self.run, "Run");
            });
            ScrollArea::vertical().show(ui, |ui| {
                CollapsingHeader::new("angle")
                    .default_open(true)
                    .show(ui, |ui| {
                        Plot::new("angle").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("angle", self.angle.as_slice()));
                            ui.line(Line::new("observer_angle", self.observer_angle.as_slice()));
                        });
                    });
                CollapsingHeader::new("speed")
                    .default_open(true)
                    .show(ui, |ui| {
                        Plot::new("speed").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("speed", self.speed.as_slice()));
                            ui.line(Line::new("observer_speed", self.observer_speed.as_slice()));
                        });
                    });
                CollapsingHeader::new("current_dq")
                    .default_open(true)
                    .show(ui, |ui| {
                        Plot::new("current_dq").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("id", self.id.as_slice()));
                            ui.line(Line::new("iq", self.iq.as_slice()));
                        });
                    });
                CollapsingHeader::new("observer")
                    .default_open(true)
                    .show(ui, |ui| {
                        Plot::new("observer").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("fx", self.fx.as_slice()));
                            ui.line(Line::new("fy", self.fy.as_slice()));
                        });
                        Plot::new("observer_rs").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("rs", self.rs.as_slice()));
                        });
                        Plot::new("observer_l").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("l0", self.l0.as_slice()));
                            ui.line(Line::new("l1", self.l1.as_slice()));
                        });
                        Plot::new("observer_f").height(120.0).show(ui, |ui| {
                            if auto_bounds {
                                ui.set_auto_bounds(true);
                            }
                            ui.line(Line::new("flux", self.flux.as_slice()));
                        });
                    });
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
