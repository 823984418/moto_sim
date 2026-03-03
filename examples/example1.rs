use eframe::{App, CreationContext, Frame, NativeOptions};
use egui::{CentralPanel, CollapsingHeader, Context, ScrollArea};
use egui_plot::{Line, Plot, PlotPoint};
use moto_sim::model::fan::{FAN_LOAD, FAN_MOTOR};
use moto_sim::simulation::controller::current_regulator::three_phase_pi_current_regulator::ThreePhasePICurrentRegulator;
use moto_sim::simulation::controller::current_regulator::{
    CurrentRegulator, CurrentRegulatorInput, CurrentRegulatorOutput,
};
use moto_sim::simulation::controller::observer::sensor_observer::SensorObserver;
use moto_sim::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use moto_sim::simulation::motion_load::ideal_motion_load::IdealMotionLoad;
use moto_sim::simulation::motion_load::{MotionLoad, MotionLoadInput, MotionLoadOutput};
use moto_sim::simulation::motor::pmsm::PermanentMagnetSynchronousMotor;
use moto_sim::simulation::motor::{Motor, MotorInput, MotorOutput};
use moto_sim::simulation::power_bridge::ideal_power_bridge::IdealPowerBridge;
use moto_sim::simulation::power_bridge::{PowerBridge, PowerBridgeInput, PowerBridgeOutput};
use moto_sim::simulation::{clarke, inverse_clarke, rotate};
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
    pub observer: SensorObserver,
    pub current_regulator: ThreePhasePICurrentRegulator,
}

impl Simulation {
    pub fn new() -> Self {
        let motor = PermanentMagnetSynchronousMotor { ..FAN_MOTOR };
        Self {
            delta_time: 1.0 / 60.0,
            timer: Timer::new(vec![1e-6, 1.0 / 20e3]),
            motion_load: IdealMotionLoad { ..FAN_LOAD },
            observer: SensorObserver {
                pole_pairs: motor.pole_pairs,
                pll_kp: 10.0,
                pll_ki: 1.0,
                ..Default::default()
            },
            current_regulator: ThreePhasePICurrentRegulator {
                kp: [1e3 * motor.inductance_dq[0], 1e3 * motor.inductance_dq[0]],
                ki: [1e3 * motor.rs, 1e3 * motor.rs],
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
                    self.observer.sensor_angle = self.motion_load.angle;

                    self.observer_input.current = self.motor_output.current;
                    // 实际需要重建
                    self.observer_input.voltage = self.power_bridge_output.voltage;

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
    id: Vec<PlotPoint>,
    iq: Vec<PlotPoint>,
    ca: Vec<PlotPoint>,
    cb: Vec<PlotPoint>,
}

impl Application {
    pub fn new(cc: &CreationContext) -> anyhow::Result<Self> {
        load_chinese_font(&cc.egui_ctx);
        Ok(Self {
            simulation: Simulation::new(),
            angle: Vec::new(),
            observer_angle: Vec::new(),
            speed: Vec::new(),
            id: Vec::new(),
            iq: Vec::new(),
            ca: Vec::new(),
            cb: Vec::new(),
        })
    }
}

impl App for Application {
    fn update(&mut self, ctx: &Context, frame: &mut Frame) {
        ctx.request_repaint();

        self.simulation.update();

        self.angle.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.motion_load.angle,
        ));
        self.observer_angle.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.observer.pll_angle,
        ));
        self.speed.push(PlotPoint::new(
            self.simulation.timer.time,
            self.simulation.motion_load.speed,
        ));

        let iab = rotate(
            self.simulation.motor.current_dq,
            self.simulation.observer_output.electrical_angle,
        );
        let iab = clarke(self.simulation.current_regulator_input.current);
        self.id
            .push(PlotPoint::new(self.simulation.timer.time, iab[0]));
        self.iq
            .push(PlotPoint::new(self.simulation.timer.time, iab[1]));
        let cab = clarke(self.simulation.current_regulator_input.command_current);

        self.ca
            .push(PlotPoint::new(self.simulation.timer.time, cab[0]));
        self.cb
            .push(PlotPoint::new(self.simulation.timer.time, cab[1]));

        CentralPanel::default().show(ctx, |ui| {
            let auto_bounds = ui.button("auto").clicked();
            ScrollArea::vertical().show(ui, |ui| {
                CollapsingHeader::new("angle").show(ui, |ui| {
                    Plot::new("angle").height(100.0).show(ui, |ui| {
                        if auto_bounds {
                            ui.set_auto_bounds(true);
                        }
                        ui.line(Line::new("angle", self.angle.as_slice()));
                        ui.line(Line::new("observer_angle", self.observer_angle.as_slice()));
                    });
                });
                CollapsingHeader::new("speed").show(ui, |ui| {
                    Plot::new("speed").height(100.0).show(ui, |ui| {
                        if auto_bounds {
                            ui.set_auto_bounds(true);
                        }
                        ui.line(Line::new("speed", self.speed.as_slice()));
                    });
                });
                CollapsingHeader::new("current_dq").show(ui, |ui| {
                    Plot::new("current_dq").height(100.0).show(ui, |ui| {
                        if auto_bounds {
                            ui.set_auto_bounds(true);
                        }
                        ui.line(Line::new("id", self.id.as_slice()));
                        ui.line(Line::new("iq", self.iq.as_slice()));
                        ui.line(Line::new("ca", self.ca.as_slice()));
                        ui.line(Line::new("cb", self.cb.as_slice()));
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
