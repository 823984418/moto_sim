use moto_sim::controller::observer::sensor_observer::SensorObserver;
use moto_sim::motion::ideal_motion_load::IdealMotionLoad;
use moto_sim::motor::pmsm::PermanentMagnetSynchronousMotor;

fn main() {
    let mut motion = IdealMotionLoad {
        inertia: 1.0,
        static_friction_torque: 1.0,
        kinetic_friction_factor: 1.0,
        ..Default::default()
    };
    let mut motor = PermanentMagnetSynchronousMotor {
        pole_pairs: 1.0,
        inductance_dq: [0.01, 0.01],
        ..Default::default()
    };
    let mut observer = SensorObserver {
        pole_pairs: 1.0,
        pll_kp: 0.1,
        pll_ki: 0.01,
        ..Default::default()
    };
}
