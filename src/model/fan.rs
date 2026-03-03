use crate::simulation::motion_load::ideal_motion_load::IdealMotionLoad;
use crate::simulation::motor::pmsm::PermanentMagnetSynchronousMotor;

pub const FAN_MOTOR: PermanentMagnetSynchronousMotor = PermanentMagnetSynchronousMotor {
    pole_pairs: 5.0,
    rs: 3.8,
    inductance_dq: [9.7e-3, 9.9e-3],
    flux: 0.04,
    current_dq: [0.0; 2],
};

pub const FAN_LOAD: IdealMotionLoad = IdealMotionLoad {
    inertia: 0.001,
    static_friction_torque: 1.0,
    kinetic_friction_factor: 0.01,
    angle: 0.0,
    speed: 0.0,
};
