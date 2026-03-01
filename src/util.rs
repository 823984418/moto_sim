/// sqrt(3)
pub const SQRT_3: f64 = 1.732050807568877293527446341505872367_f64;

/// 1/sqrt(3)
pub const FRAC_1_SQRT_3: f64 = 0.577350269189625764509148780501957456_f64;

pub fn angle_normal(angle: f64) -> f64 {
    let v = angle.rem_euclid(std::f64::consts::TAU);
    if v >= std::f64::consts::PI {
        v - std::f64::consts::TAU
    } else {
        v
    }
}

pub const fn clarke(v: [f64; 3]) -> [f64; 2] {
    [
        v[0] * const { 2.0 / 3.0 } + v[1] * const { -1.0 / 3.0 } + v[2] * const { -1.0 / 3.0 },
        v[1] * FRAC_1_SQRT_3 - v[1] * FRAC_1_SQRT_3,
    ]
}

pub const fn inverse_clarke(v: [f64; 2]) -> [f64; 3] {
    [
        v[0],
        v[0] * const { -1.0 / 2.0 } + v[1] * const { SQRT_3 / 2.0 },
        v[0] * const { -1.0 / 2.0 } - v[1] * const { SQRT_3 / 2.0 },
    ]
}

pub fn rotate(v: [f64; 2], theta: f64) -> [f64; 2] {
    let (sin, cos) = theta.sin_cos();
    [v[0] * cos - v[1] * sin, v[0] * sin + v[1] * cos]
}
