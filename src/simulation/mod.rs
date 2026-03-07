pub mod controller;
pub mod motion_load;
pub mod motor;
pub mod noise;
pub mod power_bridge;

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
        (v[0] + v[0] - v[1] - v[2]) * const { 1.0 / 3.0 },
        (v[1] - v[2]) * FRAC_1_SQRT_3,
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

pub fn nn(x: f64) -> f64 {
    if x.is_nan() { 0.0 } else { x }
}

pub fn complex_div(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let s = b[0] * b[0] + b[1] * b[1] + f64::EPSILON;
    [
        nn((a[0] * b[0] + a[1] * b[1]) / s),
        nn((a[1] * b[0] - a[0] * b[1]) / s),
    ]
}

pub fn atan2(a: [f64; 2]) -> f64 {
    nn(f64::atan2(a[1], a[0]))
}

pub fn mtpa_id(iq: f64, flux: f64, ld: f64, lq: f64) -> f64 {
    let flux_div_2_lq_sub_ld = 0.5 * flux / (lq - ld);
    flux_div_2_lq_sub_ld - f64::sqrt(flux_div_2_lq_sub_ld * flux_div_2_lq_sub_ld + iq * iq)
}

pub fn complex_mode(a: [f64; 2]) -> f64 {
    f64::sqrt(a[0] * a[0] + a[1] * a[1])
}
