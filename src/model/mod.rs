use crate::util::{BoxedFunction, LinearInterpolation};

pub mod fan;

pub fn zq10y_dead_mapping() -> BoxedFunction {
    let iq14_base: f64 = 2.0f64.powi(-14);
    LinearInterpolation::new(vec![
        (0.0, 0.0),
        (0.007, 245e-5 * iq14_base),
        (-0.007, -245e-5 * iq14_base),
        (0.07, 960e-5 * iq14_base),
        (-0.07, -960e-5 * iq14_base),
        (0.14, 1327e-5 * iq14_base),
        (-0.14, -1327e-5 * iq14_base),
        (0.28, 1503e-5 * iq14_base),
        (-0.28, -1503e-5 * iq14_base),
        (0.42, 1650e-5 * iq14_base),
        (-0.42, -1650e-5 * iq14_base),
        (0.56, 1774e-5 * iq14_base),
        (-0.56, -1774e-5 * iq14_base),
        (1.0, 1800e-5 * iq14_base),
        (-1.0, -1800e-5 * iq14_base),
        (2.0, 1900e-5 * iq14_base),
        (-2.0, -1900e-5 * iq14_base),
    ])
    .into()
}
