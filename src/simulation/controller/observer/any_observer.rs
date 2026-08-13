use std::collections::VecDeque;
use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::simulation::controller::observer::{Observer, ObserverInput, ObserverOutput};
use crate::simulation::{angle_normal, atan2, clarke};

// 前向传播其关于电阻的梯度与关于电感的梯度
#[derive(Debug, Copy, Clone)]
pub struct Val {
    // 值
    pub value: f64,
    // 电阻梯度
    pub grad_resistor: f64,
}

const fn epsilon(x: f64) -> f64 {
    if x.is_sign_positive() {
        x.max(f64::EPSILON)
    } else {
        x.min(-f64::EPSILON)
    }
}

impl Val {
    pub const ZERO: Self = Val {
        value: 0.0,
        grad_resistor: 0.0,
    };

    pub const fn new(value: f64) -> Self {
        Self {
            value,
            grad_resistor: 0.0,
        }
    }

    pub const fn resistor(resistor: f64) -> Self {
        Self {
            value: resistor,
            grad_resistor: 1.0,
        }
    }

    pub const fn pow2(self) -> Self {
        let a = 2.0 * self.value;
        Self {
            value: self.value * self.value,
            grad_resistor: self.grad_resistor * a,
        }
    }

    pub const fn inv(self) -> Self {
        let value = 1.0 / epsilon(self.value);
        let a = -value * value;
        Self {
            value,
            grad_resistor: self.grad_resistor * a,
        }
    }

    pub fn sqrt(self) -> Self {
        let value = self.value.sqrt();
        let a = 0.5 / value;
        Self {
            value,
            grad_resistor: self.grad_resistor * a,
        }
    }
}

impl Add<Val> for Val {
    type Output = Val;

    fn add(self, rhs: Val) -> Self::Output {
        Val {
            value: self.value + rhs.value,
            grad_resistor: self.grad_resistor + rhs.grad_resistor,
        }
    }
}

impl Sub<Val> for Val {
    type Output = Val;

    fn sub(self, rhs: Val) -> Self::Output {
        Val {
            value: self.value - rhs.value,
            grad_resistor: self.grad_resistor - rhs.grad_resistor,
        }
    }
}

impl Mul<Val> for Val {
    type Output = Val;

    fn mul(self, rhs: Val) -> Self::Output {
        let a = rhs.value;
        let b = self.value;
        Val {
            value: self.value * rhs.value,
            grad_resistor: self.grad_resistor * a + rhs.grad_resistor * b,
        }
    }
}

impl Div<Val> for Val {
    type Output = Val;

    fn div(self, rhs: Val) -> Self::Output {
        let a = 1.0 / epsilon(rhs.value);
        let value = self.value * a;
        let b = value * a;
        Val {
            value,
            grad_resistor: self.grad_resistor * a - rhs.grad_resistor * b,
        }
    }
}

impl Neg for Val {
    type Output = Val;

    fn neg(self) -> Self::Output {
        Val {
            value: -self.value,
            grad_resistor: -self.grad_resistor,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct AnyObserverSample {
    /// 采样电流
    pub i: [f64; 2],
    /// 电流积分
    pub is: [f64; 2],
    /// 电压积分
    pub us: [f64; 2],
}

#[derive(Debug, Default, Clone)]
pub struct AnyObserver {
    /// 累积的采样数
    pub sample: VecDeque<AnyObserverSample>,
    /// 保留的历史记录数
    pub history: usize,

    /// 估计电感
    pub inductor: f64,

    /// 估计电阻
    pub resistor: f64,
    pub resistor_rate: f64,
    pub resistor_range: (f64, f64),

    pub max_flux: f64,

    /// 估计磁链
    pub flux: f64,
    /// 呈现的采样点
    pub sample_point: Vec<[Val; 2]>,
    /// 采样点中心
    pub sample_point_center: [f64; 2],
}

impl AnyObserver {
    pub const fn new() -> Self {
        Self {
            sample: VecDeque::new(),
            history: 2,
            resistor: 0.0,
            resistor_rate: 1.0,
            resistor_range: (0.0, 100.0),
            inductor: 0.0,
            max_flux: 1.0,
            flux: 0.0,
            sample_point: Vec::new(),
            sample_point_center: [0.0; 2],
        }
    }

    /// sample 中的 is 与 us 是自上次 update 后重新开始的的积分值
    pub fn update(&mut self, sample: AnyObserverSample) {
        self.sample.truncate(self.history);
        for s in &mut self.sample {
            s.is[0] -= sample.is[0];
            s.is[1] -= sample.is[1];
            s.us[0] -= sample.us[0];
            s.us[1] -= sample.us[1];
        }
        self.sample.push_front(AnyObserverSample {
            i: sample.i,
            is: [0.0; 2],
            us: [0.0; 2],
        });

        if self.sample.len() < 3 {
            return;
        }

        let inv_n = 1.0 / (self.sample.len() as f64);

        let inductor = self.inductor;
        let resistor = Val::resistor(self.resistor);

        let sample_point_buffer = &mut self.sample_point;
        sample_point_buffer.clear();
        sample_point_buffer.extend(self.sample.iter().map(move |x: &AnyObserverSample| {
            [
                Val::new(x.us[0] - x.i[0] * inductor) - Val::new(x.is[0]) * resistor,
                Val::new(x.us[1] - x.i[1] * inductor) - Val::new(x.is[1]) * resistor,
            ]
        }));

        let (sum_x, sum_y) = sample_point_buffer
            .iter()
            .fold((Val::ZERO, Val::ZERO), move |(sum_x, sum_y), &[sx, sy]| {
                (sum_x + sx, sum_y + sy)
            });
        let avg_x = sum_x * Val::new(inv_n);
        let avg_y = sum_y * Val::new(inv_n);

        let max_flux2 = self.max_flux * self.max_flux;

        // 每相邻三个采样点的权重和圆心
        let d_cxy =
            sample_point_buffer
                .array_windows::<3>()
                .map(move |&[[x1, y1], [x2, y2], [x3, y3]]| {
                    let dy1 = y2 - y3;
                    let dy2 = y3 - y1;
                    let dy3 = y1 - y2;
                    let p1 = x1.pow2() + y1.pow2();
                    let p2 = x2.pow2() + y2.pow2();
                    let p3 = x3.pow2() + y3.pow2();
                    let dx1 = x3 - x2;
                    let dx2 = x1 - x3;
                    let dx3 = x2 - x1;
                    let d = Val::new(2.0) * (x1 * dy1 + x2 * dy2 + x3 * dy3);
                    let inv_d = d.inv();
                    let mut cx = (p1 * dy1 + p2 * dy2 + p3 * dy3) * inv_d;
                    let mut cy = (p1 * dx1 + p2 * dx2 + p3 * dx3) * inv_d;

                    let acx = cx.value - avg_x.value;
                    let acy = cy.value - avg_y.value;
                    if (acx * acx + acy * acy) > max_flux2 {
                        cx = avg_x;
                        cy = avg_y;
                    }

                    (d.pow2(), [cx, cy])
                });

        // 计算圆心的期望
        let (sum_d, [sum_dcx, sum_dcy]) = d_cxy.fold(
            (Val::ZERO, [Val::ZERO; 2]),
            move |(sum_d, [sum_dcx, sum_dcy]): (Val, [Val; 2]), (d, [cx, cy]): (Val, [Val; 2])| {
                (sum_d + d, [sum_dcx + cx * d, sum_dcy + cy * d])
            },
        );
        let inv_sum_d = sum_d.inv();
        let cx = sum_dcx * inv_sum_d;
        let cy = sum_dcy * inv_sum_d;

        // 计算距离圆心的距离
        let cr = <[_]>::iter(sample_point_buffer).map(move |&[x, y]: &[Val; 2]| {
            let dx = x - cx;
            let dy = y - cy;
            (dx.pow2() + dy.pow2()).sqrt()
        });

        let (sum_cr, sum_cr2) = cr.fold(
            (Val::ZERO, Val::ZERO),
            move |(sum_cr, sum_cr2): (Val, Val), cr: Val| (sum_cr + cr, sum_cr2 + cr.pow2()),
        );

        let cr_avg = sum_cr * Val::new(inv_n);
        let cr2_avg = sum_cr2 * Val::new(inv_n);
        let cr_var = cr2_avg - cr_avg.pow2();

        let resistor = (self.resistor - cr_var.grad_resistor * self.resistor_rate)
            .clamp(self.resistor_range.0, self.resistor_range.1);

        self.sample_point_center = [cx.value, cy.value];
        self.resistor = resistor;
        self.flux = cr_avg.value;
    }
}

#[derive(Debug, Default, Clone)]
pub struct AllObserver {
    pub time: f64,
    pub delta_time: f64,
    pub is: [f64; 2],
    pub us: [f64; 2],
    pub any_observer: AnyObserver,
    pub angle: f64,
    pub speed_lp: f64,
    pub speed_lp_factor: f64,
}

impl Observer<3> for AllObserver {
    fn update(&mut self, delta_time: f64, input: &ObserverInput<3>) -> ObserverOutput {
        let current = clarke(input.current);
        let voltage = clarke(input.voltage);
        self.time += delta_time;
        self.is[0] += delta_time * current[0];
        self.is[1] += delta_time * current[1];
        self.us[0] += delta_time * voltage[0];
        self.us[1] += delta_time * voltage[1];

        if self.time > self.delta_time {
            self.time -= self.delta_time;
            self.any_observer.update(AnyObserverSample {
                i: current,
                is: self.is,
                us: self.us,
            });
            self.is = [0.0; 2];
            self.us = [0.0; 2];
        }

        let flux_x = self.us[0]
            - self.is[0] * self.any_observer.resistor
            - current[0] * self.any_observer.inductor
            - self.any_observer.sample_point_center[0];

        let flux_y = self.us[1]
            - self.is[1] * self.any_observer.resistor
            - current[1] * self.any_observer.inductor
            - self.any_observer.sample_point_center[1];

        let angle = atan2([flux_x, flux_y]);
        let speed = angle_normal(angle - self.angle) / delta_time;
        self.angle = angle;
        self.speed_lp += (speed - self.speed_lp) * self.speed_lp_factor * delta_time;

        ObserverOutput {
            electrical_angle: angle,
            electrical_speed: self.speed_lp,
            continuous_speed: self.speed_lp,
        }
    }
}
