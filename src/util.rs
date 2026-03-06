use std::fmt::{Debug, Formatter};
use std::ops::{Deref, DerefMut};

use crate::simulation::nn;

#[derive(Debug, Default, Clone)]
pub struct Timer {
    pub event_time: Vec<f64>,
    pub event_step: Vec<f64>,
    pub time: f64,
}

impl Timer {
    pub fn new(event_step: Vec<f64>) -> Self {
        Self {
            event_time: vec![0.0; event_step.len()],
            event_step,
            time: 0.0,
        }
    }

    pub fn check_time(&mut self) -> Option<usize> {
        if let Some((event, &time)) = self
            .event_time
            .iter()
            .enumerate()
            .min_by(|a, b| f64::total_cmp(a.1, b.1))
        {
            if time <= self.time {
                self.event_time[event] += self.event_step[event];
                return Some(event);
            }
        }
        None
    }
}

pub struct BoxedFunction(Box<dyn Function>);

impl<T: Function + 'static> From<T> for BoxedFunction {
    fn from(value: T) -> Self {
        Self(Box::new(value))
    }
}

impl Deref for BoxedFunction {
    type Target = dyn Function;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl DerefMut for BoxedFunction {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

impl Debug for BoxedFunction {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("BoxedFunction").finish()
    }
}

impl Clone for BoxedFunction {
    fn clone(&self) -> Self {
        Self(self.0.clone_to_boxed())
    }
}

impl Default for BoxedFunction {
    fn default() -> Self {
        Self(Box::new(|_| 0.0))
    }
}

pub trait Function {
    fn eval(&self, x: f64) -> f64;

    fn clone_to_boxed<'s>(&self) -> Box<dyn Function + 's>
    where
        Self: 's;
}

impl<F: Fn(f64) -> f64 + Clone> Function for F {
    fn eval(&self, x: f64) -> f64 {
        self(x)
    }

    fn clone_to_boxed<'s>(&self) -> Box<dyn Function + 's>
    where
        Self: 's,
    {
        Box::new(self.clone())
    }
}

#[derive(Debug, Default, Clone)]
pub struct LinearInterpolation {
    samples: Vec<(f64, f64)>,
}

impl LinearInterpolation {
    pub fn new(mut samples: Vec<(f64, f64)>) -> Self {
        samples.sort_by(|a, b| f64::total_cmp(&a.0, &b.0));
        Self { samples }
    }

    pub fn eval(&self, x: f64) -> f64 {
        if self.samples.is_empty() {
            return f64::NAN;
        }
        let search = self
            .samples
            .binary_search_by(|(v, _)| f64::total_cmp(v, &x));
        match search {
            Ok(i) => self.samples[i].1,
            Err(i) => {
                if i == 0 {
                    return self.samples[0].1;
                }
                if i >= self.samples.len() - 1 {
                    return self.samples[self.samples.len() - 1].1;
                }
                let left = &self.samples[i - 1];
                let right = &self.samples[i];

                nn((right.1 - left.1) / (right.0 - left.0) * (x - right.0)) + right.1
            }
        }
    }
}

impl Function for LinearInterpolation {
    fn eval(&self, x: f64) -> f64 {
        self.eval(x)
    }

    fn clone_to_boxed<'s>(&self) -> Box<dyn Function + 's>
    where
        Self: 's,
    {
        Box::new(self.clone())
    }
}
