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
