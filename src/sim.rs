use crate::time::{InstantLike, Millis};
use nalgebra as na;

pub enum WaveForm {
    Sine,
    Square,
}

pub struct SignalGenerator<I: InstantLike> {
    fcn: fn(f64) -> f64,
    initial_time: I,
    amplitude: f64,
    offset: f64,
}

impl<I: InstantLike> SignalGenerator<I> {
    pub fn new(waveform: WaveForm, initial_time: I, amplitude: f64, offset: f64) -> Self {
        Self {
            fcn: match waveform {
                WaveForm::Sine => f64::sin,
                WaveForm::Square => |x| x.sin().signum(),
            },
            initial_time,
            amplitude,
            offset,
        }
    }

    pub fn generate(&self, time: I) -> f64 {
        self.amplitude * (self.fcn)(time.duration_since(self.initial_time).as_secs_f64())
            + self.offset
    }
}

pub fn sine_signal(time: Millis, initial_time: Millis) -> f64 {
    time.duration_since(initial_time).as_secs_f64().sin()
}

pub fn square_signal(time: Millis, initial_time: Millis) -> f64 {
    time.duration_since(initial_time)
        .as_secs_f64()
        .sin()
        .signum()
}

pub struct MassSpringDamper {
    pub natural_frequency: f64,
    pub damping_ratio: f64,
}

impl MassSpringDamper {
    /// Implements the state-space realization of the mass-spring-damper system:
    /// ┌     ┐   ┌              ┐┌    ┐   ┌     ┐
    /// │ p'  │ = │  0     1     ││ p  │ + │ 0   │ u
    /// │ p'' │   │  -ωₙ²  -2ζωₙ ││ p' │   │ ωₙ² │
    /// └     ┘   └              ┘└    ┘   └     ┘
    ///     ┌      ┐┌    ┐
    /// p = │ 1  0 ││ p  │         
    ///     └      ┘│ p' │
    ///             └    ┘
    pub fn f(&self, x: na::Vector2<f64>, u: f64) -> na::Vector2<f64> {
        let omega_sq = self.natural_frequency.powi(2);
        let two_zeta_omega = 2.0 * self.natural_frequency * self.damping_ratio;

        let mat_a = na::Matrix2::new(0.0, 1.0, -omega_sq, -two_zeta_omega);
        let mat_b = na::Vector2::new(0.0, omega_sq);

        mat_a * x + mat_b * u
    }

    pub fn h(&self, x: na::Vector2<f64>) -> f64 {
        x[0]
    }
}
