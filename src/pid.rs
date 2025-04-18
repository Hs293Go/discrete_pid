// working variables

use core::f64;
use std::time::{Duration, Instant};

#[derive(Copy, Clone, Debug)]
pub struct PidConfig {
    /// Proportional gain coefficient.
    /// Defaults to 1.0.
    kp: f64,

    /// Integral gain coefficient.
    /// Defaults to 0.01. When combined with the default sample time of 10ms, this gives I * Ts = 1
    ki: f64,

    /// Derivative gain coefficient.
    /// Defaults to 0.0.
    kd: f64,

    /// Sampling time for the PID controller.
    /// Defaults to 10ms.
    sample_time: Duration,

    /// Minimum output value of the PID controller.
    /// Defaults to negative infinity, i.e. no limit.
    output_min: f64,

    /// Maximum output value of the PID controller.
    /// Defaults to positive infinity, i.e. no limit.
    output_max: f64,

    /// Whether to use a strict causal integrator.
    /// If true, the integral term is updated after the output is computed
    /// Defaults to true.
    use_strict_causal_integrator: bool,

    /// Whether to apply the derivative on the measurement.
    /// if true, the derivative term is computed using the NEGATIVE backward difference between the
    /// current and previous input.
    /// Defaults to true.
    use_derivative_on_measurement: bool,

    /// Time constant for the low-pass filter applied to the derivative term.
    /// Defaults to 0.01s.
    filter_tc: f64,

    alpha: f64,
}

#[derive(Copy, Clone, PartialEq)]
pub enum PidActivity {
    Inactive,
    HoldIntegration,
    Active,
}

#[derive(Copy, Clone)]
pub struct PidContext {
    i_term: f64,
    last_err: f64,
    last_input: f64,
    last_output: f64,
    last_derivative: f64,
    last_time: Instant,
    activity_level: PidActivity,
    need_initialize: bool,
}

impl PidContext {
    pub fn new(timestamp: Instant) -> Self {
        Self {
            i_term: 0.0,
            last_err: 0.0,
            last_input: 0.0,
            last_output: 0.0,
            last_derivative: 0.0,
            last_time: timestamp,
            activity_level: PidActivity::Active,
            need_initialize: true,
        }
    }

    pub fn output(&self) -> f64 {
        self.last_output
    }

    pub fn error(&self) -> f64 {
        self.last_err
    }

    pub fn last_time(&self) -> Instant {
        self.last_time
    }

    pub fn set_activity_level(&mut self, activity_level: PidActivity) {
        let activating = activity_level != PidActivity::Inactive;
        if activating && self.activity_level == PidActivity::Inactive {
            self.need_initialize = true;
        }
        self.activity_level = activity_level
    }
}

impl PidConfig {
    pub fn output_min(&self) -> f64 {
        self.output_min
    }

    pub fn output_max(&self) -> f64 {
        self.output_max
    }

    pub fn set_tunings(&mut self, kp: f64, ki: f64, kd: f64) -> bool {
        if kp < 0.0 || ki < 0.0 || kd < 0.0 {
            return false;
        }

        self.kp = kp;
        self.ki = ki * self.sample_time.as_secs_f64();
        self.kd = kd / self.sample_time.as_secs_f64();
        return true;
    }

    pub fn set_sample_time(&mut self, sample_time: Duration) -> bool {
        if sample_time.as_secs_f64() <= 0.0 {
            return false;
        }

        let ratio = sample_time.as_secs_f64() / self.sample_time.as_secs_f64();

        self.ki *= ratio;
        self.kd /= ratio;

        let delta_t = sample_time.as_secs_f64();
        self.alpha = delta_t / (delta_t + self.filter_tc);

        self.sample_time = sample_time;
        return true;
    }

    pub fn set_output_limits(&mut self, output_min: f64, output_max: f64) -> bool {
        if output_min >= output_max {
            return false;
        }

        self.output_min = output_min;
        self.output_max = output_max;

        return true;
    }

    pub fn set_filter_tc(&mut self, filter_tc: f64) -> bool {
        if filter_tc <= 0.0 {
            return false;
        }

        let delta_t = self.sample_time.as_secs_f64();
        self.alpha = delta_t / (delta_t + filter_tc);

        self.filter_tc = filter_tc;
        return true;
    }
}

impl Default for PidConfig {
    fn default() -> Self {
        PidConfig {
            kp: 1.0,
            ki: 0.01,
            kd: 0.0,
            sample_time: Duration::from_millis(10),
            output_min: -f64::INFINITY,
            output_max: f64::INFINITY,
            use_strict_causal_integrator: true,
            use_derivative_on_measurement: true,
            filter_tc: 0.01,
            alpha: 0.5,
        }
    }
}

pub struct Pid {
    config: PidConfig,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Pid { config }
    }

    pub fn config(&self) -> &PidConfig {
        &self.config
    }

    pub fn config_mut(&mut self) -> &mut PidConfig {
        &mut self.config
    }

    pub fn compute(
        &self,
        ctx: &PidContext,
        input: f64,
        setpoint: f64,
        timestamp: Instant,
        feedforward: Option<f64>,
    ) -> (PidContext, f64) {
        let mut state = ctx.clone();
        if state.activity_level == PidActivity::Inactive {
            return (state, state.last_output);
        }

        let time_delta = timestamp.duration_since(state.last_time);

        // Do not compute if the time delta is less than the sample time
        if time_delta < self.config.sample_time {
            return (state, state.last_output);
        }

        let error = setpoint - input;

        // If the PID controller is just switched active, initialize the state
        if state.need_initialize {
            state.last_time = timestamp;
            state.last_input = input;
            state.last_err = error;
            state.i_term = state.last_output;
            state.i_term = state
                .i_term
                .clamp(self.config.output_min, self.config.output_max);
            state.need_initialize = false;
        }

        if !self.config.use_strict_causal_integrator {
            state = self.update_integral(state, error);
        }

        // Optional derivative on measurement to mitigate derivative kick
        let raw_derivative = if self.config.use_derivative_on_measurement {
            state.last_input - input // Note reversed order of operands
        } else {
            error - state.last_err
        };

        // Pass the derivative through a first-order LPF
        let derivative =
            self.config.alpha * raw_derivative + (1.0 - self.config.alpha) * state.last_derivative;
        state.last_derivative = derivative;

        // Clamp output to prevent windup
        let output = self.config.kp * error
            + state.i_term
            + self.config.kd * derivative
            + feedforward.unwrap_or(0.0);
        let clamped_output = output.clamp(self.config.output_min, self.config.output_max);

        if self.config.use_strict_causal_integrator {
            state = self.update_integral(state, error);
        }

        state.last_input = input;
        state.last_err = error;
        state.last_time = timestamp;
        state.last_output = clamped_output;
        return (state, clamped_output);
    }

    fn update_integral(&self, mut ctx: PidContext, error: f64) -> PidContext {
        if ctx.activity_level == PidActivity::HoldIntegration {
            return ctx;
        }
        // Fold gain into i-term calculation for bumpless parameter change
        ctx.i_term += self.config.ki * error;

        // Clamp i-term to prevent windup
        ctx.i_term = ctx
            .i_term
            .clamp(self.config.output_min, self.config.output_max);
        ctx
    }
}
