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

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum PidActivity {
    Inactive,
    HoldIntegration,
    Active,
}

#[derive(Copy, Clone, Debug)]
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
        true
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
        true
    }

    pub fn set_output_limits(&mut self, output_min: f64, output_max: f64) -> bool {
        if output_min >= output_max {
            return false;
        }

        self.output_min = output_min;
        self.output_max = output_max;

        true
    }

    pub fn set_filter_tc(&mut self, filter_tc: f64) -> bool {
        if filter_tc <= 0.0 {
            return false;
        }

        let delta_t = self.sample_time.as_secs_f64();
        self.alpha = delta_t / (delta_t + filter_tc);

        self.filter_tc = filter_tc;
        true
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
        self.activity_level = activity_level
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
        mut ctx: PidContext,
        input: f64,
        setpoint: f64,
        timestamp: Instant,
        feedforward: Option<f64>,
    ) -> (f64, PidContext) {
        if ctx.activity_level == PidActivity::Inactive {
            return (ctx.last_output, ctx);
        }

        let time_delta = timestamp.duration_since(ctx.last_time);

        // Do not compute if the time delta is less than the sample time
        if time_delta < self.config.sample_time {
            return (ctx.last_output, ctx);
        }

        let error = setpoint - input;

        // If the PID controller is just switched active, initialize the state
        if ctx.need_initialize {
            ctx.last_time = timestamp;
            ctx.last_input = input;
            ctx.last_err = error;
            ctx.i_term = ctx.last_output;
            ctx.i_term = ctx
                .i_term
                .clamp(self.config.output_min, self.config.output_max);
            ctx.need_initialize = false;
        }

        if !self.config.use_strict_causal_integrator {
            ctx = self.update_integral(ctx, error);
        }

        // Optional derivative on measurement to mitigate derivative kick
        let raw_derivative = if self.config.use_derivative_on_measurement {
            ctx.last_input - input // Note reversed order of operands
        } else {
            error - ctx.last_err
        };

        // Pass the derivative through a first-order LPF
        let derivative =
            self.config.alpha * raw_derivative + (1.0 - self.config.alpha) * ctx.last_derivative;
        ctx.last_derivative = derivative;

        // Clamp output to prevent windup
        let output = self.config.kp * error
            + ctx.i_term
            + self.config.kd * derivative
            + feedforward.unwrap_or(0.0);
        let clamped_output = output.clamp(self.config.output_min, self.config.output_max);

        if self.config.use_strict_causal_integrator {
            ctx = self.update_integral(ctx, error);
        }

        ctx.last_input = input;
        ctx.last_err = error;
        ctx.last_time = timestamp;
        ctx.last_output = clamped_output;
        (clamped_output, ctx)
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
