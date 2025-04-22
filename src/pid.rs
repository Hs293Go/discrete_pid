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

    /// Time constant for the low-pass filter applied to the derivative term.
    /// Defaults to 0.01s.
    filter_tc: f64,

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

    /// Smoothing constant for the low-pass filter applied to the derivative term.
    /// Not to be set directly, but calculated based on the filter time constant and sample time.
    smoothing_constant: f64,
}

impl Default for PidConfig {
    fn default() -> Self {
        PidConfig {
            kp: 1.0,
            ki: 0.01,
            kd: 0.0,
            filter_tc: 0.01,
            sample_time: Duration::from_millis(10),
            output_min: -f64::INFINITY,
            output_max: f64::INFINITY,
            use_strict_causal_integrator: true,
            use_derivative_on_measurement: false,
            smoothing_constant: 0.5,
        }
    }
}

impl PidConfig {
    /// Returns the proportional gain.
    pub fn kp(&self) -> f64 {
        self.kp
    }

    /// Returns the total integral gain.
    /// This gain is the internal ki value, inversely scaled by the sample time to produce the value that the
    /// user passes into set_ki
    pub fn ki(&self) -> f64 {
        self.ki / self.sample_time.as_secs_f64()
    }

    /// Returns the total derivative gain.
    /// This gain is the internal kd value, scaled by the sample time to produce the value that the
    /// user passes into set_kd
    pub fn kd(&self) -> f64 {
        self.kd * self.sample_time.as_secs_f64()
    }

    /// Returns the time constant for the low-pass filter applied to the derivative term.
    pub fn filter_tc(&self) -> f64 {
        self.filter_tc
    }

    /// Returns the sampling time for the PID controller.
    pub fn sample_time(&self) -> Duration {
        self.sample_time
    }

    /// Returns the minimum output limit.
    pub fn output_min(&self) -> f64 {
        self.output_min
    }

    /// Returns the maximum output limit.
    pub fn output_max(&self) -> f64 {
        self.output_max
    }

    /// Returns the flag indicating whether to use a strict causal integrator.
    pub fn use_strict_causal_integrator(&self) -> bool {
        self.use_strict_causal_integrator
    }

    /// Returns the flag indicating whether to apply the derivative on the measurement.
    pub fn use_derivative_on_measurement(&self) -> bool {
        self.use_derivative_on_measurement
    }

    /// Sets the proportional gain.
    ///
    /// The proportional gain must be greater than zero. If the user intends to disable the PID
    /// controller, they should set the activity level to `Inactive` instead.
    ///
    /// # Arguments
    /// - `kp`: The new proportional gain.
    ///
    /// # Returns
    /// - `true` if the gain was set successfully.
    /// - `false` if the gain is less than or equal to zero or not finite.
    pub fn set_kp(&mut self, kp: f64) -> bool {
        if kp <= 0.0 || !kp.is_finite() {
            return false;
        }
        self.kp = kp;
        true
    }

    /// Sets the integral gain.
    ///
    /// The user passes in a 'total' integral gain, which is scaled by the sample time to produce
    /// the actual ki value used in the PID algorithm.
    ///
    /// # Arguments
    /// - `ki`: The new integral gain.
    ///
    /// # Returns
    /// - `true` if the gain was set successfully.
    /// - `false` if the gain is less than zero or not finite.
    pub fn set_ki(&mut self, ki: f64) -> bool {
        if ki < 0.0 || !ki.is_finite() {
            return false;
        }
        self.ki = ki * self.sample_time.as_secs_f64();
        true
    }

    /// Sets the derivative gain.
    ///
    /// The user passes in a 'total' derivative gain, which is scaled inversely by the sample time to produce
    /// the actual kd value used in the PID algorithm.
    ///
    /// # Arguments
    /// - `kd`: The new derivative gain.
    ///
    /// # Returns
    /// - `true` if the gain was set successfully.
    /// - `false` if the gain is less than zero or not finite.
    pub fn set_kd(&mut self, kd: f64) -> bool {
        if kd < 0.0 || !kd.is_finite() {
            return false;
        }
        self.kd = kd / self.sample_time.as_secs_f64();
        true
    }

    /// Sets the time constant for the low-pass filter applied to the derivative term.
    ///
    /// # Arguments
    /// - `filter_tc`: The new time constant for the filter.
    ///
    /// # Returns
    /// - `true` if the time constant was set successfully.
    /// - `false` if the time constant is less than or equal to zero or non finite.
    pub fn set_filter_tc(&mut self, filter_tc: f64) -> bool {
        if filter_tc <= 0.0 || !filter_tc.is_finite() {
            return false;
        }

        let delta_t = self.sample_time.as_secs_f64();
        self.smoothing_constant = delta_t / (delta_t + filter_tc);

        self.filter_tc = filter_tc;
        true
    }

    /// Sets the sample time for the PID controller. Rescales the integral and derivative gains and
    /// the filter for the derivative term to maintain consistent behavior.
    ///
    /// # Arguments
    /// - `sample_time`: The new sample time.
    ///
    /// # Returns
    /// - `true` if the sample time was set successfully.
    /// - `false` if the sample time is zero or max
    pub fn set_sample_time(&mut self, sample_time: Duration) -> bool {
        if sample_time.is_zero() || sample_time == Duration::MAX {
            return false;
        }

        let ratio = sample_time.as_secs_f64() / self.sample_time.as_secs_f64();

        self.ki *= ratio;
        self.kd /= ratio;

        let delta_t = sample_time.as_secs_f64();
        self.smoothing_constant = delta_t / (delta_t + self.filter_tc);

        self.sample_time = sample_time;
        true
    }

    /// Sets the minimum and maximum output limits for the PID controller.
    ///
    /// These limits may be set to infinity to disable clamping.
    ///
    /// # Arguments
    /// - `output_min`: The minimum output limit.
    /// - `output_max`: The maximum output limit.
    ///
    /// # Returns
    /// - `true` if the limits were set successfully.
    /// - `false` if the minimum limit is greater than or equal to the maximum limit, or either
    ///   limit is not finite.
    pub fn set_output_limits(&mut self, output_min: f64, output_max: f64) -> bool {
        if output_min >= output_max || output_max.is_nan() || output_min.is_nan() {
            return false;
        }

        self.output_min = output_min;
        self.output_max = output_max;

        true
    }

    /// Sets whether to use a strict causal integrator.
    pub fn set_use_strict_causal_integrator(&mut self, use_strict_causal_integrator: bool) {
        self.use_strict_causal_integrator = use_strict_causal_integrator;
    }

    /// Sets whether to apply the derivative on the measurement.
    pub fn set_use_derivative_on_measurement(&mut self, use_derivative_on_measurement: bool) {
        self.use_derivative_on_measurement = use_derivative_on_measurement;
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IntegratorActivity {
    Inactive,
    HoldIntegration,
    Active,
}

#[derive(Copy, Clone, Debug)]
pub struct PidContext {
    /// The integral term of the PID controller. This accumulates the error multiplied by the I
    /// gain to avoid integral jump.
    /// Users should not access this directly
    i_term: f64,

    /// The last error value. This is available for the user to query, and is also used to compute
    /// the derivative term when derivative on measurement is off
    last_err: f64,

    /// The last input value. This is used to compute the derivative term when derivative on
    /// measurement is on
    last_input: f64,

    /// The last output value. This is available for the user to query, and is also used to hold
    /// the last output unchanged when the PID controller is inactive
    last_output: f64,

    /// The last derivative value. This is used by the low-pass filter to smooth the derivative
    /// Users should not access this directly
    last_derivative: f64,

    /// The last time the PID controller was computed. This is compared to the current time to
    /// determine if `sample_time` has elapsed since the last computation and the controller should
    /// compute a new output
    last_time: Instant,

    /// The activity level of the PID controller. This determines if the integrator is active,
    /// holding the last value unchanged, or inactive and zeroed.
    integrator_activity: IntegratorActivity,

    /// A flag indicating whether the PID controller is active. If false, the output will not be
    /// updated and the last output will be held unchanged.
    is_active: bool,

    /// A flag indicating whether the PID controller has been initialized. This is used to
    /// reinitialize the controller when upon being activated
    is_initialized: bool,
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
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: false,
        }
    }

    /// Returns the output (last computed value) of the PID controller.
    pub fn output(&self) -> f64 {
        self.last_output
    }

    /// Returns the error (difference between setpoint and input) of the PID controller.
    pub fn error(&self) -> f64 {
        self.last_err
    }

    /// Returns the last time the PID controller was computed.
    pub fn last_time(&self) -> Instant {
        self.last_time
    }

    /// Queries whether the PID controller is active.
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    /// Queries the integrator activity level of the PID controller.
    pub fn integrator_activity(&self) -> IntegratorActivity {
        self.integrator_activity
    }

    /// Queries whether the PID controller has been initialized.
    pub fn is_initialized(&self) -> bool {
        self.is_initialized
    }

    /// Activates the PID controller. If the controller was previously inactive, it will be
    /// reinitialized.
    pub fn activate(&mut self) {
        if !self.is_active {
            self.is_initialized = false;
        }
        self.is_active = true;
    }

    /// Deactivates the PID controller. The output will not be updated and the last output will be
    /// held unchanged.
    pub fn deactivate(&mut self) {
        self.is_active = false;
    }

    /// Resets the integral term of the PID controller to zero.
    pub fn reset_integral(&mut self) {
        self.i_term = 0.0;
    }

    /// Sets the integrator activity level of the PID controller.
    /// If the activity level is set to `Inactive` from any other state, the integral term will be
    /// reset to zero.
    pub fn set_integrator_activity(&mut self, activity: IntegratorActivity) {
        self.integrator_activity = activity;

        let is_active = self.integrator_activity != IntegratorActivity::Inactive;
        if is_active && activity == IntegratorActivity::Inactive {
            self.reset_integral();
        }
    }
}

/// A functional implementation of a PID (Proportional-Integral-Derivative) controller.
///
/// This struct represents a PID controller, which computes the control output based proportional,
/// integral, and derivative terms based on the error between a setpoint and a process variable.
/// This implementation is stateless so a context object must be passed in and returned with each
/// call to `compute`.
pub struct FuncPidController {
    config: PidConfig,
}

/// A stateful implementation of a PID (Proportional-Integral-Derivative) controller.
///
/// This struct represents a PID controller, which computes the control output based proportional,
/// integral, and derivative terms based on the error between a setpoint and a process variable.
/// This implementation maintains its own state, so it can be used without passing a context object.
pub struct PidController {
    ctx: PidContext,
    controller: FuncPidController,
}

impl FuncPidController {
    pub fn new(config: PidConfig) -> Self {
        FuncPidController { config }
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
        if !ctx.is_active {
            return (ctx.last_output, ctx);
        }

        let time_delta = timestamp.duration_since(ctx.last_time);

        // Do not compute if the time delta is less than the sample time
        if time_delta < self.config.sample_time {
            return (ctx.last_output, ctx);
        }

        let error = setpoint - input;

        // If the PID controller is just switched active, initialize the state
        if !ctx.is_initialized {
            ctx.last_time = timestamp;
            ctx.last_input = input;
            ctx.last_err = error;
            ctx.i_term = ctx.last_output;
            ctx.i_term = ctx
                .i_term
                .clamp(self.config.output_min, self.config.output_max);
            ctx.is_initialized = true;
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
        let derivative = self.config.smoothing_constant * raw_derivative
            + (1.0 - self.config.smoothing_constant) * ctx.last_derivative;
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
        match ctx.integrator_activity {
            IntegratorActivity::Inactive => {
                ctx.i_term = 0.0;
            }
            IntegratorActivity::HoldIntegration => {
                return ctx;
            }
            IntegratorActivity::Active => {
                ctx.i_term += self.config.ki * error;
                ctx.i_term = ctx
                    .i_term
                    .clamp(self.config.output_min, self.config.output_max);
            }
        }
        ctx
    }
}

impl PidController {
    pub fn new(config: PidConfig) -> Self {
        let controller = FuncPidController::new(config);
        Self {
            ctx: PidContext::new(Instant::now()),
            controller,
        }
    }

    pub fn config(&self) -> &PidConfig {
        &self.controller.config
    }

    pub fn config_mut(&mut self) -> &mut PidConfig {
        &mut self.controller.config
    }

    pub fn compute(
        &mut self,
        input: f64,
        setpoint: f64,
        timestamp: Instant,
        feedforward: Option<f64>,
    ) -> f64 {
        let (output, ctx) =
            self.controller
                .compute(self.ctx, input, setpoint, timestamp, feedforward);
        self.ctx = ctx;
        output
    }

    pub fn activate(&mut self) {
        self.ctx.activate();
    }

    pub fn deactivate(&mut self) {
        self.ctx.deactivate();
    }
}
