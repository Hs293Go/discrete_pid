// The PID Controller
// Copyright © 2025 Hs293Go
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

use crate::time::InstantLike;

use core::time::Duration;
use num_traits::float::FloatCore;

#[derive(Copy, Clone, Debug)]
pub struct PidConfig<F: FloatCore> {
    /// Proportional gain coefficient.
    /// Defaults to 1.0.
    kp: F,

    /// Integral gain coefficient.
    /// Defaults to F::zero(). When combined with the default sample time of 10ms, this gives I * Ts = 1
    ki: F,

    /// Derivative gain coefficient.
    /// Defaults to F::zero().
    kd: F,

    /// Time constant for the low-pass filter applied to the derivative term.
    /// Defaults to F::zero().
    filter_tc: F,

    /// Sampling time for the PID controller.
    /// Defaults to 10ms.
    sample_time: Duration,

    /// Minimum output value of the PID controller.
    /// Defaults to negative infinity, i.e. no limit.
    output_min: F,

    /// Maximum output value of the PID controller.
    /// Defaults to positive infinity, i.e. no limit.
    output_max: F,

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
    smoothing_constant: F,
}

impl<F: FloatCore> Default for PidConfig<F> {
    fn default() -> Self {
        PidConfig {
            kp: F::one(),
            ki: F::from(0.01).unwrap(),
            kd: F::zero(),
            filter_tc: F::from(0.01).unwrap(),
            sample_time: Duration::from_millis(10),
            output_min: -F::infinity(),
            output_max: F::infinity(),
            use_strict_causal_integrator: true,
            use_derivative_on_measurement: false,
            smoothing_constant: F::from(0.5).unwrap(),
        }
    }
}

impl<F: FloatCore> PidConfig<F> {
    /// Returns the proportional gain.
    pub fn kp(&self) -> F {
        self.kp
    }

    /// Returns the total integral gain.
    /// This gain is the internal ki value, inversely scaled by the sample time to produce the value that the
    /// user passes into set_ki
    pub fn ki(&self) -> F {
        self.ki / F::from(self.sample_time.as_secs_f64()).unwrap()
    }

    /// Returns the total derivative gain.
    /// This gain is the internal kd value, scaled by the sample time to produce the value that the
    /// user passes into set_kd
    pub fn kd(&self) -> F {
        self.kd * F::from(self.sample_time.as_secs_f64()).unwrap()
    }

    /// Returns the time constant for the low-pass filter applied to the derivative term.
    pub fn filter_tc(&self) -> F {
        self.filter_tc
    }

    /// Returns the sampling time for the PID controller.
    pub fn sample_time(&self) -> Duration {
        self.sample_time
    }

    /// Returns the minimum output limit.
    pub fn output_min(&self) -> F {
        self.output_min
    }

    /// Returns the maximum output limit.
    pub fn output_max(&self) -> F {
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
    pub fn set_kp(&mut self, kp: F) -> bool {
        if kp <= F::zero() || !kp.is_finite() {
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
    pub fn set_ki(&mut self, ki: F) -> bool {
        if ki < F::zero() || !ki.is_finite() {
            return false;
        }
        self.ki = ki * F::from(self.sample_time.as_secs_f64()).unwrap();
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
    pub fn set_kd(&mut self, kd: F) -> bool {
        if kd < F::zero() || !kd.is_finite() {
            return false;
        }
        self.kd = kd / F::from(self.sample_time.as_secs_f64()).unwrap();
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
    pub fn set_filter_tc(&mut self, filter_tc: F) -> bool {
        if filter_tc <= F::zero() || !filter_tc.is_finite() {
            return false;
        }

        let delta_t = F::from(self.sample_time.as_secs_f64()).unwrap();
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

        let ratio = F::from(sample_time.as_secs_f64() / self.sample_time.as_secs_f64()).unwrap();

        self.ki = self.ki * ratio;
        self.kd = self.kd / ratio;

        let delta_t = F::from(sample_time.as_secs_f64()).unwrap();
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
    pub fn set_output_limits(&mut self, output_min: F, output_max: F) -> bool {
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
pub struct PidContext<T: InstantLike, F: FloatCore> {
    /// The integral term of the PID controller. This accumulates the error multiplied by the I
    /// gain to avoid integral jump.
    /// Users should not access this directly
    i_term: F,

    /// The last error value. This is available for the user to query, and is also used to compute
    /// the derivative term when derivative on measurement is off
    last_err: F,

    /// The last input value. This is used to compute the derivative term when derivative on
    /// measurement is on
    last_input: F,

    /// The last output value. This is available for the user to query, and is also used to hold
    /// the last output unchanged when the PID controller is inactive
    last_output: F,

    /// The last derivative value. This is used by the low-pass filter to smooth the derivative
    /// Users should not access this directly
    last_derivative: F,

    /// The last time the PID controller was computed. This is compared to the current time to
    /// determine if `sample_time` has elapsed since the last computation and the controller should
    /// compute a new output. When the PID has never been run before, this is set to None.
    last_time: Option<T>,

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

impl<T: InstantLike, F: FloatCore> PidContext<T, F> {
    /// Creates a new uninitialized PID context. This context is automatically initialized when the
    /// PID controller computes its first output.
    pub fn new_uninitialized() -> Self {
        Default::default()
    }

    /// Creates a new PID context with the given timestamp, input, and output values. This
    /// initialization ensures smooth continuity of the PID controller's output under the following
    /// assumptions:
    ///
    /// * The input (process value) is at a steady state (or minimally changing)
    /// * There is no error at the time of initialization, such that the output is purely
    ///   contributed by the integral term
    ///
    /// # Arguments
    /// - `timestamp`: The timestamp of the initialization.
    /// - `input`: The input (process value) at the time of initialization.
    /// - `output`: The output of the PID controller at the time of initialization.
    pub fn new_initialize(timestamp: T, input: F, output: F) -> Self {
        Self {
            i_term: output,
            last_err: F::zero(),
            last_input: input,
            last_output: output,
            last_derivative: F::zero(),
            last_time: Some(timestamp),
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: true,
        }
    }

    /// Returns the output (last computed value) of the PID controller.
    pub fn output(&self) -> F {
        self.last_output
    }

    /// Returns the error (difference between setpoint and input) of the PID controller.
    pub fn error(&self) -> F {
        self.last_err
    }

    /// Returns the last time the PID controller was computed.
    pub fn last_time(&self) -> Option<T> {
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
        self.i_term = F::zero();
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

impl<T: InstantLike, F: FloatCore> Default for PidContext<T, F> {
    fn default() -> Self {
        Self {
            i_term: F::zero(),
            last_err: F::zero(),
            last_input: F::zero(),
            last_output: F::zero(),
            last_derivative: F::zero(),
            last_time: None,
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: false,
        }
    }
}

/// A functional implementation of a PID (Proportional-Integral-Derivative) controller.
///
/// This struct represents a PID controller, which computes the control output based proportional,
/// integral, and derivative terms based on the error between a setpoint and a process variable.
/// This implementation is stateless so a context object must be passed in and returned with each
/// call to `compute`.
pub struct FuncPidController<F: FloatCore> {
    config: PidConfig<F>,
}

/// A stateful implementation of a PID (Proportional-Integral-Derivative) controller.
///
/// This struct represents a PID controller, which computes the control output based proportional,
/// integral, and derivative terms based on the error between a setpoint and a process variable.
/// This implementation maintains its own state, so it can be used without passing a context object.
pub struct PidController<T: InstantLike, F: FloatCore> {
    ctx: PidContext<T, F>,
    controller: FuncPidController<F>,
}

impl<F: FloatCore> FuncPidController<F> {
    pub fn new(config: PidConfig<F>) -> Self {
        FuncPidController { config }
    }

    pub fn config(&self) -> &PidConfig<F> {
        &self.config
    }

    pub fn config_mut(&mut self) -> &mut PidConfig<F> {
        &mut self.config
    }

    pub fn compute<T: InstantLike>(
        &self,
        mut ctx: PidContext<T, F>,
        input: F,
        setpoint: F,
        timestamp: T,
        feedforward: Option<F>,
    ) -> (F, PidContext<T, F>) {
        if !ctx.is_active {
            return (ctx.last_output, ctx);
        }

        let error = setpoint - input;

        // If the PID controller is just switched active or has never been run before (last_time is
        // None), initialize the state then run the controller without checking if the sample time
        // has elapsed
        if !ctx.is_initialized || ctx.last_time.is_none() {
            ctx.last_time = Some(timestamp);
            ctx.last_input = input;
            ctx.last_err = error;
            ctx.i_term = ctx.last_output;
            ctx.i_term = ctx
                .i_term
                .clamp(self.config.output_min, self.config.output_max);
            ctx.is_initialized = true;
        } else {
            // If there is no need to initialize and the controller has been called before, check if the
            // time delta is less than the sample time
            let time_delta = timestamp.duration_since(ctx.last_time.unwrap());
            if time_delta < self.config.sample_time {
                return (ctx.last_output, ctx);
            }
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
            + (F::one() - self.config.smoothing_constant) * ctx.last_derivative;
        ctx.last_derivative = derivative;

        // Clamp output to prevent windup
        let output = self.config.kp * error
            + ctx.i_term
            + self.config.kd * derivative
            + feedforward.unwrap_or(F::zero());
        let clamped_output = output.clamp(self.config.output_min, self.config.output_max);

        if self.config.use_strict_causal_integrator {
            ctx = self.update_integral(ctx, error);
        }

        ctx.last_input = input;
        ctx.last_err = error;
        ctx.last_time = Some(timestamp);
        ctx.last_output = clamped_output;
        (clamped_output, ctx)
    }

    fn update_integral<T: InstantLike>(
        &self,
        mut ctx: PidContext<T, F>,
        error: F,
    ) -> PidContext<T, F> {
        match ctx.integrator_activity {
            IntegratorActivity::Inactive => {
                ctx.i_term = F::zero();
            }
            IntegratorActivity::HoldIntegration => {
                return ctx;
            }
            IntegratorActivity::Active => {
                ctx.i_term = (ctx.i_term + self.config.ki * error)
                    .clamp(self.config.output_min, self.config.output_max);
            }
        }
        ctx
    }
}

impl<T: InstantLike, F: FloatCore> PidController<T, F> {
    pub fn new(config: PidConfig<F>) -> Self {
        let controller = FuncPidController::new(config);
        Self {
            ctx: PidContext::<T, F>::new_uninitialized(),
            controller,
        }
    }

    pub fn config(&self) -> &PidConfig<F> {
        &self.controller.config
    }

    pub fn config_mut(&mut self) -> &mut PidConfig<F> {
        &mut self.controller.config
    }

    pub fn compute(&mut self, input: F, setpoint: F, timestamp: T, feedforward: Option<F>) -> F {
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
