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

#[cfg(feature = "std")]
use thiserror::Error;

/// Error type for PID configuration errors.
///
/// This enum represents various errors that can occur when setting up or modifying the PID
/// configuration. All of these errors are related to invalid values that break the invariants of the
/// PID configuration.
#[cfg_attr(feature = "std", derive(Error))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum PidConfigError {
    /// The proportional gain is non-positive or non-finite.
    #[cfg_attr(
        feature = "std",
        error("Proportional gain is non-positive or non-finite")
    )]
    InvalidProportionalGain = 1,

    /// The integral gain is negative or non-finite.
    #[cfg_attr(feature = "std", error("Integral gain is negative or non-finite"))]
    InvalidIntegralGain = 2,

    /// The derivative gain is negative or non-finite.
    #[cfg_attr(feature = "std", error("Derivative gain is negative or non-finite"))]
    InvalidDerivativeGain = 3,

    /// The filter time constant is non-positive or non-finite.
    #[cfg_attr(
        feature = "std",
        error("Filter time constant is non-positive or non-finite")
    )]
    InvalidSampleTime = 4,

    /// The output limits are flipped or not finite.
    #[cfg_attr(feature = "std", error("Output limits are flipped or NAN"))]
    InvalidOutputLimits = 5,

    /// The filter time constant is non-positive or non-finite.
    #[cfg_attr(
        feature = "std",
        error("Filter time constant is non-positive or non-finite")
    )]
    InvalidFilterTimeConstant = 6,
}

/// A container for the configuration of PID controllers.
///
/// The configuration parameters for a PID controller are:
/// - Gain: proportional (`kp`), integral (`ki`), and derivative (`kd`)
/// - Sample time (`sample_time`)
/// - Output limits (`output_min`, `output_max`)
/// - Flags that toggles strict causal integrator and derivative on measurement
/// - Time constant for the low-pass filter on the derivative term
///
/// Modify the configuration using the provided setter methods in-place, or use
/// [`PidConfigBuilder`] to create a fully-specified configuration.
///
/// Tune the PID controller on-the-fly by accessing its configuration through
/// [`FuncPidController::config_mut`] or [`PidController::config_mut`], then calling the desired
/// setters.
///
/// # Details
///
/// [`use_strict_causal_integrator`](PidConfig::use_strict_causal_integrator) is a flag that determines whether the integral term is updated
/// before or after the output is computed. If set to true, the integral term is updated after the
/// output is conmputed, such that the output strictly depends on information from the past.
///
/// [`use_derivative_on_measurement`](PidConfig::use_derivative_on_measurement) is a flag that determines whether the derivative term is
/// computed by finite-differencing the **input/measurement** or **error**. If the setpoint is
/// varying slowly but discontinuously (e.g. discrete waypoints/human control), derivative on error
/// will introduce _derivative kick_ and cause the output to spike. In this case, it is better to
/// use derivative on measurement. If the setpoint is varying rapid and/or continuously (e.g.
/// computed by a path planner), derivative on measurement discards setpoint variation information
/// and is **wrong**.
///
/// # Invariants
///
/// All parameters must not be NaN. The following invariants must be satisfied:
///
/// - `kp > 0`,  and `filter_tc > 0`; all finite
/// - Neither `sample_time.is_zero()` nor `sample_time == Duration::MAX`
/// - `ki >= 0` and `kd >= 0`; all finite
/// - `output_min < output_max`
///
/// All setters return a [`Result`] containing a member in [`PidConfigError`] when encountering an
/// invariant-breaking value.
///
/// # Example
/// ```rust
/// use discrete_pid::pid::{PidConfig, PidConfigError};
/// use core::time::Duration;
///
/// let mut config = PidConfig::default();
/// assert!(config.set_kp(2.0).is_ok());
/// assert!(config.set_ki(-1.0) == Err(PidConfigError::InvalidIntegralGain));
/// config.set_kd(0.1);
/// config.set_sample_time(Duration::from_millis(20));
/// config.set_output_limits(-10.0, 10.0);
/// ```
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

fn check_kp<F: FloatCore>(kp: F) -> Result<(), PidConfigError> {
    if kp <= F::zero() || !kp.is_finite() {
        return Err(PidConfigError::InvalidProportionalGain);
    }
    Ok(())
}

fn check_ki<F: FloatCore>(ki: F) -> Result<(), PidConfigError> {
    if ki < F::zero() || !ki.is_finite() {
        return Err(PidConfigError::InvalidIntegralGain);
    }
    Ok(())
}

fn check_kd<F: FloatCore>(kd: F) -> Result<(), PidConfigError> {
    if kd < F::zero() || !kd.is_finite() {
        return Err(PidConfigError::InvalidDerivativeGain);
    }
    Ok(())
}

fn check_filter_tc<F: FloatCore>(filter_tc: F) -> Result<(), PidConfigError> {
    if filter_tc <= F::zero() || !filter_tc.is_finite() {
        return Err(PidConfigError::InvalidFilterTimeConstant);
    }
    Ok(())
}

fn check_sample_time(sample_time: Duration) -> Result<(), PidConfigError> {
    if sample_time.is_zero() || sample_time == Duration::MAX {
        return Err(PidConfigError::InvalidSampleTime);
    }
    Ok(())
}

fn check_output_limits<F: FloatCore>(output_min: F, output_max: F) -> Result<(), PidConfigError> {
    if output_min >= output_max || output_max.is_nan() || output_min.is_nan() {
        return Err(PidConfigError::InvalidOutputLimits);
    }
    Ok(())
}

impl<F: FloatCore> Default for PidConfig<F> {
    /// Creates a new [`PidConfig`] instance with the following default values.
    ///
    /// | Parameter                     | Value     | Note                                                               |
    /// |-------------------------------|-----------|--------------------------------------------------------------------|
    /// | Proportional gain             | 1.0       |                                                                    |
    /// | Integral gain                 | 0.01      | time-invariant gain: 1.0 when combined with the default sample time of 10ms |
    /// | Derivative gain               | 0.0       | Disabled                                                           |
    /// | Filter time constant          | 0.01      | in seconds                                                         |
    /// | Sample time                   | 0.01      | in seconds                                                         |
    /// | Output minimum                | -infinity | no limit                                                           |
    /// | Output maximum                | +infinity | no limit                                                           |
    /// | Use strict causal integrator  | true      |                                                                    |
    /// | Use derivative on measurement | false     |                                                                    |
    fn default() -> Self {
        PidConfig {
            kp: F::one(),
            ki: F::from(0.01).unwrap(),
            kd: F::zero(),
            filter_tc: F::from(0.01).unwrap(),
            sample_time: Duration::from_millis(10),
            output_min: -F::infinity(),
            output_max: F::infinity(),
            use_strict_causal_integrator: false,
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

    /// Returns the time-invariant integral gain.
    /// This is computed by inversely scaling the internal `ki` by the sample time; It would give
    /// the value that you last passed into [`set_ki`](Self::set_ki())
    pub fn ki(&self) -> F {
        self.ki / F::from(self.sample_time.as_secs_f64()).unwrap()
    }

    /// Returns the time-invariant derivative gain.
    /// This is computed by inversely scaling the internal `kd` by the sample time; It would give
    /// the value that you last passed into [`set_kd`](Self::set_kd)
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

    /// Returns the flag indicating whether to use a strict causal integrator. See
    /// [Details](crate::pid::PidConfig#details) for more information on the semantics of this
    /// flag.
    pub fn use_strict_causal_integrator(&self) -> bool {
        self.use_strict_causal_integrator
    }

    /// Returns the flag indicating whether to apply the derivative on the measurement. See
    /// [Details](crate::pid::PidConfig#details) for more information on the semantics of this
    /// flag.
    pub fn use_derivative_on_measurement(&self) -> bool {
        self.use_derivative_on_measurement
    }

    /// Sets the proportional gain.
    ///
    /// `kp` must be greater than zero. If you intend to disable the PID controller, call
    /// [`PidContext::deactivate`] instead.
    ///
    /// # Arguments
    /// - `kp`: Proportional gain.
    ///
    /// # Returns
    /// - `Ok(())` if the gain was set successfully.
    /// - `Err(PidConfigError::InvalidProportionalGain)` if `kp <= 0` or not finite.
    pub fn set_kp(&mut self, kp: F) -> Result<(), PidConfigError> {
        check_kp(kp)?;
        self.kp = kp;
        Ok(())
    }

    /// Sets the integral gain.
    ///
    /// `ki` must not be negative; You can set `ki` to 0 to disable the derivative term.
    ///
    /// You pass in a time-invariant `ki`, which is scaled by the sample time to produce the
    /// internal ki value.
    ///
    /// # Arguments
    /// - `ki`: Integral gain.
    ///
    /// # Returns
    /// - `Ok(())` if the gain was set successfully.
    /// - `Err(PidConfigError::InvalidIntegralGain)` if `ki < 0` or not finite.
    pub fn set_ki(&mut self, ki: F) -> Result<(), PidConfigError> {
        check_ki(ki)?;
        self.ki = ki * F::from(self.sample_time.as_secs_f64()).unwrap();
        Ok(())
    }

    /// Sets the derivative gain.
    ///
    /// `kd` must not be negative; You can set `kd` to 0 to disable the derivative term.
    ///
    /// You pass in a time-invariant `kd`, which is scaled inversely by the sample time to produce
    /// the internal kd value
    ///
    /// # Arguments
    /// - `kd`: Derivative gain.
    ///
    /// # Returns
    /// - `Ok(())` if the gain was set successfully.
    /// - `Err(PidConfigError::InvalidDerivativeGain)` if `kd < 0` or not finite.
    pub fn set_kd(&mut self, kd: F) -> Result<(), PidConfigError> {
        check_kd(kd)?;
        self.kd = kd / F::from(self.sample_time.as_secs_f64()).unwrap();
        Ok(())
    }

    /// Sets the time constant for the low-pass filter applied to the derivative term.
    ///
    /// `filter_tc` must be greater than zero and finite.
    ///
    /// # Arguments
    /// - `filter_tc`: Time constant for the filter.
    ///
    /// # Returns
    /// - `Ok(())` if the time constant was set successfully.
    /// - `Err(PidConfigError::InvalidFilterTimeConstant)` if `filter_tc <= 0` or not finite.
    pub fn set_filter_tc(&mut self, filter_tc: F) -> Result<(), PidConfigError> {
        check_filter_tc(filter_tc)?;
        let delta_t = F::from(self.sample_time.as_secs_f64()).unwrap();
        self.smoothing_constant = delta_t / (delta_t + filter_tc);
        self.filter_tc = filter_tc;
        Ok(())
    }

    /// Sets the sample time for the PID controller. Rescales the integral and derivative gains and
    /// the filter for the derivative term to maintain consistent behavior.
    ///
    /// `sample_time` must not be the zero duration or the max duration.
    ///
    /// # Arguments
    /// - `sample_time`: Sample time.
    ///
    /// # Returns
    /// - `Ok(())` if the sample time was set successfully.
    /// - `Err(PidConfigError::InvalidSampleTime)` if `sample_time.is_zero()` or `sample_time == Duration::MAX`.
    pub fn set_sample_time(&mut self, sample_time: Duration) -> Result<(), PidConfigError> {
        check_sample_time(sample_time)?;

        let ratio = F::from(sample_time.as_secs_f64() / self.sample_time.as_secs_f64()).unwrap();

        self.ki = self.ki * ratio;
        self.kd = self.kd / ratio;

        let delta_t = F::from(sample_time.as_secs_f64()).unwrap();
        self.smoothing_constant = delta_t / (delta_t + self.filter_tc);

        self.sample_time = sample_time;
        Ok(())
    }

    /// Sets the minimum and maximum output limits for the PID controller.
    ///
    /// You may set these limits to infinity to disable clamping.
    ///
    /// # Arguments
    /// - `output_min`: The minimum output limit.
    /// - `output_max`: The maximum output limit.
    ///
    /// # Returns
    /// - `Ok(())` if the limits were set successfully.
    /// - `Err(PidConfigError::InvalidOutputLimits)` if `output_min >= output_max` or if either
    ///   limit is NaN.
    pub fn set_output_limits(
        &mut self,
        output_min: F,
        output_max: F,
    ) -> Result<(), PidConfigError> {
        check_output_limits(output_min, output_max)?;

        self.output_min = output_min;
        self.output_max = output_max;

        Ok(())
    }

    /// Sets whether to use a strict causal integrator.
    /// See [Details](crate::pid::PidConfig#details) for more information on the semantics of this
    /// flag.
    ///
    /// # Arguments
    /// - `use_strict_causal_integrator`: Whether to use a strict causal integrator.
    pub fn set_use_strict_causal_integrator(&mut self, use_strict_causal_integrator: bool) {
        self.use_strict_causal_integrator = use_strict_causal_integrator;
    }

    /// Sets whether to apply the derivative on the measurement.
    /// See [Details](crate::pid::PidConfig#details) for more information on the semantics of this
    /// flag.
    ///
    /// # Arguments
    /// - `use_derivative_on_measurement`: Whether to apply the derivative on the measurement.
    pub fn set_use_derivative_on_measurement(&mut self, use_derivative_on_measurement: bool) {
        self.use_derivative_on_measurement = use_derivative_on_measurement;
    }
}

/// A builder for fine-grained construction of `PidConfig` instances.
///
/// This struct provides a convenient way to create a `PidConfig` instance while configuring some
/// desired parameters, leaving the rest at their default values. Note that the configured
/// parameter values are validated when the `build` method is called, and an error is returned if
/// any of the values are invalid.
///
/// This builder allows the PID configuration to be fully specified at construction time and left
/// immutable afterwards. Combined with the `FuncPidController` implementation, this allows for a
/// immutable PID controller that can be used in a functional style, giving perfectly reproducible
/// control outputs.
///
/// # Example
///
/// ```rust
/// use discrete_pid::pid::{PidConfigBuilder, PidConfig};
/// use core::time::Duration;
///
/// let pid_config = PidConfigBuilder::default()
///     .kp(2.0)
///     .ki(0.5)
///     .kd(0.1)
///     .filter_tc(0.01)
///     .sample_time(Duration::from_millis(20))
///     .output_limits(-10.0, 10.0)
///     .use_strict_causal_integrator(true)
///     .use_derivative_on_measurement(false)
///     .build()
///     .expect("Failed to build PID config; Exiting application");
/// ```
#[derive(Debug)]
pub struct PidConfigBuilder<F: FloatCore> {
    kp: F,
    total_ki: F,
    total_kd: F,
    filter_tc: F,
    sample_time: Duration,
    output_min: F,
    output_max: F,
    use_strict_causal_integrator: bool,
    use_derivative_on_measurement: bool,
}

impl<F: FloatCore> Default for PidConfigBuilder<F> {
    /// Creates a new `PidConfigBuilder` instance with default values identical to those from
    /// [`PidConfig::default()`].
    fn default() -> Self {
        Self {
            kp: F::one(),
            total_ki: F::from(0.01).unwrap(),
            total_kd: F::zero(),
            filter_tc: F::from(0.01).unwrap(),
            sample_time: Duration::from_millis(10),
            output_min: -F::infinity(),
            output_max: F::infinity(),
            use_strict_causal_integrator: true,
            use_derivative_on_measurement: false,
        }
    }
}

impl<F: FloatCore> PidConfigBuilder<F> {
    /// Configures the proportional gain of the [`PidConfig`] to be built. No validation is performed
    /// until [`build`](Self::build) is called.
    ///
    /// # Arguments
    /// - `kp`: Proportional gain.
    pub fn kp(mut self, kp: F) -> Self {
        self.kp = kp;
        self
    }

    /// Configures the integral gain of the [`PidConfig`] to be built. No validation is performed
    /// until [`build`](Self::build) is called.
    ///
    /// You pass in a time-invariant `ki`, which is scaled by the sample time to produce
    /// the internal `ki` value
    ///
    /// # Arguments
    /// - `ki`: Integral gain.
    pub fn ki(mut self, total_ki: F) -> Self {
        self.total_ki = total_ki;
        self
    }

    /// Configures the derivative gain of the [`PidConfig`] to be built. No validation is performed
    /// until [`build`](Self::build) is called.
    ///
    /// You pass in a time-invariant `kd`, which is scaled inversely by the sample time to produce
    /// the internal `kd` value
    ///
    /// # Arguments
    /// - `kd`: Derivative gain.
    pub fn kd(mut self, total_kd: F) -> Self {
        self.total_kd = total_kd;
        self
    }

    /// Configures the time constant for the low-pass filter applied to the derivative term of the
    /// [`PidConfig`] to be built. No validation is performed until [`build`](Self::build) is
    /// called.
    ///
    /// # Arguments
    /// - `filter_tc`: Time constant for the filter.
    pub fn filter_tc(mut self, filter_tc: F) -> Self {
        self.filter_tc = filter_tc;
        self
    }

    /// Configures the sample time for the [`PidConfig`] to be built. No validation is performed
    /// until [`build`](Self::build) is called.
    ///
    /// # Arguments
    /// - `sample_time`: Sample time.
    pub fn sample_time(mut self, sample_time: Duration) -> Self {
        self.sample_time = sample_time;
        self
    }

    /// Configures the output limits for the [`PidConfig`] to be built. No validation is performed
    /// until [`build`](Self::build) is called.
    ///
    /// # Arguments
    /// - `min`: Minimum output limit.
    /// - `max`: Maximum output limit.
    pub fn output_limits(mut self, min: F, max: F) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// Configures whether to use a strict causal integrator for the [`PidConfig`] to be built.
    pub fn use_strict_causal_integrator(mut self, enabled: bool) -> Self {
        self.use_strict_causal_integrator = enabled;
        self
    }

    /// Configures whether to apply the derivative on the measurement for the [`PidConfig`] to be
    /// built.
    pub fn use_derivative_on_measurement(mut self, enabled: bool) -> Self {
        self.use_derivative_on_measurement = enabled;
        self
    }

    /// Builds a new `PidConfig` instance after validating the configured parameters.
    ///
    /// # Returns
    /// - `Ok(PidConfig<F>)` if the configuration is valid.
    /// - `Err(PidConfigError)` if any of the configured parameters is invalid. Refer to the
    ///   section [Invariants](PidConfig#Invariants) for details.
    pub fn build(self) -> Result<PidConfig<F>, PidConfigError> {
        check_kp(self.kp)?;
        check_ki(self.total_ki)?;
        check_kd(self.total_kd)?;
        check_filter_tc(self.filter_tc)?;
        check_sample_time(self.sample_time)?;
        check_output_limits(self.output_min, self.output_max)?;

        let delta_t = F::from(self.sample_time.as_secs_f64()).unwrap();
        let smoothing_constant = delta_t / (delta_t + self.filter_tc);

        Ok(PidConfig {
            kp: self.kp,
            ki: self.total_ki * delta_t,
            kd: self.total_kd / delta_t,
            filter_tc: self.filter_tc,
            sample_time: self.sample_time,
            output_min: self.output_min,
            output_max: self.output_max,
            use_strict_causal_integrator: self.use_strict_causal_integrator,
            use_derivative_on_measurement: self.use_derivative_on_measurement,
            smoothing_constant,
        })
    }
}

/// The activity level of the PID controller's integrator.
///
/// # Details
/// During PID operation, the integrator could be paused, or disabled entirely. Pausing the
/// integrator is useful when the integrator has already accumulated a useful amount of error
/// enough to eliminate the steady state error but the system is entering a regime where noisy
/// sensor feedback or rapid setpoint changes could cause the integrator to accumulate unwanted
/// error.
///
/// # Example
///
/// In an autopilot application, you could set integrator activity to
///
/// - [`Inactive`](crate::pid::IntegratorActivity::Inactive) when the aircraft not under autopilot control at all
/// - [`HoldIntegration`](crate::pid::IntegratorActivity::HoldIntegration) when the aircraft is
///   under autopilot control but below an altitude threshold (ground effect causes strong
///   disturbances present in that region only)
/// - [`Active`](IntegratorActivity::Active) when the aircraft is under autopilot control and above the altitude threshold
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IntegratorActivity {
    /// The integrator is disabled and the i-term set to zero
    Inactive,

    /// The integrator is paused and the i-term is held at the last value
    HoldIntegration,

    /// The integrator is active and the i-term is updated
    Active,
}

/// This is the trait that implements the core PID law via the `evaluate_pid_law` methods.
///
/// This PID law is not aware of early-retu
/// It requires the implementor to provide a few methods to query the saved working variables, e.g.
/// the integral term, last error, last input, and last derivative for computing the I and D terms.
trait PidAlgorithm<F: FloatCore> {
    fn i_term(&self) -> F;

    fn last_error(&self) -> F;

    fn last_input(&self) -> F;

    fn last_derivative(&self) -> F;

    fn integrator_activity(&self) -> IntegratorActivity;

    #[must_use]
    fn evaluate_pid_law(
        &self,
        config: &PidConfig<F>,
        error: F,
        input: F,
        feedforward: Option<F>,
    ) -> (F, F, F) {
        let mut i_term = self.i_term();
        if !config.use_strict_causal_integrator {
            i_term = self.compute_i_term(config, error);
        }
        // Optional derivative on measurement to mitigate derivative kick
        let raw_derivative = if config.use_derivative_on_measurement {
            self.last_input() - input // Note reversed order of operands
        } else {
            error - self.last_error()
        };

        // Pass the derivative through a first-order LPF
        let derivative = config.smoothing_constant * raw_derivative
            + (F::one() - config.smoothing_constant) * self.last_derivative();

        let mut output =
            config.kp * error + i_term + config.kd * derivative + feedforward.unwrap_or(F::zero());
        output = output.clamp(config.output_min, config.output_max);

        if config.use_strict_causal_integrator {
            i_term = self.compute_i_term(config, error);
        }
        (output, i_term, derivative)
    }

    #[must_use]
    fn compute_i_term(&self, config: &PidConfig<F>, error: F) -> F {
        match self.integrator_activity() {
            IntegratorActivity::Inactive => F::zero(),
            IntegratorActivity::HoldIntegration => self.i_term(),
            IntegratorActivity::Active => {
                (self.i_term() + config.ki * error).clamp(config.output_min, config.output_max)
            }
        }
    }
}

/// A container for mutable state and runtime parameters of PID controllers.
///
/// This struct is used to store the internal state of the PID controller. You can query various
/// useful information from this struct, such as he [`last_time`](PidContext::last_time) a PID
/// computation was performed and the [`output`](PidContext::output) and
/// [`error`](PidContext::error) at that time. Furthermore, you can manage PID runtime behavior
/// such as [`activate`](PidContext::activate)-ing the PID, resetting the integral term, and pausing/resuming the
/// integrator by calling appropriate methods on this struct.
#[derive(Copy, Clone, Debug)]
pub struct PidContext<T: InstantLike, F: FloatCore> {
    output: F,
    i_term: F,
    last_err: F,
    last_input: F,
    last_derivative: F,
    last_time: Option<T>,
    integrator_activity: IntegratorActivity,
    is_active: bool,
    is_initialized: bool,
}

impl<T: InstantLike, F: FloatCore> PidAlgorithm<F> for PidContext<T, F> {
    fn i_term(&self) -> F {
        self.i_term
    }

    fn last_error(&self) -> F {
        self.last_err
    }

    fn last_input(&self) -> F {
        self.last_input
    }

    fn last_derivative(&self) -> F {
        self.last_derivative
    }

    fn integrator_activity(&self) -> IntegratorActivity {
        self.integrator_activity()
    }
}

impl<T: InstantLike, F: FloatCore> PidContext<T, F> {
    /// Creates a new uninitialized PID context. This context is automatically initialized when the
    /// PID controller computes its first output.
    pub fn new_uninit() -> Self {
        Default::default()
    }

    /// Creates a new PID context initialized to the given timestamp, input, and output values.
    /// This initialization ensures smooth continuity of the PID controller's output under the
    /// following assumptions:
    ///
    /// * The input (process value) is at a steady state (or minimally changing)
    /// * There is no transient error at the time of initialization, such that the output is purely
    ///   contributed by the integral term, which could be steady-state error or externally
    ///   measured disturbance
    ///
    /// # Arguments
    /// - `timestamp`: The timestamp of the initialization.
    /// - `input`: The input (process value) at the time of initialization.
    /// - `output`: The output of the PID controller at the time of initialization.
    pub fn new(timestamp: T, input: F, output: F) -> Self {
        Self {
            i_term: output,
            last_err: F::zero(),
            last_input: input,
            output,
            last_derivative: F::zero(),
            last_time: Some(timestamp),
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: true,
        }
    }

    /// Queries the last output value. This is available for the user to query, and is also used to hold
    /// the last output unchanged when the PID controller is inactive
    pub fn output(&self) -> F {
        self.output
    }

    /// Queries the last time the PID controller was computed. This is compared to the current time to
    /// determine if `sample_time` has elapsed since the last computation and the controller should
    /// compute a new output.
    ///
    /// # Returns
    /// - `Some(T)` if the PID controller has been run before.
    /// - `None` if the PID controller has never been run before.
    pub fn last_time(&self) -> Option<T> {
        self.last_time
    }

    /// Queries whether the PID controller is active. If false, the output will not be
    /// updated and the last output will be held unchanged.
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    /// Queries the integrator activity level of the PID controller.
    pub fn integrator_activity(&self) -> IntegratorActivity {
        self.integrator_activity
    }

    /// Queries whether the PID controller has been initialized. This is used to
    /// reinitialize the controller when upon being activated
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
        let is_active = self.integrator_activity != IntegratorActivity::Inactive;
        if is_active && activity == IntegratorActivity::Inactive {
            self.reset_integral();
        }

        self.integrator_activity = activity;
    }
}

impl<T: InstantLike, F: FloatCore> Default for PidContext<T, F> {
    fn default() -> Self {
        Self {
            i_term: F::zero(),
            last_err: F::zero(),
            last_input: F::zero(),
            output: F::zero(),
            last_derivative: F::zero(),
            last_time: None,
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: false,
        }
    }
}

/// A functional PID controller.
pub struct FuncPidController<F: FloatCore> {
    config: PidConfig<F>,
}

impl<F: FloatCore> FuncPidController<F> {
    /// Creates a new `FuncPidController` instance with the given configuration.
    ///
    /// # Arguments
    /// - `config`: The PID configuration.
    pub fn new(config: PidConfig<F>) -> Self {
        FuncPidController { config }
    }

    /// Returns the PID configuration.
    pub fn config(&self) -> &PidConfig<F> {
        &self.config
    }

    /// Returns a mutable reference to the PID configuration. This allows you to modify the
    /// configuration and tune the PID controller on-the-fly
    pub fn config_mut(&mut self) -> &mut PidConfig<F> {
        &mut self.config
    }

    /// Computes the PID control output based on the given input, setpoint, and timestamp, and
    /// optionally a feedforward term.
    ///
    /// # Arguments
    /// - `ctx`: The PID context containing the current state of the controller.
    /// - `input`: The current process variable (PV) or input value.
    /// - `setpoint`: The desired setpoint or target value.
    /// - `timestamp`: The current timestamp.
    /// - `feedforward`: An optional feedforward term to be added to the output.
    ///
    /// # Returns
    /// - A tuple containing the computed output and the updated PID context.
    pub fn compute<T: InstantLike>(
        &self,
        mut ctx: PidContext<T, F>,
        input: F,
        setpoint: F,
        timestamp: T,
        feedforward: Option<F>,
    ) -> (F, PidContext<T, F>) {
        if !ctx.is_active {
            return (ctx.output, ctx);
        }

        let error = setpoint - input;

        // If the PID controller is just switched active or has never been run before (last_time is
        // None), initialize the state then run the controller without checking if the sample time
        // has elapsed
        if !ctx.is_initialized {
            ctx.last_err = error;
            ctx.last_input = input;
            ctx.i_term = ctx
                .output
                .clamp(self.config.output_min, self.config.output_max);
            ctx.is_initialized = true;
        } else {
            // If there is no need to initialize and the controller has been called before, check if the
            // time delta is less than the sample time
            if timestamp.duration_since(ctx.last_time.unwrap()) < self.config.sample_time {
                return (ctx.output, ctx);
            }
        }

        (ctx.output, ctx.i_term, ctx.last_derivative) =
            ctx.evaluate_pid_law(&self.config, error, input, feedforward);
        ctx.last_err = error;
        ctx.last_input = input;
        ctx.last_time = Some(timestamp);
        (ctx.output, ctx)
    }
}

/// A stateful PID controller.
pub struct PidController<T: InstantLike, F: FloatCore> {
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
    output: F,

    /// The last derivative value. This is used by the low-pass filter to smooth the derivative
    /// Users should not access this directly
    derivative: F,

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

    config: PidConfig<F>,
}

impl<T: InstantLike, F: FloatCore> PidController<T, F> {
    /// Creates a new PID controller with the given configuration without initializing working
    /// variables
    pub fn new_uninit(config: PidConfig<F>) -> Self {
        Self {
            i_term: F::zero(),
            last_err: F::zero(),
            last_input: F::zero(),
            output: F::zero(),
            derivative: F::zero(),
            last_time: None,
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: false,
            config,
        }
    }

    /// Creates a new PID controller with the given configuration and initializes the working
    /// variables. Refer to [`PidContext::new`].
    pub fn new(config: PidConfig<F>, timestamp: T, input: F, output: F) -> Self {
        Self {
            i_term: output.clamp(config.output_min, config.output_max),
            last_err: F::zero(),
            last_input: input,
            output: output.clamp(config.output_min, config.output_max),
            derivative: F::zero(),
            last_time: Some(timestamp),
            is_active: true,
            integrator_activity: IntegratorActivity::Active,
            is_initialized: true,
            config,
        }
    }

    /// Returns the PID configuration.
    pub fn config(&self) -> &PidConfig<F> {
        &self.config
    }

    /// Returns a mutable reference to the PID configuration. This allows you to modify the
    /// configuration and tune the PID controller on-the-fly
    pub fn config_mut(&mut self) -> &mut PidConfig<F> {
        &mut self.config
    }

    /// Computes the PID control output based on the given input, setpoint, and timestamp, and
    /// optionally a feedforward term.
    ///
    /// # Arguments
    /// - `input`: The current process variable (PV) or input value.
    /// - `setpoint`: The desired setpoint or target value.
    /// - `timestamp`: The current timestamp.
    /// - `feedforward`: An optional feedforward term to be added to the output.
    ///
    /// # Returns
    /// - The computed output of the PID controller.
    pub fn compute(&mut self, input: F, setpoint: F, timestamp: T, feedforward: Option<F>) -> F {
        if !self.is_active {
            return self.output;
        }

        let error = setpoint - input;

        // If the PID controller is just switched active or has never been run before (last_time is
        // None), initialize the state then run the controller without checking if the sample time
        // has elapsed
        if !self.is_initialized {
            self.last_err = error;
            self.last_input = input;
            self.i_term = self
                .output
                .clamp(self.config.output_min, self.config.output_max);
            self.is_initialized = true;
        } else {
            // If there is no need to initialize and the controller has been called before, check if the
            // time delta is less than the sample time
            if timestamp.duration_since(self.last_time.unwrap()) < self.config.sample_time {
                return self.output;
            }
        }
        (self.output, self.i_term, self.derivative) =
            self.evaluate_pid_law(&self.config, error, input, feedforward);

        self.last_err = error;
        self.last_input = input;
        self.last_time = Some(timestamp);
        self.output
    }

    /// Queries the last output value. This is available for the user to query, and is also used to hold
    /// the last output unchanged when the PID controller is inactive
    pub fn output(&self) -> F {
        self.output
    }

    /// Queries the last time the PID controller was computed. This is compared to the current time to
    /// determine if `sample_time` has elapsed since the last computation and the controller should
    /// compute a new output.
    ///
    /// # Returns
    /// - `Some(T)` if the PID controller has been run before.
    /// - `None` if the PID controller has never been run before.
    pub fn last_time(&self) -> Option<T> {
        self.last_time
    }

    /// Queries whether the PID controller is active. If false, the output will not be
    /// updated and the last output will be held unchanged.
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    /// Queries the integrator activity level of the PID controller.
    pub fn integrator_activity(&self) -> IntegratorActivity {
        self.integrator_activity
    }

    /// Queries whether the PID controller has been initialized. This is used to
    /// reinitialize the controller when upon being activated
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
        let is_active = self.integrator_activity != IntegratorActivity::Inactive;
        if is_active && activity == IntegratorActivity::Inactive {
            self.reset_integral();
        }

        self.integrator_activity = activity;
    }
}

impl<T: InstantLike, F: FloatCore> PidAlgorithm<F> for PidController<T, F> {
    fn i_term(&self) -> F {
        self.i_term
    }

    fn last_error(&self) -> F {
        self.last_err
    }

    fn last_input(&self) -> F {
        self.last_input
    }

    fn last_derivative(&self) -> F {
        self.derivative
    }

    fn integrator_activity(&self) -> IntegratorActivity {
        self.integrator_activity()
    }
}
