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

#[cfg(feature = "simulation")]
mod data;

#[cfg(feature = "simulation")]
use discrete_pid::sim;

use discrete_pid::pid::{
    FuncPidController, IntegratorActivity, PidConfig, PidConfigBuilder, PidConfigError, PidContext,
    PidController,
};

use discrete_pid::time::Millis;
use std::time::Duration;

mod test_pid {

    use super::*;

    pub fn make_controller() -> (FuncPidController<f64>, PidContext<Millis, f64>) {
        let config = PidConfig::default();
        let controller = FuncPidController::new(config);
        let ctx = PidContext::new_uninit();
        (controller, ctx)
    }

    pub fn make_stateful_controller() -> PidController<Millis, f64> {
        let config = PidConfig::default();
        PidController::new_uninit(config)
    }

    pub fn get_next_timestamp(
        pid: &FuncPidController<f64>,
        ctx: &PidContext<Millis, f64>,
    ) -> Millis {
        let now = ctx.last_time().unwrap_or(Millis(0));
        now + pid.config().sample_time()
    }

    pub fn get_next_timestamp_stateful(pid: &PidController<Millis, f64>) -> Millis {
        pid.last_time().unwrap_or(Millis(0)) + pid.config().sample_time()
    }
}

mod test_pid_config {

    use super::test_pid::make_controller;
    use super::*;

    const NEW_KP: f64 = 10.0;
    // Zero, negative and non-finite kp are invalid
    const INVALID_KP_VALUES: &[f64; 4] = &[0.0, -1.0, f64::INFINITY, f64::NAN];

    #[test]
    fn test_get_and_set_kp() {
        let (mut pid, _) = make_controller();
        let config = pid.config_mut();

        // Default kp is 1
        assert_eq!(config.kp(), 1.0);

        // Set a new kp
        assert!(config.set_kp(NEW_KP).is_ok());
        assert_eq!(config.kp(), NEW_KP);

        for it in INVALID_KP_VALUES {
            // Setting negative kp should fail
            assert_eq!(
                config.set_kp(*it),
                Err(PidConfigError::InvalidProportionalGain)
            );

            // Failing to set kp should not change the value
            assert_eq!(config.kp(), NEW_KP);
        }
    }

    #[test]
    fn test_build_kp() {
        let mut default_init_config = PidConfig::<f64>::default();
        assert!(default_init_config.set_kp(NEW_KP).is_ok());

        let built_config = PidConfigBuilder::default().kp(NEW_KP).build();
        assert!(built_config.is_ok());
        assert_eq!(built_config.unwrap().kp(), default_init_config.kp());

        for it in INVALID_KP_VALUES {
            assert_eq!(
                PidConfigBuilder::default().kp(*it).build().map(|_| ()),
                Err(PidConfigError::InvalidProportionalGain)
            );
        }
    }

    const NEW_KI: f64 = 10.0;
    // Negative and non-finite ki are invalid
    const INVALID_KI_VALUES: &[f64; 3] = &[-1.0, f64::INFINITY, f64::NAN];

    #[test]
    fn test_get_and_set_ki() {
        let (mut pid, _) = make_controller();
        let config = pid.config_mut();

        // Default ki is 1
        assert_eq!(config.ki(), 1.0);

        // Set a new ki
        assert!(config.set_ki(NEW_KI).is_ok());
        assert_eq!(config.ki(), NEW_KI);

        // Changing sample time does not change total ki
        assert!(config.set_sample_time(Duration::from_millis(150)).is_ok());
        assert_eq!(config.ki(), NEW_KI);

        for it in INVALID_KI_VALUES {
            assert_eq!(config.set_ki(*it), Err(PidConfigError::InvalidIntegralGain));

            // Failing to set ki should not change the value
            assert_eq!(config.ki(), NEW_KI);
        }

        // Zero ki is valid
        assert!(config.set_ki(0.0).is_ok());
        assert_eq!(config.ki(), 0.0);
    }

    #[test]
    fn test_build_ki() {
        let mut default_init_config = PidConfig::<f64>::default();
        assert!(default_init_config.set_ki(NEW_KI).is_ok());

        let built_config = PidConfigBuilder::default().ki(NEW_KI).build();
        assert!(built_config.is_ok());
        assert_eq!(built_config.unwrap().ki(), default_init_config.ki());

        let built_config_2 = PidConfigBuilder::default()
            .ki(10.0)
            .sample_time(Duration::from_millis(200))
            .build();
        assert!(built_config_2.is_ok());
        assert_eq!(built_config_2.unwrap().ki(), default_init_config.ki());

        for it in INVALID_KI_VALUES {
            assert_eq!(
                PidConfigBuilder::default().ki(*it).build().map(|_| ()),
                Err(PidConfigError::InvalidIntegralGain)
            );
        }
    }

    const NEW_KD: f64 = 10.0;
    // Negative and non-finite kd are invalid
    const INVALID_KD_VALUES: &[f64; 3] = &[-1.0, f64::INFINITY, f64::NAN];

    #[test]
    fn test_get_and_set_kd() {
        let (mut pid, _) = make_controller();
        let config = pid.config_mut();

        // Default kd is 0
        assert_eq!(config.kd(), 0.0);

        // Set a new kd
        assert!(config.set_kd(NEW_KD).is_ok());
        assert_eq!(config.kd(), NEW_KD);

        // Changing sample time does not change total kd
        assert!(config.set_sample_time(Duration::from_millis(150)).is_ok());
        assert_eq!(config.kd(), NEW_KD);

        for it in INVALID_KD_VALUES {
            assert_eq!(
                config.set_kd(*it),
                Err(PidConfigError::InvalidDerivativeGain)
            );

            // Failing to set kd should not change the value
            assert_eq!(config.kd(), NEW_KD);
        }

        // Zero kd is valid
        assert!(config.set_kd(0.0).is_ok());
        assert_eq!(config.kd(), 0.0);
    }

    #[test]
    fn test_build_kd() {
        let mut default_init_config = PidConfig::<f64>::default();
        assert!(default_init_config.set_kd(NEW_KD).is_ok());

        let built_config = PidConfigBuilder::default().kd(NEW_KD).build();
        assert!(built_config.is_ok());
        assert_eq!(built_config.unwrap().kd(), default_init_config.kd());

        let built_config_2 = PidConfigBuilder::default()
            .kd(10.0)
            .sample_time(Duration::from_millis(200))
            .build();
        assert!(built_config_2.is_ok());
        assert_eq!(built_config_2.unwrap().kd(), default_init_config.kd());

        for it in INVALID_KD_VALUES {
            assert_eq!(
                PidConfigBuilder::default().kd(*it).build().map(|_| ()),
                Err(PidConfigError::InvalidDerivativeGain)
            );
        }
    }

    const NEW_TC: f64 = 10.0;
    // Negative and non-finite filter time constants are invalid
    const INVALID_FILTER_TC_VALUES: &[f64; 4] = &[-1.0, 0.0, f64::NAN, f64::INFINITY];

    #[test]
    fn test_get_and_set_filter_tc() {
        let (mut pid, _) = make_controller();
        let config = pid.config_mut();

        // Default filter time constant is 0
        assert_eq!(config.filter_tc(), 0.01);

        // Set a new filter time constant
        assert!(config.set_filter_tc(NEW_TC).is_ok());
        assert_eq!(config.filter_tc(), NEW_TC);

        for it in INVALID_FILTER_TC_VALUES {
            assert_eq!(
                config.set_filter_tc(*it),
                Err(PidConfigError::InvalidFilterTimeConstant)
            );

            // Failing to set filter time constant should not change the value
            assert_eq!(config.filter_tc(), NEW_TC);
        }
    }

    #[test]
    fn test_build_filter_tc() {
        let mut default_init_config = PidConfig::<f64>::default();
        assert!(default_init_config.set_filter_tc(NEW_TC).is_ok());

        let built_config = PidConfigBuilder::default().filter_tc(NEW_TC).build();
        assert!(built_config.is_ok());
        assert_eq!(
            built_config.unwrap().filter_tc(),
            default_init_config.filter_tc()
        );

        for it in INVALID_FILTER_TC_VALUES {
            assert_eq!(
                PidConfigBuilder::default()
                    .filter_tc(*it)
                    .build()
                    .map(|_| ()),
                Err(PidConfigError::InvalidFilterTimeConstant)
            );
        }
    }

    const NEW_SAMPLE_TIME: Duration = Duration::from_millis(100);
    // Negative and non-finite sample times are invalid
    const INVALID_SAMPLE_TIME_VALUES: &[Duration; 2] = &[Duration::from_millis(0), Duration::MAX];

    #[test]
    fn test_get_and_set_sample_time() {
        let (mut pid, _) = make_controller();
        let config = pid.config_mut();

        // Default sample time is 10ms
        assert_eq!(config.sample_time(), Duration::from_millis(10));

        let gains = (config.kp(), config.ki(), config.kd());
        // Set a new sample time
        assert!(config.set_sample_time(NEW_SAMPLE_TIME).is_ok());
        assert_eq!(config.sample_time(), NEW_SAMPLE_TIME);

        // Changing sample time does not change any of kp/ki/kd
        let gains_round_trip = (config.kp(), config.ki(), config.kd());
        assert_eq!(gains, gains_round_trip);

        for it in INVALID_SAMPLE_TIME_VALUES {
            assert_eq!(
                config.set_sample_time(*it),
                Err(PidConfigError::InvalidSampleTime)
            );

            // Failing to set sample time should not change the value
            assert_eq!(config.sample_time(), NEW_SAMPLE_TIME);
        }
    }

    #[test]
    fn test_build_sample_time() {
        let mut default_init_config = PidConfig::<f64>::default();
        assert!(default_init_config.set_sample_time(NEW_SAMPLE_TIME).is_ok());

        let built_config = PidConfigBuilder::<f64>::default()
            .sample_time(NEW_SAMPLE_TIME)
            .build();
        assert!(built_config.is_ok());
        assert_eq!(
            built_config.unwrap().sample_time(),
            default_init_config.sample_time()
        );

        for it in INVALID_SAMPLE_TIME_VALUES {
            assert_eq!(
                PidConfigBuilder::<f64>::default()
                    .sample_time(*it)
                    .build()
                    .map(|_| ()),
                Err(PidConfigError::InvalidSampleTime)
            );
        }
    }
}

mod test_pid_qualitative_performance {
    use super::test_pid::{get_next_timestamp, make_controller};
    use super::*;

    mod p_control {
        use super::*;

        #[test]
        fn test_pure_proportional_control() {
            let (mut pid, ctx) = make_controller();
            let config = pid.config_mut();
            assert!(config.set_ki(0.0).is_ok());
            assert!(config.set_kd(0.0).is_ok());

            let (_, ctx) = pid.compute(ctx, 0.5, 1.0, get_next_timestamp(&pid, &ctx), None);
            let output = ctx.output();

            assert_eq!(output, 0.5); // Assuming kp = 1.0
        }
    }

    mod i_control {
        use super::*;
        // Construct a output limit that is hit when a constant error of 10 is accumulated over 5 controller computations
        const N_STEPS: usize = 5;
        const BASE_ERROR: f64 = 10.0;

        #[test]
        fn test_integral_accumulation() {
            let (pid, mut ctx) = make_controller();
            let mut output;

            let mut outputs = vec![];

            for _ in 0..10 {
                (output, ctx) = pid.compute(ctx, 0.0, 1.0, get_next_timestamp(&pid, &ctx), None);
                outputs.push(output);
            }

            assert!(outputs.windows(2).all(|w| w[1] >= w[0])); // Output should increase as integral accumulates
        }

        #[test]
        fn test_integral_windup_and_recovery() {
            let (mut pid, mut ctx) = make_controller();
            let mut output: f64;

            let limit: f64 =
                BASE_ERROR * (1.0 + N_STEPS as f64 * pid.config().sample_time().as_secs_f64());

            assert!(pid.config_mut().set_output_limits(-limit, limit).is_ok());

            for i in 0..10 {
                // Setpoint is constant, but input (process value) is held unchanged, so integral
                // should accumulate
                (output, ctx) =
                    pid.compute(ctx, 0.0, BASE_ERROR, get_next_timestamp(&pid, &ctx), None);
                if i >= N_STEPS {
                    assert_eq!(output, limit);
                } else {
                    assert!(output < limit);
                }
            }

            // Relatively small setpoint change in the opposite direction
            (output, _) = pid.compute(ctx, 0.0, -0.1, get_next_timestamp(&pid, &ctx), None);
            // Output should immediate respond by decreasing
            assert!(output < limit, "Expected reversal due to anti-windup");
        }

        #[test]
        fn test_integral_deactivation_resets_iterm() {
            let (mut pid, mut ctx) = make_controller();
            let mut output: f64;

            // Construct a output limit that is hit when a constant error of 10 is accumulated over 5 controller computations
            let limit: f64 =
                BASE_ERROR * (1.0 + N_STEPS as f64 * pid.config().sample_time().as_secs_f64());

            assert!(pid.config_mut().set_output_limits(-limit, limit).is_ok());

            for i in 0..10 {
                // Setpoint is constant, but input (process value) is held unchanged, so integral
                // should accumulate

                // Deactivate integration just before the integral term causes the output to hit the limit
                if i == N_STEPS - 1 {
                    ctx.set_integrator_activity(IntegratorActivity::Inactive);
                }

                (output, ctx) =
                    pid.compute(ctx, 0.0, BASE_ERROR, get_next_timestamp(&pid, &ctx), None);

                // Then reactivate integration
                if i == N_STEPS - 1 {
                    ctx.set_integrator_activity(IntegratorActivity::Active);
                }

                assert!(output < limit);
            }
        }

        #[test]
        fn test_hold_integration_prevents_integral_growth() {
            let (pid, mut ctx) = make_controller();
            let mut output: f64;

            // Compute once with the integral active, then manually compute the i-term after this step
            (_, ctx) = pid.compute(ctx, 0.0, 8.0, get_next_timestamp(&pid, &ctx), None);
            let last_i_term =
                ctx.error() * pid.config().ki() * pid.config().sample_time().as_secs_f64();

            ctx.set_integrator_activity(IntegratorActivity::HoldIntegration);

            const SETPOINTS: [f64; 5] = [-5.0, 1.0, 0.0, 1.0, 5.0];
            for setpoint in SETPOINTS {
                // Setpoint is constant
                (output, ctx) =
                    pid.compute(ctx, 0.0, setpoint, get_next_timestamp(&pid, &ctx), None);
                // Output becomes P-term, i.e. setpoint * unity P gain + last i-term
                assert_eq!(output, setpoint + last_i_term);
            }
        }
    }

    mod d_control {
        use super::*;

        #[test]
        fn test_derivative_boosting_and_damping() {
            let (mut pid, mut ctx) = make_controller();

            assert!(pid.config_mut().set_kd(1.0).is_ok());

            // An initial step to start storing error/input
            (_, ctx) = pid.compute(ctx, 0.0, 5.0, get_next_timestamp(&pid, &ctx), None);

            const NEW_SETPOINT: f64 = 10.0;

            pid.config_mut().set_use_derivative_on_measurement(false);
            let (output_no_derivative_on_meas, _) =
                pid.compute(ctx, 1.0, NEW_SETPOINT, get_next_timestamp(&pid, &ctx), None);

            pid.config_mut().set_use_derivative_on_measurement(true);
            let (output_with_derivative_on_meas, _) =
                pid.compute(ctx, 1.0, NEW_SETPOINT, get_next_timestamp(&pid, &ctx), None);

            assert!(pid.config_mut().set_kd(0.0).is_ok());
            let (output_no_derivative, _) =
                pid.compute(ctx, 1.0, NEW_SETPOINT, get_next_timestamp(&pid, &ctx), None);

            // Compared to the output with no derivative term, the output with derivative is boosted
            assert!(output_no_derivative_on_meas > output_no_derivative);

            // Compared to the output with no derivative term, the output with derivative on measurement is dampened
            assert!(output_with_derivative_on_meas < output_no_derivative);
        }

        #[test]
        fn test_derivative_kick_mitigation() {
            let (mut pid, mut ctx) = make_controller();

            assert!(pid.config_mut().set_kd(1.0).is_ok());

            // An initial step to start storing error/input
            (_, ctx) = pid.compute(ctx, 0.0, 5.0, get_next_timestamp(&pid, &ctx), None);

            const NEW_SETPOINT: f64 = 50.0;

            pid.config_mut().set_use_derivative_on_measurement(false);
            let (output_no_derivative_on_meas, _) =
                pid.compute(ctx, 1.0, NEW_SETPOINT, get_next_timestamp(&pid, &ctx), None);

            pid.config_mut().set_use_derivative_on_measurement(true);
            let (output_with_derivative_on_meas, _) =
                pid.compute(ctx, 1.0, NEW_SETPOINT, get_next_timestamp(&pid, &ctx), None);

            assert!(output_with_derivative_on_meas < output_no_derivative_on_meas);
            // Derivative term should reduce output
        }
    }

    mod safety_and_lifecycle {
        use super::*;
        #[test]
        fn test_output_within_limits() {
            let (pid, mut ctx) = make_controller();
            let mut output;

            for _ in 0..10 {
                (output, ctx) = pid.compute(ctx, 100.0, 0.0, get_next_timestamp(&pid, &ctx), None);
                assert!(output >= pid.config().output_min());
                assert!(output <= pid.config().output_max());
            }
        }

        #[test]
        fn test_activation_and_initialization() {
            let (pid, mut ctx) = make_controller();
            let mut output: f64;

            // Compute once with the controller active, then cache the last output
            (output, ctx) = pid.compute(ctx, 0.0, 1.5, get_next_timestamp(&pid, &ctx), None);
            let last_output = output;

            ctx.deactivate();

            const SETPOINTS: [f64; 5] = [-5.0, 1.0, 0.0, 1.0, 5.0];
            const INPUTS: [f64; 5] = [0.0, 1.0, 2.0, 3.0, 4.0];
            for (input, setpoint) in INPUTS.into_iter().zip(SETPOINTS) {
                // Check that output held unchanged regardless of input setpoint when deactivated
                (output, ctx) =
                    pid.compute(ctx, input, setpoint, get_next_timestamp(&pid, &ctx), None);
                assert_eq!(output, last_output);
            }

            ctx.activate();
            (output, ctx) = pid.compute(ctx, 5.0, 7.0, get_next_timestamp(&pid, &ctx), None);

            // Check that computing after reactivation triggers controller initialization
            assert!(ctx.is_initialized());

            // Check that output is different from the last output
            assert_ne!(output, last_output);
        }

        #[test]
        fn test_deactivation_before_first_computation() {
            let (pid, mut ctx) = make_controller();
            let mut output;
            ctx.deactivate();

            // Attempt to compute with deactivated PID
            (output, ctx) = pid.compute(ctx, 0.0, 1.5, get_next_timestamp(&pid, &ctx), None);

            assert!(!ctx.is_active());
            assert!(!ctx.is_initialized());
            assert_eq!(output, 0.0);

            ctx.activate();

            (output, ctx) = pid.compute(ctx, 0.0, 1.5, get_next_timestamp(&pid, &ctx), None);

            assert!(ctx.is_active());
            assert!(ctx.is_initialized());
            assert_eq!(output, 1.5); // Assuming kp = 1.0
        }
    }
}

#[cfg(feature = "simulation")]
mod test_pid_numerical_performance {
    use super::data::*;
    use super::test_pid::*;
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra as na;

    fn configure_pid_nondefault(pid: &mut FuncPidController<f64>) {
        assert!(pid.config_mut().set_kp(10.0).is_ok());
        assert!(pid.config_mut().set_ki(20.0).is_ok());
        assert!(pid.config_mut().set_kd(1.0).is_ok());
        assert!(pid.config_mut().set_filter_tc(0.02).is_ok());
    }

    /// To recreate these test results, create the following simulink model
    ///
    /// ┌───────────┐       ┌─────────────────────────┐        ┌───────────┐
    /// │ sine wave │       │ Discrete PID            │        │ To        │
    /// │           │───────│ Filt. Met.: Back. Euler │────────│ Workspace │
    /// │ (default) │       │ Derivative: 0.01        │        │           │
    /// └───────────┘       │ Filter Coeff: 50        │        └───────────┘
    ///                     └─────────────────────────┘        
    ///
    /// and using the ODE1 (Euler) solver with a fixed step size of 0.01
    ///
    /// Then copy the saved data in intervals of 20 steps, i.e. 1:20:end
    /// Though this is an open-loop test, this test effectively engages the D-term and the filter in
    /// the D-term, since both are configured with non-default parameters.
    #[test]
    fn test_simulink_open_loop_behavior_compliance() {
        let (mut pid, mut ctx) = make_controller();

        // Set non-default D-gain and filter time constant
        configure_pid_nondefault(&mut pid);

        let mut output: f64;

        let sine = sim::SignalGenerator::new(sim::WaveForm::Sine, Millis(0), 1.0, 0.0);

        for i in 0..1000usize {
            let last_time = ctx.last_time().unwrap_or(Millis(0));
            let timestamp = last_time + Duration::from_millis(FIXED_STEP_SIZE_MS);

            let sine_signal = sine.generate(last_time);
            (output, ctx) = pid.compute(ctx, 0.0, sine_signal, timestamp, None);
            if i % DOWNSAMPLE_FACTOR == 0 {
                let expected = OL_SINE_RESPONSE[i / DOWNSAMPLE_FACTOR];
                assert_relative_eq!(output, expected, epsilon = 1e-12);
            }
        }
    }

    /// To recreate these test results, create the following simulink model
    ///
    /// ┌───────────┐       ┌─────────────────────────┐       ┌─────────────┐       ┌───────────┐
    /// │ sine wave │       │ Discrete PID            │       │ State space │       │ To        │
    /// │           │──+X───│ Filt. Met.: Back. Euler │───────│ Ax + Bu     │───┬───│ Workspace │
    /// │ (default) │   -   │ Derivative: 0.01        │       │ Cx + Du     │   │   │           │
    /// └───────────┘   │   │ Filter Coeff: 50        │       └─────────────┘   │   └───────────┘
    ///                 │   └─────────────────────────┘                         │
    ///                 └───────────────────────────────────────────────────────┘
    ///
    /// where A = [0, 1; -4 * pi^2, -0.8 * 2 * pi], B = [0; 4 * pi^2], C = [1, 0], D = [0],
    /// the state-space realization of the mass-spring-damper system:
    ///
    /// x'' + 2ζωₙx' + ωₙ²x = u,     ωₙ = 2π, ζ = 0.2
    ///
    /// and using the ODE1 (Euler) solver with a fixed step size of 0.01
    ///
    /// Then copy the saved data in intervals of 20 steps, i.e. 1:20:end
    /// This is a closed-loop test, the output of the PID is fed back to the input of a LTI system
    /// (a mass-spring-damper system). This test validates the PID controller's ability to
    /// stabilize and control nontrivial systems like the Simulink PID block does.
    #[test]
    fn test_simulink_closed_loop_behavior_compliance() {
        let (mut pid, mut ctx) = make_controller();

        // Set non-default D-gain and filter time constant
        configure_pid_nondefault(&mut pid);

        let mut state = na::vector![0.0, 0.0];
        let mut control: f64;
        let mut output: f64 = 0.0;

        let mdl = sim::MassSpringDamper {
            natural_frequency: 2.0 * std::f64::consts::PI,
            damping_ratio: 0.2,
        };

        let sine = sim::SignalGenerator::new(sim::WaveForm::Sine, Millis(0), 1.0, 0.0);

        const FIXED_STEP_SIZE_S: f64 = FIXED_STEP_SIZE_MS as f64 * 0.001;

        for i in 0..1000usize {
            let last_time = ctx.last_time().unwrap_or(Millis(0));
            let timestamp = last_time + Duration::from_millis(FIXED_STEP_SIZE_MS);

            (control, ctx) = pid.compute(ctx, output, sine.generate(last_time), timestamp, None);
            if i % DOWNSAMPLE_FACTOR == 0 {
                let expected = CL_SINE_RESPONSE[i / DOWNSAMPLE_FACTOR];
                assert_relative_eq!(output, expected, epsilon = 1e-12);
            }
            state += mdl.f(state, control) * FIXED_STEP_SIZE_S;
            output = mdl.h(state);
        }
    }
}

mod test_stateful_pid {
    use super::test_pid::*;
    use super::*;

    #[cfg(feature = "simulation")]
    use super::data::*;

    #[cfg(feature = "simulation")]
    use nalgebra as na;

    #[test]
    fn test_lazy_initialization_and_reset() {
        let mut pid = make_stateful_controller();

        // Should be uninitialized initially
        assert!(!pid.is_initialized());

        let timestamp = Millis(0);
        let _ = pid.compute(0.0, 1.0, timestamp, None);

        // Should be initialized after first compute
        assert!(pid.is_initialized());
        assert!(pid.is_active());

        pid.deactivate();
        assert!(!pid.is_active());

        pid.activate();
        assert!(pid.is_active());
    }

    #[test]
    fn test_integrator_control_flow() {
        let mut pid = make_stateful_controller();

        let base_error = 2.0;
        let _ = pid.compute(0.0, base_error, Millis(0), None);

        pid.set_integrator_activity(IntegratorActivity::HoldIntegration);

        const NUM_ITERS: usize = 10;
        let held_output = (0..NUM_ITERS).fold(0.0, |_, _| {
            pid.compute(0.0, base_error, get_next_timestamp_stateful(&pid), None)
        });

        pid.set_integrator_activity(IntegratorActivity::Inactive);
        let inactive_output = (0..NUM_ITERS).fold(0.0, |_, _| {
            pid.compute(0.0, base_error, get_next_timestamp_stateful(&pid), None)
        });

        pid.set_integrator_activity(IntegratorActivity::Active);

        let active_output = (0..NUM_ITERS).fold(0.0, |_, _| {
            pid.compute(0.0, base_error, get_next_timestamp_stateful(&pid), None)
        });

        // Outputs should differ because integrator is reset and resumes
        assert!(active_output > inactive_output, "Expected active output ({active_output}) to be greater than inactive output ({inactive_output})");
        assert!(held_output > inactive_output, "Expected held output ({held_output}) to be greater than inactive output ({inactive_output})");
    }

    #[cfg(feature = "simulation")]
    #[test]
    fn test_forwarding_to_stateful_pid_open_loop_numerical_equivalence() {
        let (mut func_pid, mut ctx) = make_controller();

        let mut stateful_pid = make_stateful_controller();

        // Set non-default D-gain and filter time constant
        assert!(func_pid.config_mut().set_kd(0.01).is_ok());
        assert!(func_pid.config_mut().set_filter_tc(0.02).is_ok());
        assert!(stateful_pid.config_mut().set_kd(0.01).is_ok());
        assert!(stateful_pid.config_mut().set_filter_tc(0.02).is_ok());

        let mut expected: f64;

        for _ in 0..1000usize {
            let last_time = ctx.last_time().unwrap_or(Millis(0));
            let timestamp = last_time + Duration::from_millis(FIXED_STEP_SIZE_MS);

            let sine_signal = sim::sine_signal(last_time, Millis(0));
            (expected, ctx) = func_pid.compute(ctx, 0.0, sine_signal, timestamp, None);

            let result = stateful_pid.compute(0.0, sine_signal, timestamp, None);

            assert_eq!(result, expected);
        }
    }

    /// This test builds upon `test_simulink_closed_loop_behavior_compliance` and ensures the
    /// stateful PID controller behaves the same as the functional PID controller numerically
    #[cfg(feature = "simulation")]
    #[test]
    fn test_forwarding_to_stateful_pid_closed_loop_numerical_equivalence() {
        let (mut func_pid, mut ctx) = make_controller();

        let mut stateful_pid = make_stateful_controller();

        // Set non-default D-gain and filter time constant
        assert!(func_pid.config_mut().set_kd(0.01).is_ok());
        assert!(func_pid.config_mut().set_filter_tc(0.02).is_ok());
        assert!(stateful_pid.config_mut().set_kd(0.01).is_ok());
        assert!(stateful_pid.config_mut().set_filter_tc(0.02).is_ok());

        let mut state = na::vector![0.0, 0.0];
        let mut expected: f64;
        let mut output: f64 = 0.0;

        let mdl = sim::MassSpringDamper {
            natural_frequency: 2.0 * std::f64::consts::PI,
            damping_ratio: 0.2,
        };

        const FIXED_STEP_SIZE_S: f64 = FIXED_STEP_SIZE_MS as f64 * 0.001;

        for _ in 0..1000usize {
            let last_time = ctx.last_time().unwrap_or(Millis(0));
            let timestamp = last_time + Duration::from_millis(FIXED_STEP_SIZE_MS);

            let sine_signal = sim::sine_signal(last_time, Millis(0));
            (expected, ctx) = func_pid.compute(ctx, output, sine_signal, timestamp, None);

            let result = stateful_pid.compute(output, sine_signal, timestamp, None);
            assert_eq!(result, expected);
            state += mdl.f(state, expected) * FIXED_STEP_SIZE_S;
            output = mdl.h(state);
        }
    }
}
