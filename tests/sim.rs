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
mod fixtures;

#[cfg(feature = "simulation")]
mod data;

#[cfg(feature = "simulation")]
mod test_pid_numerical_performance {
    use super::data::*;
    use super::fixtures::test_pid::*;

    use core::time::Duration;
    use discrete_pid::pid::*;
    use discrete_pid::sim;
    use discrete_pid::time::Millis;

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

    /// This test builds upon `test_simulink_open_loop_behavior_compliance` and ensures the
    /// stateful PID controller behaves the same as the functional PID controller numerically
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
