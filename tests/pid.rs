use robust_pid::pid::{Pid, PidActivity, PidConfig, PidContext};
mod data;
use std::time::{Duration, Instant};

use num::Float;

struct Tolerances<T: Float> {
    pub relative: T,
    pub absolute: T,
}

impl<T: Float> Default for Tolerances<T> {
    fn default() -> Self {
        Tolerances {
            relative: T::from(1e-5).expect("Failed to create T from 1e-5"),
            absolute: T::from(1e-8).expect("Failed to create T from 1e-8"),
        }
    }
}

fn is_close<T: Float>(a: T, b: T, tolerance: Tolerances<T>) -> bool {
    if a == b {
        return true;
    }

    let diff = (a - b).abs();
    let norm = (a.abs() + b.abs()).min(T::max_value());
    // or even faster: std::min(std::abs(a + b), std::numeric_limits<float>::max());
    // keeping this commented out until I update figures below
    diff < tolerance.absolute.max(tolerance.relative * norm)
}

fn make_controller() -> (Pid, PidContext) {
    let mut config = PidConfig::default();
    config.set_output_limits(-10.0, 10.0);
    let controller = Pid::new(config);
    let mut ctx = PidContext::new(Instant::now());
    ctx.set_activity_level(PidActivity::Active);
    (controller, ctx)
}

#[test]
fn test_output_within_limits() {
    let (pid, mut ctx) = make_controller();
    let now = Instant::now();

    let mut output;
    for _ in 0..10 {
        (output, ctx) = pid.compute(ctx, 100.0, 0.0, now + Duration::from_millis(200), None);
        assert!(output >= pid.config().output_min());
        assert!(output <= pid.config().output_max());
    }
}

#[test]
fn test_integral_accumulation() {
    let (pid, mut ctx) = make_controller();
    let now = Instant::now();

    let mut outputs = vec![];

    let mut output;
    for i in 0..10 {
        (output, ctx) = pid.compute(
            ctx,
            0.0, // input
            1.0, // constant setpoint
            now + Duration::from_millis(100 * (i + 1) as u64),
            None,
        );
        outputs.push(output);
    }

    assert!(outputs.windows(2).all(|w| w[1] >= w[0])); // Output should increase as integral accumulates
}

#[test]
fn test_derivative_response() {
    let (pid, ctx) = make_controller();
    let now = Instant::now();

    // Initial step
    let (_, ctx) = pid.compute(ctx, 0.0, 1.0, now + Duration::from_millis(100), None);

    // No change
    let (output2, ctx) = pid.compute(ctx, 0.0, 1.0, now + Duration::from_millis(200), None);

    // Sudden change in input
    let (output3, _) = pid.compute(ctx, 1.0, 1.0, now + Duration::from_millis(300), None);

    assert!(output3 < output2); // Derivative term should reduce output
}

#[test]
fn test_hold_integration_prevents_integral_growth() {
    let (pid, mut ctx) = make_controller();
    let now = Instant::now();

    ctx.set_activity_level(PidActivity::HoldIntegration);

    let (output1, ctx1) = pid.compute(ctx, 0.0, 1.0, now + Duration::from_millis(100), None);
    let (output2, _) = pid.compute(ctx1, 0.0, 1.0, now + Duration::from_millis(200), None);

    // Check that integral didn't increase much
    assert!((output2 - output1).abs() < 1e-3);
}

#[test]
fn test_initialize_on_reactivation() {
    let (pid, mut ctx) = make_controller();
    let now = Instant::now();

    ctx.set_activity_level(PidActivity::Inactive);
    let (output1, _) = pid.compute(ctx, 0.0, 1.0, now + Duration::from_millis(100), None);

    ctx.set_activity_level(PidActivity::Active);
    let (output2, _) = pid.compute(ctx, 0.0, 1.0, now + Duration::from_millis(200), None);

    assert!(output1 != output2); // Controller should reinitialize and recompute
                                 // assert!(!ctx2.need_initialize); // Initialization should now be done
}

#[test]
fn test_simulink_open_loop_behavior_compliance() {
    let (pid, mut ctx) = make_controller();

    let mut output: f64;
    for i in 0..1000usize {
        let timestamp = ctx.last_time() + Duration::from_millis(data::FIXED_STEP_SIZE_MS);

        let setpoint = (i as f64 * data::FIXED_STEP_SIZE_MS as f64 / 1000.0).sin();

        (output, ctx) = pid.compute(ctx, 0.0, setpoint, timestamp, None);
        if i % 20 == 0 {
            assert!(is_close(
                output,
                data::OL_SINE_RESPONSE_50MS[i / 20usize],
                Tolerances::default()
            ));
        }
    }
}

#[test]
fn test_simulink_closed_loop_behavior_compliance() {
    let (pid, mut ctx) = make_controller();

    let gain = 0.8;

    let mut state: f64 = 0.0;
    let mut output: f64;
    for i in 0..1000usize {
        let timestamp = ctx.last_time() + Duration::from_millis(data::FIXED_STEP_SIZE_MS);
        (output, ctx) = pid.compute(ctx, state, 1.0, timestamp, None);
        if i % 20 == 0 {
            assert!(is_close(
                state,
                data::CL_STEP_RESPONSE_50MS[i / 20usize],
                Tolerances::default()
            ));
        }
        state = gain * output;
    }
}
