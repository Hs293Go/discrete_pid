use robust_pid::pid::{FuncPidController, PidActivity, PidConfig, PidContext};
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

fn make_controller() -> (FuncPidController, PidContext) {
    let mut config = PidConfig::default();
    config.set_output_limits(-10.0, 10.0);
    let controller = FuncPidController::new(config);
    let mut ctx = PidContext::new(Instant::now());
    ctx.set_activity_level(PidActivity::Active);
    (controller, ctx)
}

#[test]
fn test_get_and_set_kp() {
    let (mut pid, _) = make_controller();
    let config = pid.config_mut();

    // Default kp is 1
    assert_eq!(config.kp(), 1.0);

    const NEW_KP: f64 = 10.0;
    // Set a new kp
    assert!(config.set_kp(NEW_KP));
    assert_eq!(config.kp(), NEW_KP);

    // Zero, negative and non-finite kp are invalid
    const INVALID_VALUES: &[f64; 4] = &[0.0, -1.0, f64::INFINITY, f64::NAN];

    for it in INVALID_VALUES {
        // Setting negative kp should fail
        assert!(!config.set_kp(*it));

        // Failing to set kp should not change the value
        assert_eq!(config.kp(), NEW_KP);
    }
}

#[test]
fn test_get_and_set_ki() {
    let (mut pid, _) = make_controller();
    let config = pid.config_mut();

    // Default ki is 1
    assert_eq!(config.ki(), 1.0);

    const NEW_KI: f64 = 10.0;
    // Set a new ki
    assert!(config.set_ki(NEW_KI));
    assert_eq!(config.ki(), NEW_KI);

    // Changing sample time does not change total ki
    config.set_sample_time(Duration::from_millis(150));
    assert_eq!(config.ki(), NEW_KI);

    // Negative and non-finite ki are invalid
    const INVALID_VALUES: &[f64; 3] = &[-1.0, f64::INFINITY, f64::NAN];

    for it in INVALID_VALUES {
        assert!(!config.set_ki(*it));

        // Failing to set ki should not change the value
        assert_eq!(config.ki(), NEW_KI);
    }

    // Zero ki is valid
    assert!(config.set_ki(0.0));
    assert_eq!(config.ki(), 0.0);
}

#[test]
fn test_get_and_set_kd() {
    let (mut pid, _) = make_controller();
    let config = pid.config_mut();

    // Default kd is 0
    assert_eq!(config.kd(), 0.0);

    const NEW_KD: f64 = 10.0;
    // Set a new kd
    config.set_kd(NEW_KD);
    assert_eq!(config.kd(), NEW_KD);

    // Changing sample time does not change total kd
    config.set_sample_time(Duration::from_millis(150));
    assert_eq!(config.kd(), NEW_KD);

    // Negative and non-finite kd are invalid
    const INVALID_VALUES: &[f64; 3] = &[-1.0, f64::INFINITY, f64::NAN];

    for it in INVALID_VALUES {
        assert!(!config.set_kd(*it));

        // Failing to set kd should not change the value
        assert_eq!(config.kd(), NEW_KD);
    }

    // Zero kd is valid
    assert!(config.set_kd(0.0));
    assert_eq!(config.kd(), 0.0);
}

#[test]
fn test_get_and_set_filter_tc() {
    let (mut pid, _) = make_controller();
    let config = pid.config_mut();

    // Default filter time constant is 0
    assert_eq!(config.filter_tc(), 0.01);

    const NEW_TC: f64 = 10.0;
    // Set a new filter time constant
    assert!(config.set_filter_tc(NEW_TC));
    assert_eq!(config.filter_tc(), NEW_TC);

    // Negative and non-finite filter time constants are invalid
    const INVALID_VALUES: &[f64; 4] = &[-1.0, 0.0, f64::NAN, f64::INFINITY];

    for it in INVALID_VALUES {
        assert!(!config.set_filter_tc(*it));

        // Failing to set filter time constant should not change the value
        assert_eq!(config.filter_tc(), NEW_TC);
    }
}

#[test]
fn test_get_and_set_sample_time() {
    let (mut pid, _) = make_controller();
    let config = pid.config_mut();

    // Default sample time is 10ms
    assert_eq!(config.sample_time(), Duration::from_millis(10));

    const NEW_SAMPLE_TIME: Duration = Duration::from_millis(100);
    let gains = (config.kp(), config.ki(), config.kd());
    // Set a new sample time
    assert!(config.set_sample_time(NEW_SAMPLE_TIME));
    assert_eq!(config.sample_time(), NEW_SAMPLE_TIME);

    // Changing sample time does not change any of kp/ki/kd
    let gains_round_trip = (config.kp(), config.ki(), config.kd());
    assert_eq!(gains, gains_round_trip);

    // Negative and non-finite sample times are invalid
    const INVALID_VALUES: &[Duration; 2] = &[Duration::from_millis(0), Duration::MAX];

    for it in INVALID_VALUES {
        assert!(!config.set_sample_time(*it));

        // Failing to set sample time should not change the value
        assert_eq!(config.sample_time(), NEW_SAMPLE_TIME);
    }
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
