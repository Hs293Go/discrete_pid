//! Per-call PidModifiers + term decomposition.

use core::time::Duration;

use discrete_pid::pid::{FuncPidController, PidConfig, PidConfigBuilder, PidContext, PidModifiers};
use discrete_pid::time::Millis;

fn config(kp: f64, ki: f64) -> PidConfig<f64> {
    PidConfigBuilder::default()
        .kp(kp)
        .ki(ki)
        .kd(0.0)
        .sample_time(Duration::from_millis(10))
        .build()
        .unwrap()
}

// Drive one full step (init call, then one sample_time later) and return (output, ctx).
fn step(
    pid: &FuncPidController<f64>,
    mods: &PidModifiers<f64>,
    sp: f64,
) -> (f64, PidContext<Millis, f64>) {
    let ctx: PidContext<Millis, f64> = PidContext::new_uninit();
    let (_, ctx) = pid.compute_with_modifiers(ctx, 0.0, sp, Millis(0), None, mods);
    pid.compute_with_modifiers(ctx, 0.0, sp, Millis(10), None, mods)
}

#[test]
fn default_modifiers_match_plain_compute() {
    let pid = FuncPidController::new(config(2.0, 0.5));
    let ctx: PidContext<Millis, f64> = PidContext::new_uninit();
    let (_, ctx) = pid.compute(ctx, 0.0, 1.0, Millis(0), None);
    let (plain, _) = pid.compute(ctx, 0.0, 1.0, Millis(10), None);

    let (modified, _) = step(&pid, &PidModifiers::default(), 1.0);
    assert_eq!(plain, modified);
}

#[test]
fn p_scale_scales_the_p_term() {
    let pid = FuncPidController::new(config(2.0, 0.0));
    let mods = PidModifiers {
        p_scale: 0.5,
        ..Default::default()
    };
    let (out, ctx) = step(&pid, &mods, 1.0); // kp*error*p_scale = 2*1*0.5
    assert!((ctx.terms().p - 1.0).abs() < 1e-12);
    assert!((out - 1.0).abs() < 1e-12);
}

#[test]
fn directional_antiwindup_blocks_one_way_only() {
    let pid = FuncPidController::new(config(1.0, 1.0));

    // Baseline: positive error accumulates integral.
    let (_, base) = step(&pid, &PidModifiers::default(), 1.0);
    assert!(base.terms().i > 0.0);

    // Saturated high: positive error must not grow the integral.
    let blocked = PidModifiers {
        allow_integral_increase: false,
        ..Default::default()
    };
    let (_, held) = step(&pid, &blocked, 1.0);
    assert_eq!(held.terms().i, 0.0);

    // ...but a negative error still unwinds it.
    let unwind = PidModifiers {
        allow_integral_increase: false,
        ..Default::default()
    };
    let ctx: PidContext<Millis, f64> = PidContext::new_uninit();
    let (_, ctx) =
        pid.compute_with_modifiers(ctx, 0.0, 1.0, Millis(0), None, &PidModifiers::default());
    let (_, ctx) =
        pid.compute_with_modifiers(ctx, 0.0, 1.0, Millis(10), None, &PidModifiers::default());
    let before = ctx.terms().i;
    let (_, ctx) = pid.compute_with_modifiers(ctx, 0.0, -1.0, Millis(20), None, &unwind);
    assert!(ctx.terms().i < before);
}
