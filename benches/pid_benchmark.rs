#[cfg(feature = "bench")]
use criterion::{black_box, criterion_group, criterion_main, Criterion};

use discrete_pid::{pid, time};

#[cfg(feature = "bench")]
fn bench_discrete_pid(c: &mut Criterion) {
    let config = pid::PidConfigBuilder::default()
        .kp(1.0)
        .ki(0.5)
        .kd(0.1)
        .output_limits(-10.0, 10.0)
        .build()
        .unwrap();

    let pid = pid::FuncPidController::new(config);
    let mut ctx = pid::PidContext::<time::Millis, f64>::new_uninit();
    let setpoint = 1.0;
    let mut measurement = 0.9;
    let dt = 10;
    let mut now = time::Millis(0);
    let mut output: f64 = 0.0;

    c.bench_function("func_pid", |b| {
        b.iter(|| {
            (output, ctx) =
                pid.compute(ctx, black_box(measurement), black_box(setpoint), now, None);
            measurement += 0.0001; // prevent constant inputs
            now.0 += dt;
            black_box(output);
        });
    });
}

struct SimplePidConfig {
    kp: f64,
    ki: f64,
    kd: f64,
}

#[cfg(feature = "bench")]
fn bench_simple_pid(c: &mut Criterion) {
    let config = pid::PidConfigBuilder::default()
        .kp(1.0)
        .ki(0.5)
        .kd(0.1)
        .output_limits(-10.0, 10.0)
        .build()
        .unwrap();

    let mut pid = pid::PidController::new_uninit(config);
    let setpoint = 1.0;
    let mut measurement = 0.9;
    let dt = 10;
    let mut now = time::Millis(0);
    let mut output: f64 = 0.0;

    c.bench_function("pid", |b| {
        b.iter(|| {
            output = pid.compute(black_box(measurement), black_box(setpoint), now, None);
            measurement += 0.0001; // prevent constant inputs
            now.0 += dt;
            black_box(output);
        });
    });
}

// The true PID implementation computes the elapsed time between computations and uses it to
// update the integral and derivative terms. This is truest to the mathematical definition of PID,
// but requires a bit more computation every loop and has to handle DB0 in the derivative term.
#[cfg(feature = "bench")]
fn bench_true_pid(c: &mut Criterion) {
    let kp = 1.0;
    let ki = 0.5;
    let kd = 0.1;
    let mut err_sum: f64 = 0.0;
    let mut last_err: f64 = 0.1;

    let mut measurement = 0.9;
    let setpoint = 1.0;

    let mut now = 0.01;
    let mut last_time: f64 = 0.0;
    let cfg = SimplePidConfig { kp, ki, kd };
    let mut output: f64 = 0.0;
    c.bench_function("naive_pid", |b| {
        b.iter(|| {
            black_box(measurement);
            black_box(setpoint);
            let time_change = now - last_time;
            if time_change <= 1e-6 {
                return; // avoid division by zero
            }
            /*Compute all the working error variables*/
            let error = setpoint - measurement;
            err_sum += error * time_change;
            err_sum = err_sum.clamp(-10.0, 10.0);
            let d_err = (error - last_err) / time_change;

            /*Compute PID Output*/
            output = cfg.kp * error + cfg.ki * err_sum + cfg.kd * d_err;
            output = output.clamp(-10.0, 10.0);
            /*Remember some variables for next time*/
            last_err = error;
            last_time = now;
            black_box(output);

            now += 0.01;

            measurement += 0.0001; // prevent constant inputs
        });
    });
}

#[cfg(feature = "bench")]
criterion_group!(
    benches,
    bench_discrete_pid,
    bench_simple_pid,
    bench_true_pid,
);
#[cfg(feature = "bench")]
criterion_main!(benches);

#[cfg(not(feature = "bench"))]
pub fn main() {
    eprintln!("This benchmark requires the 'bench' feature to be enabled.");
    eprintln!("Run with `cargo bench --features bench`.");
}
