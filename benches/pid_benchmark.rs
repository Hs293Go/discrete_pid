//! Benchmark for the PID controllers
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

use criterion::{black_box, criterion_group, criterion_main, Criterion};

use discrete_pid::{pid, time};

fn make_config() -> pid::PidConfig<f64> {
    pid::PidConfigBuilder::default()
        .kp(1.0)
        .ki(0.5)
        .kd(0.1)
        .output_limits(-10.0, 10.0)
        .build()
        .unwrap()
}

/// The (stateless) FuncPidController is relatively slow compared to the PidController and the
/// naive PID law. But each computation still only takes time on the order of nanoseconds.
fn bench_func_pid(c: &mut Criterion) {
    let pid = pid::FuncPidController::new(make_config());
    let mut ctx = pid::PidContext::<time::Millis, f64>::new_uninit();
    let setpoint = 1.0;
    let mut measurement = 0.9;
    let dt = 10;
    let mut now = time::Millis(0);
    let mut output: f64 = 0.0;

    c.bench_function("functional PID", |b| {
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

/// The (stateful) PidController should be 3-4x faster than the FuncPidController mainly because
/// the context variables are stored inline inside the controller structure, which massively
/// benefits optimization but makes the controller mutable.
fn bench_stateful_pid(c: &mut Criterion) {
    let mut pid = pid::PidController::new_uninit(make_config());
    let setpoint = 1.0;
    let mut measurement = 0.9;
    let dt = 10;
    let mut now = time::Millis(0);
    let mut output: f64 = 0.0;

    c.bench_function("stateful PID", |b| {
        b.iter(|| {
            output = pid.compute(black_box(measurement), black_box(setpoint), now, None);
            measurement += 0.0001; // prevent constant inputs
            now.0 += dt;
            black_box(output);
        });
    });
}

// The naive PID implementation computes the elapsed time between computations and uses it to
// update the integral and derivative terms. This is truest to the mathematical definition of PID,
// but requires a bit more computation every loop and has to handle DB0 in the derivative term.
// Otherwise, it has NO sample time handling (pid-rs), NO D-term filtering (pid-rs), and NO
// activity/integral activity checks.
// Even so, this should not be > 50% faster than the PidController.
fn bench_naive_pid(c: &mut Criterion) {
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
    c.bench_function("naive PID", |b| {
        b.iter(|| {
            black_box(measurement);
            black_box(setpoint);
            let time_change = now - last_time;
            if time_change <= 1e-6 {
                return; // avoid division by zero
            }
            // Compute all the working error variables
            let error = setpoint - measurement;
            err_sum += error * time_change;

            // Clamping the integral term is the bare minimum we could do to ensure safety. Leave
            // it in the benchmark
            err_sum = err_sum.clamp(-10.0, 10.0);
            let d_err = (error - last_err) / time_change;

            // Compute PID Output
            output = cfg.kp * error + cfg.ki * err_sum + cfg.kd * d_err;
            // Ditto about lamping the output
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

criterion_group!(benches, bench_func_pid, bench_stateful_pid, bench_naive_pid,);
criterion_main!(benches);
