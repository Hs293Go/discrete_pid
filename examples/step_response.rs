//! Example of step response of a mass-spring-damper system under PID control
//! This example requires the `--features simulation` flag to be enabled.
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
pub fn main() {
    use nalgebra as na;

    use std::time::Duration;

    let rec = rerun::RecordingStreamBuilder::new("step_response")
        .spawn()
        .unwrap();
    rec.log_static(
        "setpoint",
        &rerun::SeriesLines::new()
            .with_colors([[0, 255, 0]])
            .with_widths([2.0]),
    )
    .unwrap();
    rec.log_static(
        "output",
        &rerun::SeriesLines::new()
            .with_colors([[255, 0, 0]])
            .with_widths([2.0]),
    )
    .unwrap();

    use discrete_pid::{
        pid::{FuncPidController, PidConfigBuilder, PidContext},
        time::Millis,
    };

    const FIXED_STEP_SIZE_MS: u64 = 10;

    use discrete_pid::sim;
    use discrete_pid::sim::SignalGenerator;

    let cfg = PidConfigBuilder::default()
        .kp(10.0)
        .ki(25.0)
        .kd(10.0)
        .filter_tc(0.01)
        .build()
        .unwrap();
    let pid = FuncPidController::new(cfg);

    let mut ctx = PidContext::new(Millis(0), 0.0, 0.0);

    let mut state = na::Vector2::<f64>::zeros();
    let mut control: f64;
    let mut output: f64 = 0.0;

    let mdl = sim::MassSpringDamper {
        natural_frequency: 0.5 * std::f64::consts::PI,
        damping_ratio: 0.2,
    };

    const FIXED_STEP_SIZE_S: f64 = FIXED_STEP_SIZE_MS as f64 * 0.001;

    let mut timestamps = vec![];

    let square = SignalGenerator::new(sim::WaveForm::Square, Millis(0), 0.5, 0.5);

    for _ in 0..1000usize {
        let last_time = ctx.last_time().unwrap_or(Millis(0));
        let timestamp = last_time + Duration::from_millis(FIXED_STEP_SIZE_MS);

        let setpoint = square.generate(timestamp);
        (control, ctx) = pid.compute(ctx, output, setpoint, timestamp, None);
        state = sim::rk4_step(|x| mdl.f(x, control), state, FIXED_STEP_SIZE_S);
        output = mdl.h(state);

        timestamps.push(timestamp.0 as f64 / 1000.0);
        rec.set_time("sim_time", Duration::from_millis(timestamp.0));
        rec.log("setpoint", &rerun::Scalars::single(setpoint))
            .unwrap();
        rec.log("output", &rerun::Scalars::single(output)).unwrap();
    }
}

#[cfg(not(feature = "simulation"))]
fn main() {
    eprintln!("This example requires `--features simulation` to run.");
}
