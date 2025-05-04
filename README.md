# Discrete PID Controller in Rust

[![Rust](https://img.shields.io/badge/rust-stable-blue.svg)](https://www.rust-lang.org/)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](./LICENSE)
[![CI](https://github.com/Hs293Go/discrete_pid/actions/workflows/ci.yml/badge.svg)](https://github.com/Hs293Go/discrete_pid/actions)

A correctness-first discrete PID controller for embedded and real-time
applications.

| [Step Response](./examples/step_response.rs)                                                                     | [Quadrotor Rate Control](./examples/quadrotor_control.rs)                                                                |
| ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| ![Step Response](https://raw.githubusercontent.com/Hs293Go/discrete_pid/refs/heads/main/media/step_response.png) | ![Quadrotor Control](https://raw.githubusercontent.com/Hs293Go/discrete_pid/refs/heads/main/media/quadrotor_control.png) |

## Why This PID?

- **Correct and Principled**

  - **Inspired** by Brett Beauregard's battle-tested and well documented[^1]
    [PID for Arduino](https://github.com/br3ttb/Arduino-PID-Library) and the
    authoritative reference by Åström & Hägglund [^2]
  - Verified in an extensive test suite, including numerical verification
    against Simulink’s Discrete PID Controller --- _this controller has the same
    behavior you remember from your first controls course_
  - Supports **functionally pure** (stateless) PID computation, enabling
    **perfectly reproducible** control outputs and easy thread-safety

- **Real-World Ready**

  - Explicitly designed for discrete-time control --- supports variable **sample
    time** and tunable **LPF on derivative**.
  - Supports fine-grained control over PID activity: online (de)activation,
    (re)initialization, and pausing/resuming integration

- **Lightweight and Dependency-Free**

  - Usage in `#![no_std]` mode only requires core float traits from `num_traits`

## Quick Start

```rust
use core::time::Duration;
use discrete_pid::pid::{FuncPidController, PidConfigBuilder, PidContext};
use discrete_pid::time::Micros;

let loop_time = Duration::from_micros(125);
let cfg = PidConfigBuilder::default()
    .kp(2.0)
    .ki(1.5)
    .sample_time(loop_time)
    .filter_tc(0.1)
    .build()
    .expect("Failed to build a PID configuration");

let controller = FuncPidController::new(cfg);
let mut ctx = PidContext::<Micros, f32>::new_uninit();

let pos_meas = 0.5;
let pos_setpoint = 1.0;
let timestamp = Micros(125);
let vel_setpoint = 2.0;

let (output, new_ctx) =
    controller.compute(ctx, pos_meas, pos_setpoint, timestamp, vel_setpoint.into());
```

## Examples

One of the most notable achievements of PID controllers in recent years has been
the stabilization and control of quadrotors. We demonstrate just that using our
PID controller in an example.

![Quadrotor 3D trajectory](https://raw.githubusercontent.com/Hs293Go/discrete_pid/refs/heads/main/media/quadrotor_animation.gif)

Run the example:

```sh
cargo run --example quadrotor_control --features simulation
```

Visualize the results:

```sh
python3 examples/plot_quadrotor_trajectory.py         # Generates static plot
python3 examples/plot_quadrotor_trajectory.py --animate show  # Live 3D animation
python3 examples/plot_quadrotor_trajectory.py --animate save  # Save to GIF
```

## Shoutouts

- [`pid-rs`](https://crates.io/crates/pid): well-known and effective; You may
  consider using their crate if you:

  1. Are working with simple, slow-acting processes that don't benefit from
     sample time handing and D-term filtering

  2. Need to use integer process values/setpoints

  3. Need to apply clamping on P, I, and D terms separately

> [!WARNING]
>
> We doubt the usefulness of per-term clamping since it breaks linear
> superposition of the terms, generating subtle nonlinearities and potentially
> disrupting tuning techniques

- [`pidgeon`](https://github.com/security-union/pidgeon/tree/main): impressively
  designed; You may consider using their crate if you:

  1. Need support for concurrent PID control and don't mind depending on `std`
     through `std::sync::Mutex`

## License

This project is licensed under the [MIT License](./LICENSE) © 2025 Hs293Go

[^1]:
    Beauregard, B. _Improving the Beginner's PID_.
    [http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)

[^2]:
    Åström, K. J. "PID Controllers: Theory, Design, and Tuning". _The
    international society of measurement and control_, 1995.
