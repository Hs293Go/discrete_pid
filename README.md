# Discrete PID Controller in Rust

[![Rust](https://img.shields.io/badge/rust-stable-blue.svg)](https://www.rust-lang.org/)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](./LICENSE)
[![CI](https://github.com/Hs293Go/discrete_pid/actions/workflows/ci.yml/badge.svg)](https://github.com/Hs293Go/discrete_pid/actions)
[![codecov](https://codecov.io/gh/Hs293Go/discrete_pid/refs/head/main/graph/badge.svg)](https://codecov.io/gh/Hs293Go/discrete_pid)
[![crates.io](https://img.shields.io/crates/v/discrete_pid.svg)](https://crates.io/crates/discrete_pid)
[![docs.rs](https://docs.rs/discrete_pid/badge.svg)](https://docs.rs/discrete_pid)

A PID controller for robotics and discrete control systems in rust.

| [Step Response](./examples/step_response.rs)                                                                     | [Quadrotor Rate Control](./examples/quadrotor_control.rs)                                                                |
| ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| ![Step Response](https://raw.githubusercontent.com/Hs293Go/discrete_pid/refs/heads/main/media/step_response.png) | ![Quadrotor Control](https://raw.githubusercontent.com/Hs293Go/discrete_pid/refs/heads/main/media/quadrotor_control.png) |

## Why This PID?

- **Principled and Reliable**

  - **Inspired** by Brett Beauregard's battle-tested and well documented[^1]
    [PID for Arduino](https://github.com/br3ttb/Arduino-PID-Library)
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
  - As an alternative to the functional PID controller, the _stateful_ PID
    controller is competitive with the naive PID law in terms of speed

- **Lightweight and Dependency-Free**

  - Usage in `#![no_std]` mode only requires core float traits from `num_traits`

## Application Example

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

In this example, we used our PID controller to track quadrotor body rates
computed by an upstream _geometric tracking controller_. The
[README for the examples](examples/README.md) contains more details about the
cascaded controller and the quadrotor simulation.

## Quick Start

```rust
use core::time::Duration;
use discrete_pid::{pid, time};

let loop_time = Duration::from_micros(125);
let cfg = pid::PidConfigBuilder::default()
    .kp(2.0)
    .ki(1.5)
    .sample_time(loop_time)
    .filter_tc(0.1)
    .build()
    .expect("Failed to build a PID configuration");

let controller = pid::FuncPidController::new(cfg);
let mut ctx = pid::PidContext::new_uninit();

let pos_meas = 0.5;
let pos_setpoint = 1.0;
let timestamp = time::Micros(125);
let vel_setpoint = 2.0;

let (output, new_ctx) =
    controller.compute(ctx, pos_meas, pos_setpoint, timestamp, vel_setpoint.into());
```

## Shoutouts

- [`pid-rs`](https://crates.io/crates/pid): well-known and effective; You may
  consider using their crate instead if you:

  1. Are working with simple, slow-acting processes that don't benefit from
     sample time handling and D-term filtering

  2. Need to use integer process values/setpoints

  3. Need to apply clamping on P, I, and D terms separately

> [!WARNING]
>
> We doubt the usefulness of per-term clamping since it breaks linear
> superposition of the terms, generating subtle nonlinearities and potentially
> disrupting tuning techniques

- [`pidgeon`](https://github.com/security-union/pidgeon/tree/main): Strong
  support for synchronization and built-in visualization; You may consider using
  their crate instead if you:

  1. Need support for concurrent PID control and don't mind depending on `std`
     through `std::sync::Mutex`

## Documentation and Links

📦 [View on crates.io](https://crates.io/crates/discrete_pid)  
📚 [API Docs on docs.rs](https://docs.rs/discrete_pid)

## License

This project is licensed under the [MIT License](./LICENSE) © 2025 Hs293Go

[^1]:
    Beauregard, B. _Improving the Beginner's PID_.
    [http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
