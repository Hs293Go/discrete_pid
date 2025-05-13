#![warn(missing_docs)]

//! # Discrete PID Controller
//!
//! This library provides a discrete PID (Proportional-Integral-Derivative) controller in Rust.
//!
//! If is built around three core principles: **Simulink compliance**, **best-practice
//! discrete-time compliance** (based on the [Arduino PID Library](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)),
//! and a **dual API** supporting both stateful and functional use.
//!
//! ## Features
//!
//! - Respects the best practices for PID control:
//!   - Configurable and fully validated PID gains.
//!   - Anti reset-windup: Bounded output and integral terms
//!   - Optional derivative-on-measurement to mitigate derivative kick.
//!   - Bumpless tuning and (de)activation
//!
//! - Explicit support for **discrete-time** control requirements:
//!   - Configurable sampling time: _It's a no-op if the controller is called before one sampling period elapsed_.
//!   - Support for low-pass filtering on the derivative term.
//!
//! - Numerical compliance with Simulink's PID block:
//!   - Option to use a strict causal integrator to match Simulink's behavior.
//!
//! ## Platform Support
//! This crate supports `no_std` environments by default.
//! Enable the `std` feature for richer error messages and `std::time::Instant` support.
//!
//! ## Usage
//!
//! ### PID Controller
//!
//! The PID controller is simple, easy to use with its simple API, and **fast**.
//!
//! However, like most PID controllers it embeds mutable state inside the controller: The `compute`
//! method is not pure, and its output changes as the state of the integrator and filter changes.
//! The controller **must** be `mut`.
//!
//! ```rust
//! use discrete_pid::{pid, time};
//!
//! // The PID controller is automatically initialized at the first call
//! let mut pid = pid::PidController::new_uninit(pid::PidConfig::default());
//!
//! // Freely change the PID configuration
//! assert!(pid.config_mut().set_kp(2.0).is_ok());
//!
//! let pos_feedback = 1.0;
//! let pos_setpoint = 0.0;
//!
//! // Very thin wrappers over integral timestamps are provided for you
//! let new_timestamp = time::Millis(10);
//!
//! let output = pid.compute(pos_feedback, pos_setpoint, new_timestamp, None);
//!
//! ```
//!
//! ### Functional PID Controller
//!
//! The functional PID controller holds **no** mutable state.
//!
//! In exchange, the controller lets you explicitly manage the state of the controller and the
//! `compute` method is **functionally pure**, making it exceptionally easy to test and validate,
//! or to make thread-safe. If the PID configuration is final, the controller itself can be left
//! immutable as well.
//!
//! ```rust
//! use discrete_pid::{pid, time};
//!
//! let config = pid::PidConfigBuilder::default()
//!     .kp(2.0)
//!     .ki(0.2)
//!     .build()
//!     .expect("Invalid PID config");
//! let pid = pid::FuncPidController::new(config);
//!
//! // You can pre-initialize the PID context as well
//! let timestamp = time::Millis(10);
//! let steady_state_pos_feedback = 0.5;
//! let last_output = 0.01;
//! let mut context = pid::PidContext::<time::Millis, f64>::new(
//!     timestamp,
//!     steady_state_pos_feedback,
//!     last_output,
//! );
//!
//! let pos_feedback = 1.0;
//! let pos_setpoint = 2.0;
//! let timestamp = time::Millis(11);
//! let vel_setpoint = 0.1; // Higher-order setpoint --- feedforward
//!
//! let (output, updated_context) = pid.compute(
//!     context,
//!     pos_feedback,
//!     pos_setpoint,
//!     timestamp,
//!     vel_setpoint.into(),
//! );
//! ```
//!
//! ### Plugging in your Instant type
//!
//! Timekeeping is critical to our PID controller. We abstract over time representation using a
//! minimal [`InstantLike`](time::InstantLike) trait.
//!
//! While we provide [`Millis`](time::Millis), [`Micros`](time::Micros),
//! [`SecondsF64`](time::SecondsF64) and [`StdInstant`](time::StdInstant) (with `std` feature) wrappers over common
//! time representations, you can easily opt any time type into this framework as shown below:
//!
//! ``` rust
//! use core::time::Duration;
//! use discrete_pid::pid::{PidConfig, PidController};
//! use discrete_pid::time::InstantLike;
//!
//! #[derive(Copy, Clone)]
//! struct Time {
//!     sec: i32,
//!     nsec: i32,
//! }
//!
//! impl InstantLike for Time {
//!     fn duration_since(&self, other: Self) -> Duration {
//!         let sec = self.sec - other.sec;
//!         let nsec = self.nsec - other.nsec;
//!         Duration::new(sec as u64, nsec as u32)
//!     }
//! }
//!
//! let timestamp = Time { sec: 1, nsec: 0 };
//! let mut pid = PidController::new(PidConfig::default(), timestamp, 0.0, 0.0);
//! ```
//!
//! ## License
//! This project is licensed under the [MIT License](../LICENSE).
#![no_std]

#[cfg(feature = "std")]
extern crate std;

/// The main module for the PID controller library.
pub mod pid;

/// The module containing time-related utilities to support sampling time handling
pub mod time;

#[doc(hidden)]
#[cfg(feature = "simulation")]
pub mod sim;

#[doc = include_str!("../README.md")]
#[cfg(doctest)]
pub struct ReadmeDoctests;
