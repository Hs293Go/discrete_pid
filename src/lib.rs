#![warn(missing_docs)]

//! # Discrete PID Controller Library
//!
//! This library provides a discrete PID (Proportional-Integral-Derivative) controller in Rust.
//!
//! It includes both functional and stateful implementations, allowing users to choose the approach that best suits their needs.
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
//! ## Usage
//!
//! ### Functional PID Controller
//!
//! The functional PID controller lets you explicitly manage the state of the controller.
//!
//! In exchange, the controller holds no mutable state and the `compute` method is **functionally
//! pure**, making it exceptionally easy to test and validate, or to make thread-safe. If the PID
//! configuration is final, the controller itself can be non-`mut` as well.
//!
//! ```rust
//! use std::time::Instant;
//!
//! use discrete_pid::pid::{FuncPidController, PidConfig, PidConfigBuilder, PidContext};
//! use discrete_pid::time::StdInstant;
//!
//! let config = PidConfigBuilder::default()
//!     .kp(2.0)
//!     .ki(0.2)
//!     .build()
//!     .expect("Invalid PID config");
//! let pid = FuncPidController::new(config);
//! let mut context = PidContext::<StdInstant, f64>::new_uninit();
//!
//! // pid.config_mut().set_kp(2.0); // Can't do this
//! // You can make `pid` mutable to tune gains on-the-fly. The `compute` method remains pure
//!
//! let pos_feedback = 1.0;
//! let pos_setpoint = 2.0;
//! let timestamp = Instant::now();
//! let vel_setpoint = 0.1; // Higher-order setpoint --- feedforward
//!
//! let (output, updated_context) = pid.compute(
//!     context,
//!     pos_feedback,
//!     pos_setpoint,
//!     StdInstant(timestamp),
//!     vel_setpoint.into(),
//! );
//! ```
//!
//! ### Stateful PID Controller
//!
//! The stateful PID controller manages a `PidContext` internally.
//!
//! Using the stateful PID controller saves some boilerplate at the cost of embedding mutable state
//! inside the controller: The `compute` method is not pure, and its output changes as the state of
//! the integrator and filter changes. The controller **must** be `mut`.
//!
//! ```rust
//! use std::time::Instant;
//!
//! use discrete_pid::pid::{PidConfig, PidController};
//! use discrete_pid::time::StdInstant;
//!
//! let timestamp = Instant::now();
//! let pos_feedback = 1.0;
//! let pos_setpoint = 0.0;
//! let output = 0.0;
//!
//! // See `PidContext::new` for semantics of explicit initialization of PID controller
//! let mut pid = PidController::new(
//!     PidConfig::default(),
//!     StdInstant(timestamp),
//!     pos_feedback,
//!     output,
//! );
//!
//! // Freely change the PID configuration, but take good care keeping track of your changes
//! assert!(pid.config_mut().set_kp(2.0).is_ok());
//!
//! let setpoint = 2.0;
//! let new_timestamp = Instant::now();
//!
//! let output = pid.compute(pos_feedback, pos_setpoint, StdInstant(new_timestamp), None);
//!
//! ```
//!
//! ### Plugging in your Instant type
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
//! let mut pid = PidController::new_uninit(PidConfig::default());
//!
//! let timestamp = Time { sec: 1, nsec: 0 };
//! let output = pid.compute(0.0, 1.0, timestamp, None);
//!
//! ```
//!
//! ## License
//!
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
