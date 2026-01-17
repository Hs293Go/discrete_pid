// Defines a trait for time-like objects and provides several implementations
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

use core::ops::Add;
use core::time::Duration;

/// A trait for representing time instants in PID controllers.
///
/// `InstantLike` enables portable and efficient timekeeping, abstracting over standard and embedded-friendly
/// time sources. The PID controller uses it to determine whether the sampling interval has elapsed.
///
/// This trait is designed to be minimal and supports `no_std`. The crate provides several built-in
/// implementations which are just thin wrappers around primitive numeric types
///
/// - [`Millis`] – milliseconds as an integer
/// - [`Micros`] – microseconds as an integer
/// - [`SecondsF64`] – floating-point seconds
///
/// If the `std` feature is enabled, this crate also provides:
///
/// - [`StdInstant`] – wrapper around `std::time::Instant`
///
/// ## Example
///
/// ### Implementating your own `InstantLike` type
///
/// You can define your own `InstantLike` types to interoperate with the PID controller:
///
/// ```rust
/// use discrete_pid::time::InstantLike;
/// use core::time::Duration;
///
/// #[derive(Copy, Clone)]
/// struct Time {
///   sec: i32,
///   nsec: i32,
/// }
///
/// impl InstantLike for Time {
///   fn duration_since(&self, other: Self) -> Duration {
///     let sec = self.sec - other.sec;
///     let nsec = self.nsec - other.nsec;
///     Duration::new(sec as u64, nsec as u32)
///   }
/// }
/// ```
///
/// ### Using a predefined `InstantLike` type
///
/// This example demonstrates how a conforming type can be seamlessly used with the PID controller
/// by letting the compiler infer the type of the `InstantLike` parameter.
///
/// ```rust
/// use core::time::Duration;
/// use discrete_pid::time::{InstantLike, Millis};
/// use discrete_pid::pid::{PidConfig, PidController};
///
/// let t0 = Millis(1000);
/// let t1 = Millis(1100);
///
/// assert_eq!(t1.duration_since(t0), Duration::from_millis(100));
///
/// let mut pid = PidController::new_uninit(PidConfig::default());
/// let output = pid.compute(0.0, 1.0, t1, None);
///
/// ```
///
/// ## See Also
///
/// - [`Duration`](https://doc.rust-lang.org/core/time/struct.Duration.html)
pub trait InstantLike: Copy {
    /// Returns the amount of time elapsed from another instant to this one
    #[must_use]
    fn duration_since(&self, earlier: Self) -> Duration;

    /// Serialize to a primitive form (e.g. micros since boot)
    #[cfg(feature = "serde")]
    fn to_u64(&self) -> u64;

    /// Deserialize from primitive (inverse of `to_u64`)
    #[cfg(feature = "serde")]
    fn from_u64(micros: u64) -> Self;
}

/// A wrapper around an unsigned 64-bit integer representing milliseconds
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Millis(pub u64);

impl InstantLike for Millis {
    fn duration_since(&self, earlier: Self) -> Duration {
        Duration::from_millis(self.0 - earlier.0)
    }
}

impl Add<Duration> for Millis {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        Millis(self.0 + rhs.as_millis() as u64)
    }
}

/// A wrapper around an unsigned 64-bit integer representing microseconds
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Micros(pub u64);

impl InstantLike for Micros {
    fn duration_since(&self, earlier: Self) -> Duration {
        Duration::from_micros(self.0 - earlier.0)
    }
}

impl Add<Duration> for Micros {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        Micros(self.0 + rhs.as_micros() as u64)
    }
}

/// A wrapper around an 64-bit float representing seconds
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct SecondsF64(pub f64); // seconds since an arbitrary epoch

impl InstantLike for SecondsF64 {
    fn duration_since(&self, earlier: Self) -> Duration {
        let secs = self.0 - earlier.0;
        if secs < 0.0 {
            Duration::from_secs(0) // saturate
        } else {
            Duration::from_secs_f64(secs)
        }
    }
}

impl Add<Duration> for SecondsF64 {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        SecondsF64(self.0 + rhs.as_secs_f64())
    }
}

#[test]
fn test_millis() {
    let t0 = Millis(1000);
    let t1 = t0 + Duration::from_millis(100);
    assert_eq!(t1.duration_since(t0), Duration::from_millis(100));
}

#[test]
fn test_micros() {
    let t0 = Micros(1000);
    let t1 = t0 + Duration::from_micros(100);
    assert_eq!(t1.duration_since(t0), Duration::from_micros(100));
}

#[test]
fn test_seconds_f64() {
    let t0 = SecondsF64(1.0);
    let t1 = t0 + Duration::from_secs_f64(1.0);
    assert_eq!(t1.duration_since(t0), Duration::from_secs_f64(1.0));

    let t2 = SecondsF64(0.5);
    assert_eq!(t2.duration_since(t1), Duration::from_secs_f64(0.0));
}

#[cfg(feature = "std")]
mod std_instant {

    use super::{Add, Duration, InstantLike};

    /// A wrapper around `std::time::Instant`
    #[derive(Debug, Clone, Copy)]
    pub struct StdInstant(pub std::time::Instant);

    #[cfg(feature = "std")]
    impl InstantLike for StdInstant {
        fn duration_since(&self, other: Self) -> Duration {
            self.0.duration_since(other.0)
        }
    }

    #[cfg(feature = "std")]
    impl Add<Duration> for StdInstant {
        type Output = Self;

        fn add(self, rhs: Duration) -> Self::Output {
            StdInstant(self.0 + rhs)
        }
    }

    /// Tests that StdInstant is just one constructor call away from std::time::Instant
    /// and calling duration_since is equivalent to calling the same method on the underlying Instant.
    #[cfg(all(test, feature = "std"))]
    #[test]
    fn test_std_instant_wrapper() {
        let start = std::time::Instant::now();
        let end = std::time::Instant::now();

        let wrapped_start = StdInstant(start);
        let wrapped_end = StdInstant(end);
        let result = wrapped_end.duration_since(wrapped_start);
        let expected = end.duration_since(start);
        assert_eq!(result, expected);

        let summed_end = wrapped_start + expected;
        let result = summed_end.duration_since(wrapped_start);
        assert_eq!(result, expected);
    }
}

#[cfg(feature = "std")]
pub use std_instant::StdInstant;
