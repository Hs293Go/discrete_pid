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

use core::any::Any;
use core::fmt::Debug;

/// A trait for time-like objects that can be used to measure elapsed time.
/// The PID controller uses this trait to measure elapsed time and compare it to the sample time,
/// triggering computation only if the elapsed time is greater than the sample time.
pub trait InstantLike:
    Sized
    + Add<Duration, Output = Self>
    + Clone
    + Copy
    + Debug
    + PartialEq<Self>
    + Send
    + Sync
    + Unpin
    + Any
{
    /// Returns the amount of time elapsed from another instant to this one
    #[must_use]
    fn duration_since(&self, earlier: Self) -> Duration;
}

/// A wrapper around an unsigned 64-bit integer representing milliseconds
/// You would wrap th
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

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct TimeF64(pub f64); // seconds since an arbitrary epoch

impl InstantLike for TimeF64 {
    fn duration_since(&self, earlier: Self) -> Duration {
        let secs = self.0 - earlier.0;
        if secs < 0.0 {
            Duration::from_secs(0) // saturate
        } else {
            Duration::from_secs_f64(secs)
        }
    }
}

impl Add<Duration> for TimeF64 {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self::Output {
        TimeF64(self.0 + rhs.as_secs_f64())
    }
}

impl TimeF64 {
    /// Constructs a new TimeF64 from raw seconds.
    pub fn from_secs(secs: f64) -> Self {
        TimeF64(secs)
    }

    /// Returns the underlying seconds.
    pub fn as_secs_f64(&self) -> f64 {
        self.0
    }
}

/// A convenient wrapper around `std::time::Instant` satisfying the `InstantLike` trait.
#[cfg(feature = "std")]
mod std_instant {

    use super::{Add, Duration, InstantLike};

    #[derive(Debug, Clone, Copy)]
    pub struct StdInstant(pub std::time::Instant);

    impl StdInstant {
        pub fn now() -> Self {
            StdInstant(std::time::Instant::now())
        }
    }

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

    #[cfg(feature = "std")]
    impl PartialEq for StdInstant {
        fn eq(&self, other: &Self) -> bool {
            self.0 == other.0
        }
    }

    /// Tests that StdInstant is just one constructor call away from std::time::Instant
    /// and calling duration_since is equivalent to calling the same method on the underlying Instant.
    #[cfg(all(test, feature = "std"))]
    #[test]
    fn test_std_instant_wrapper() {
        let start = StdInstant::now();
        let end = StdInstant(std::time::Instant::now());
        let result = end.duration_since(start);
        let expected = end.0.duration_since(start.0);
        assert_eq!(result, expected);
    }
}

#[cfg(feature = "std")]
pub use std_instant::StdInstant;
