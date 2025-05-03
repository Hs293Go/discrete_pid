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

use core::{ptr, time::Duration};

use crate::pid::{PidConfig, PidConfigBuilder, PidConfigError};

use std::{boxed::Box, os::raw::c_char};

pub const PID_CONFIG_SUCCESS: i32 = 0;
pub const PID_CONFIG_ERROR_NULL_POINTER: i32 = u8::MAX as i32;

#[no_mangle]
pub extern "C" fn pid_config_error_to_string(error_code: i32) -> *const c_char {
    let message = match error_code {
        PID_CONFIG_SUCCESS => b"Success\0" as &[u8],
        x if x == PidConfigError::InvalidProportionalGain as i32 => {
            b"Proportional gain is non-positive or non-finite\0" as &[u8]
        }
        x if x == PidConfigError::InvalidIntegralGain as i32 => {
            b"Integral gain is negative or non-finite\0" as &[u8]
        }
        x if x == PidConfigError::InvalidDerivativeGain as i32 => {
            b"Derivative gain is negative or non-finite\0" as &[u8]
        }
        x if x == PidConfigError::InvalidSampleTime as i32 => {
            b"Filter time constant is non-positive or non-finite\0" as &[u8]
        }
        x if x == PidConfigError::InvalidOutputLimits as i32 => {
            b"Output limits are flipped or NAN\0" as &[u8]
        }
        x if x == PidConfigError::InvalidFilterTimeConstant as i32 => {
            b"Filter time constant is non-positive or non-finite\0" as &[u8]
        }
        PID_CONFIG_ERROR_NULL_POINTER => b"Null pointer\0" as &[u8],
        _ => b"Unknown PidConfigError\0" as &[u8],
    };

    message.as_ptr() as *const c_char
}

#[repr(C)]
pub struct PidConfigF64 {
    pub inner: PidConfig<f64>,
}

#[no_mangle]
pub extern "C" fn pid_config_create() -> *mut PidConfigF64 {
    let inner = PidConfig::default();

    Box::into_raw(Box::new(PidConfigF64 { inner }))
}

#[no_mangle]
pub extern "C" fn pid_config_create_with_gains(kp: f64, ki: f64, kd: f64) -> *mut PidConfigF64 {
    match PidConfigBuilder::default().kp(kp).ki(ki).kd(kd).build() {
        Ok(inner) => Box::into_raw(Box::new(PidConfigF64 { inner })),
        Err(_) => ptr::null_mut(),
    }
}

#[no_mangle]
pub extern "C" fn pid_config_create_with_gains_and_sample_time(
    kp: f64,
    ki: f64,
    kd: f64,
    sample_time: f64,
) -> *mut PidConfigF64 {
    match PidConfigBuilder::default()
        .kp(kp)
        .ki(ki)
        .kd(kd)
        .sample_time(Duration::from_secs_f64(sample_time))
        .build()
    {
        Ok(inner) => Box::into_raw(Box::new(PidConfigF64 { inner })),
        Err(_) => ptr::null_mut(),
    }
}

/// Gets the proportional gain (Kp) of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
///
/// # Returns
/// * `f64`: The proportional gain value.
///
/// # Safety
/// If the pointer to the config object is null, this function will return -1.0
#[no_mangle]
pub unsafe extern "C" fn pid_config_get_kp(config: *const PidConfigF64) -> f64 {
    if config.is_null() {
        return -1.0;
    }

    (*config).inner.kp()
}

/// Gets the integral gain (Ki) of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
///
/// # Returns
/// * `f64`: The integral gain value.
///
/// # Safety
/// If the pointer to the config object is null, this function will return -1.0
#[no_mangle]
pub unsafe extern "C" fn pid_config_get_ki(config: *const PidConfigF64) -> f64 {
    if config.is_null() {
        return -1.0;
    }

    (*config).inner.ki()
}

/// Sets the proportional gain (Kp) of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
/// * `kp`: The new proportional gain value.
///
/// # Returns
/// * `i32`: An error code indicating if the operation succeeded
///
/// # Safety
/// If the pointer to the config object is null, this function will return false
#[no_mangle]
pub unsafe extern "C" fn pid_config_set_kp(config: *mut PidConfigF64, kp: f64) -> i32 {
    if config.is_null() {
        return PID_CONFIG_ERROR_NULL_POINTER;
    }

    match (*config).inner.set_kp(kp) {
        Ok(_) => PID_CONFIG_SUCCESS,
        Err(ec) => ec as i32,
    }
}

/// Sets the integral gain (Ki) of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
/// * `ki`: The new integral gain value.
///
/// # Returns
/// * `i32`: An error code indicating if the operation succeeded
///
/// # Safety
/// If the pointer to the config object is null, this function will return false
#[no_mangle]
pub unsafe extern "C" fn pid_config_set_ki(config: *mut PidConfigF64, ki: f64) -> i32 {
    if config.is_null() {
        return PID_CONFIG_ERROR_NULL_POINTER;
    }

    match (*config).inner.set_ki(ki) {
        Ok(_) => PID_CONFIG_SUCCESS,
        Err(ec) => ec as i32,
    }
}

/// Sets the derivative gain (Kd) of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
/// * `kd`: The new derivative gain value.
///
/// # Returns
/// * `i32`: An error code indicating if the operation succeeded
///
/// # Safety
/// If the pointer to the config object is null, this function will return false
#[no_mangle]
pub unsafe extern "C" fn pid_config_set_kd(config: *mut PidConfigF64, kd: f64) -> i32 {
    if config.is_null() {
        return PID_CONFIG_ERROR_NULL_POINTER;
    }

    match (*config).inner.set_kd(kd) {
        Ok(_) => PID_CONFIG_SUCCESS,
        Err(ec) => ec as i32,
    }
}

/// Sets the sample time of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
/// * `sample_time`: The new sample time value in seconds.
///
/// # Returns
/// * `i32`: An error code indicating if the operation succeeded
///
/// # Safety
/// If the pointer to the config object is null, this function will return false
#[no_mangle]
pub unsafe extern "C" fn pid_config_set_sample_time(
    config: *mut PidConfigF64,
    sample_time: f64,
) -> i32 {
    if config.is_null() {
        return PID_CONFIG_ERROR_NULL_POINTER;
    }

    let sample_time = Duration::from_secs_f64(sample_time);
    match (*config).inner.set_sample_time(sample_time) {
        Ok(_) => PID_CONFIG_SUCCESS,
        Err(ec) => ec as i32,
    }
}

/// Sets the output limits of the PID controller.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 struct.
/// * `min`: The minimum output limit.
/// * `max`: The maximum output limit.
///
/// # Returns
/// * `i32`: An error code indicating if the operation succeeded
///
/// # Safety
/// If the pointer to the config object is null, this function will return false
#[no_mangle]
pub unsafe extern "C" fn pid_config_set_output_limits(
    config: *mut PidConfigF64,
    min: f64,
    max: f64,
) -> i32 {
    if config.is_null() {
        return PID_CONFIG_ERROR_NULL_POINTER;
    }

    match (*config).inner.set_output_limits(min, max) {
        Ok(_) => PID_CONFIG_SUCCESS,
        Err(ec) => ec as i32,
    }
}

/// Deallocates a PidConfigF64 object.
///
/// # Arguments
/// * `config`: A pointer to the PidConfigF64 object to deallocate.
///
/// # Safety
/// The pointer must be valid and must have been allocated by the corresponding creation function.
#[no_mangle]
pub unsafe extern "C" fn pid_config_destroy(config: *mut PidConfigF64) {
    if !config.is_null() {
        drop(Box::from_raw(config));
    }
}
