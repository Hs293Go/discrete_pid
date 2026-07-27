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

#[cfg(test)]
pub mod test_pid {

    use discrete_pid::pid::*;
    use discrete_pid::time::Millis;

    pub fn make_controller() -> (FuncPidController<f64>, PidContext<Millis, f64>) {
        let config = PidConfig::default();
        let controller = FuncPidController::new(config);
        let ctx = PidContext::new_uninit();
        (controller, ctx)
    }

    pub fn make_stateful_controller() -> PidController<Millis, f64> {
        let config = PidConfig::default();
        PidController::new_uninit(config)
    }

    /// Applies `f` to a copy of the controller's configuration, then writes it back through
    /// `set_config` so the D-term filter is re-derived. Returns whatever `f` returned, so a
    /// fallible setter can still be asserted on at the call site.
    pub fn tune<R>(
        pid: &mut FuncPidController<f64>,
        f: impl FnOnce(&mut PidConfig<f64>) -> R,
    ) -> R {
        let mut config = *pid.config();
        let result = f(&mut config);
        pid.set_config(config);
        result
    }

    /// [`tune`], for the stateful controller.
    pub fn tune_stateful<R>(
        pid: &mut PidController<Millis, f64>,
        f: impl FnOnce(&mut PidConfig<f64>) -> R,
    ) -> R {
        let mut config = *pid.config();
        let result = f(&mut config);
        pid.set_config(config);
        result
    }
}
