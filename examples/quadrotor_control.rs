//! Geometric + nested PID controller example for quadrotor stabilization.
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
mod control {
    use discrete_pid::{pid, sim, time::Millis};
    use nalgebra as na;

    #[derive(Debug)]
    pub struct QuadrotorState {
        pub position: na::Vector3<f64>,
        pub orientation: na::UnitQuaternion<f64>,
        pub velocity: na::Vector3<f64>,
        pub body_rate: na::Vector3<f64>,
        pub motor_speeds: na::Vector4<f64>,
    }

    pub struct QuadrotorSimulator {
        pub mass: f64,
        pub inertia: na::Matrix3<f64>,
        pub inertia_inv: na::Matrix3<f64>,
        pub front_motor_position: na::Vector2<f64>,
        pub back_motor_position: na::Vector2<f64>,
        pub torque_coeff: f64,
        pub motor_time_constant: f64,
        pub thrust_constant: f64,
    }

    impl QuadrotorSimulator {
        pub fn new(
            mass: f64,
            inertia: na::Matrix3<f64>,
            front_motor_position: na::Vector2<f64>,
            back_motor_position: na::Vector2<f64>,
            torque_coeff: f64,
            motor_time_constant: f64,
            thrust_constant: f64,
        ) -> Option<Self> {
            inertia.try_inverse().map(|inertia_inv| Self {
                mass,
                inertia,
                inertia_inv,
                front_motor_position,
                back_motor_position,
                torque_coeff,
                motor_time_constant,
                thrust_constant,
            })
        }
        pub fn allocation_matrix(&self) -> na::Matrix4<f64> {
            let fx = self.front_motor_position[0];
            let fy = self.front_motor_position[1];
            let bx = self.back_motor_position[0];
            let by = self.back_motor_position[1];
            let c = self.torque_coeff;

            na::matrix![
                1., 1., 1., 1.;
                -by, -fy, by, fy;
                bx, -fx, bx, -fx;
                c, -c, -c, c
            ]
        }

        pub fn update(
            &self,
            mut qs: QuadrotorState,
            motor_speeds_cmd: na::Vector4<f64>,
            dt: f64,
        ) -> QuadrotorState {
            // Implement the time-derivative of Euclidean quantities in the quadrotor dynamics
            // equation; i.e. The quaternion dynamics is not updated here
            #[allow(clippy::toplevel_ref_arg)]
            let f = |x: na::SVector<f64, 13>| -> na::SVector<f64, 13> {
                let motor_thrusts =
                    motor_speeds_cmd.component_mul(&motor_speeds_cmd) * self.thrust_constant;
                let thrust_torque = self.allocation_matrix() * motor_thrusts;

                let collective_thrust = thrust_torque[0] / self.mass;
                let torque = thrust_torque.fixed_rows::<3>(1);

                let velocity = x.fixed_rows::<3>(3);
                let body_rate = x.fixed_rows::<3>(6);
                let motor_speeds = x.fixed_rows::<4>(9);

                na::stack![velocity;
                    collective_thrust * qs.orientation.transform_vector(&na::Vector3::z()) - na::Vector3::z() * 9.81;
                    self.inertia_inv * (torque - body_rate.cross(&(self.inertia * body_rate)));
                    (motor_speeds_cmd - motor_speeds) / self.motor_time_constant
                ]
            };

            // Update the Euclidean quantities in the quadrotor dynamics equation with a RK4
            // integrator
            #[allow(clippy::toplevel_ref_arg)]
            let x0 = na::stack![qs.position; qs.velocity; qs.body_rate; qs.motor_speeds];
            let x = sim::rk4_step(f, x0, dt);
            qs.position.copy_from(&x.fixed_rows::<3>(0));
            let updated_body_rate = x.fixed_rows::<3>(6);

            // Specifically update the quaternion by converting the body rates (as angle-axis) to a
            // quaternion increment then multiplying it with the current orientation
            qs.orientation *= na::UnitQuaternion::from_scaled_axis(updated_body_rate * dt);
            qs.velocity.copy_from(&x.fixed_rows::<3>(3));
            qs.body_rate.copy_from(&updated_body_rate);
            qs.motor_speeds.copy_from(&x.fixed_rows::<4>(9));
            qs
        }
    }

    /// The geometric controller is relatively simple, so we embed the parameters directly in
    /// the struct
    pub struct GeometricController {
        pub mass: f64,
        pub k_position: na::Vector3<f64>,
        pub k_velocity: na::Vector3<f64>,
        pub k_angle: na::Vector3<f64>,
        pub k_rate: na::Vector3<f64>,
    }

    /// Geometric tracking controller. This is copied verbatim from
    /// Mellinger and Kumar's "Minimum Snap Trajectory Generation and Control for Quadrotors"
    impl GeometricController {
        /// The vee-map converts a skew-symmetric matrix to a vector
        fn vee(mat: na::Matrix3<f64>) -> na::Vector3<f64> {
            na::Vector3::new(mat[(2, 1)], mat[(0, 2)], mat[(1, 0)])
        }

        /// This controller is stateless, so we don't use a context
        pub fn compute(
            &self,
            quad_state: &QuadrotorState,
            setpoint: &QuadrotorState,
            yaw: f64,
        ) -> (f64, na::Vector3<f64>) {
            // Position control law --- essentially PD control on position
            // TODO: Readers may consider applying our PID controller here
            let e_p = quad_state.position - setpoint.position;
            let e_v = quad_state.velocity - setpoint.velocity;
            let f_des = -self.k_position.component_mul(&e_p) - self.k_velocity.component_mul(&e_v)
                + na::Vector3::z() * self.mass * 9.81;

            let collective_thrust_setpoint = f_des.norm();

            // For simplicity, we intentionally don't handle zero thrust vectors that cannot be
            // normalized --- this is very possible in agile flight.
            let zb_des = f_des.normalize();
            let xc_des = na::vector![yaw.cos(), yaw.sin(), 0.0];
            let yb_des = zb_des.cross(&xc_des).normalize();
            let xb_des = yb_des.cross(&zb_des);
            // Desired orientation for delivering the desired force vector
            let rd = na::Matrix3::from_columns(&[xb_des, yb_des, zb_des]);

            // Current orientation
            let r = quad_state.orientation.to_rotation_matrix();
            let r = r.matrix();

            // Let R_d and R be the desired and actual rotation matrices
            // angle_error = vee(0.5 * (R_dᵀ R - Rᵀ R_d))
            let angle_error = Self::vee(rd.transpose() * r - r.transpose() * rd) / 2.0;
            let rate_error = quad_state.body_rate - setpoint.body_rate;

            // TODO: Readers may consider applying our robust PID controller here
            let rate_setpoint =
                -self.k_angle.component_mul(&angle_error) - self.k_rate.component_mul(&rate_error);
            (collective_thrust_setpoint, rate_setpoint)
        }
    }

    pub struct RateControllerContext(pub [pid::PidContext<Millis, f64>; 3]);

    impl RateControllerContext {
        pub fn new() -> Self {
            RateControllerContext([
                pid::PidContext::<Millis, f64>::new_uninit(),
                pid::PidContext::<Millis, f64>::new_uninit(),
                pid::PidContext::<Millis, f64>::new_uninit(),
            ])
        }
    }

    pub struct RateController(pub [pid::FuncPidController<f64>; 3]);

    impl RateController {
        pub fn new(
            cfg_roll: pid::PidConfig<f64>,
            cfg_pitch: pid::PidConfig<f64>,
            cfg_yaw: pid::PidConfig<f64>,
        ) -> Self {
            let pid_roll = pid::FuncPidController::new(cfg_roll);
            let pid_pitch = pid::FuncPidController::new(cfg_pitch);
            let pid_yaw = pid::FuncPidController::new(cfg_yaw);
            RateController([pid_roll, pid_pitch, pid_yaw])
        }

        pub fn compute(
            &self,
            mut ctx: RateControllerContext,
            body_rate: na::Vector3<f64>,
            rate_setpoint: na::Vector3<f64>,
            timestamp: Millis,
        ) -> (na::Vector3<f64>, RateControllerContext) {
            let mut output = na::Vector3::zeros();

            for axis in 0..3 {
                (output[axis], ctx.0[axis]) = self.0[axis].compute(
                    ctx.0[axis],
                    body_rate[axis],
                    rate_setpoint[axis],
                    timestamp,
                    None,
                );
            }
            (output, ctx)
        }
    }
}

#[cfg(feature = "simulation")]
mod simulation {
    use control::*;
    use discrete_pid::{pid, time};
    use nalgebra as na;
    use std::f64;
    use std::fs::{create_dir_all, File};
    use std::time::Duration;

    use std::io::Write;
    use std::path::Path;
    pub fn make_controllers() -> (QuadrotorSimulator, GeometricController, RateController) {
        const QUAD_MASS: f64 = 1.0;

        let quad = QuadrotorSimulator::new(
            QUAD_MASS,
            na::Matrix3::from_diagonal(&na::vector![0.025, 0.025, 0.043]),
            na::vector![0.075, 0.1],
            na::vector![0.075, 0.1],
            0.01386,
            0.033,
            1.56252e-6,
        )
        .expect("Incorrect constant in example: inertia. Notify developer.");

        // TODO: Apart from adjusting the gains, readers may also consider making the quadrotor
        // mass not match the mass of the quadrotor in the simulator. This introduces steady state
        // errors that cannot be eliminated by the PD tracking law in Mellinger's controller, but
        // replacing their PD tracking law with our PID controller may help
        let geometric_controller = GeometricController {
            mass: QUAD_MASS,
            k_position: na::vector![1.0, 1.0, 4.0],
            k_velocity: na::vector![1.8, 1.8, 8.0],
            k_angle: na::vector![6.0, 6.0, 3.0],
            k_rate: na::vector![1.5, 1.5, 1.3],
        };

        let rate_controller_config = pid::PidConfigBuilder::default()
            .kp(0.2)
            .ki(0.2)
            .kd(0.01)
            .use_derivative_on_measurement(true)
            .output_limits(-20.0, 20.0)
            .build()
            .expect("Incorrect constant in example: rate controller. Notify developer.");
        let rate_controller = RateController::new(
            rate_controller_config,
            rate_controller_config,
            rate_controller_config,
        );
        (quad, geometric_controller, rate_controller)
    }

    use super::*;

    // For simplicity, we let the quadrotor navigate to sparsely spaced discrete waypoints. This
    // results in relatively jumpy trajectories --- when the quadrotor reaches a waypoint and its
    // setpoint jumps to the next waypoint, the quadrotor is liable to 'jump up' as a sudden large
    // thrust is commanded by the position control segment, but the quadrotor hasn't rotated soon
    // enough to deliver that thrust in the right direction.
    // TODO: Readers may consider implementing the minimum-snap trajectory generation algorithm
    const INITIAL_POSITION: na::Vector3<f64> = na::vector![0.0, 0.0, 1.0];
    const TARGET_POSITIONS: [na::Vector3<f64>; 4] = [
        na::vector![5.0, 0.0, 1.0],
        na::vector![5.0, 5.0, 1.0],
        na::vector![0.0, 5.0, 1.0],
        INITIAL_POSITION,
    ];
    const TARGET_HEADINGS: [f64; 4] = [
        0.0,
        f64::consts::FRAC_PI_2,
        f64::consts::PI,
        f64::consts::FRAC_PI_2 + f64::consts::PI,
    ];
    const RADIUS_OF_ACCEPTANCE: f64 = 0.1;
    const MAX_NUM_STEPS: i32 = 5000;

    pub struct SimOut {
        tout: Vec<f64>,
        position: Vec<na::Vector3<f64>>,
        rate_setpoint: Vec<na::Vector3<f64>>,
        body_rate: Vec<na::Vector3<f64>>,
    }

    pub fn run_simulation(
        quad: QuadrotorSimulator,
        geometric_controller: GeometricController,
        rate_controller: RateController,
    ) -> SimOut {
        let mut rate_control_ctx = RateControllerContext::new();
        const STEP_SIZE: f64 = 0.01; // Just use the default PID step size
        const SIM_STEPS_PER_CONTROL_STEP: i32 = 10; // Update the dynamics at 10x the control rate
        const SIM_STEP_SIZE: f64 = STEP_SIZE / SIM_STEPS_PER_CONTROL_STEP as f64;

        let mut torque_setpoint: na::Vector3<f64>;
        let mut target_idx = 0;

        let mut quad_state = QuadrotorState {
            position: INITIAL_POSITION,
            orientation: na::UnitQuaternion::identity(),
            velocity: na::Vector3::zeros(),
            body_rate: na::Vector3::zeros(),
            motor_speeds: na::Vector4::zeros(),
        };

        let mut out = SimOut {
            tout: vec![0.0],
            position: vec![INITIAL_POSITION],
            rate_setpoint: vec![na::Vector3::zeros()],
            body_rate: vec![na::Vector3::zeros()],
        };

        // TODO: Readers may run this simulation with separate threads for the quadrotor simulators
        // and inner/outer loop controllers. This is a good opportunity to demonstrate how easily
        // our functionally pure controllers can be parallelized
        for step in 0..MAX_NUM_STEPS {
            let target_setpoint = QuadrotorState {
                position: TARGET_POSITIONS[target_idx],
                orientation: na::UnitQuaternion::identity(),
                velocity: na::Vector3::zeros(),
                body_rate: na::Vector3::zeros(),
                motor_speeds: na::Vector4::zeros(),
            };

            // Outer loop: Use Geometric Controller to track the full state setpoint
            let (thrust_setpoint, rate_setpoint) = geometric_controller.compute(
                &quad_state,
                &target_setpoint,
                TARGET_HEADINGS[target_idx],
            );

            let timestamp = rate_control_ctx.0[0].last_time().unwrap_or(time::Millis(0))
                + Duration::from_secs_f64(STEP_SIZE);
            (torque_setpoint, rate_control_ctx) = rate_controller.compute(
                rate_control_ctx,
                quad_state.body_rate,
                rate_setpoint,
                timestamp,
            );

            #[allow(clippy::toplevel_ref_arg)]
            let thrust_torque = na::stack![na::vector![thrust_setpoint]; torque_setpoint];

            let motor_thrusts = quad
                .allocation_matrix()
                .lu()
                .solve(&thrust_torque)
                .expect("Incorrect constant in example: allocation matrix. Notify developer.")
                .map(|x| x.clamp(0.0, 9.0));
            let motor_speeds = motor_thrusts.map(|x| (x / quad.thrust_constant).sqrt());

            quad_state = (0..SIM_STEPS_PER_CONTROL_STEP).fold(quad_state, |quad_state, _| {
                quad.update(quad_state, motor_speeds, SIM_STEP_SIZE)
            });
            if (quad_state.position - TARGET_POSITIONS[target_idx]).norm() < RADIUS_OF_ACCEPTANCE {
                target_idx += 1;
                if target_idx >= TARGET_POSITIONS.len() {
                    println!("Reached final target position on step {step}");
                    break;
                }
            }
            out.tout.push(timestamp.0 as f64 / 1e3);
            out.position.push(quad_state.position);
            out.rate_setpoint.push(rate_setpoint);
            out.body_rate.push(quad_state.body_rate);
        }
        out
    }

    pub fn write_results(simulation_hist: SimOut) {
        let output_filename = Path::new("output/quadrotor_trajectory.csv");
        println!("Writing results to {}", output_filename.display());
        if let Some(parent) = output_filename.parent() {
            create_dir_all(parent)
                .expect("Incorrect directory structure in example. Notify developer.");
        }
        let mut file = File::create(output_filename).expect("Failed to create file");
        writeln!(file, "type,time,x,y,z,pd,qd,rd,p,q,r").expect("Failed to write header");
        TARGET_POSITIONS.iter().for_each(|vec| {
            writeln!(
                file,
                "target,{},{},{},{},NaN,NaN,NaN,NaN,NaN,NaN",
                -1.0, vec.x, vec.y, vec.z
            )
            .expect("Failed to write to file");
        });

        simulation_hist
            .tout
            .iter()
            .zip(simulation_hist.position.iter())
            .zip(simulation_hist.rate_setpoint.iter())
            .zip(simulation_hist.body_rate.iter())
            .for_each(|(((t, pos), rate_setpoint), body_rate)| {
                writeln!(
                    file,
                    "output,{},{},{},{},{},{},{},{},{},{}",
                    t,
                    pos.x,
                    pos.y,
                    pos.z,
                    rate_setpoint.x,
                    rate_setpoint.y,
                    rate_setpoint.z,
                    body_rate.x,
                    body_rate.y,
                    body_rate.z
                )
                .expect("Failed to write to file");
            });
    }
}

#[cfg(feature = "simulation")]
pub fn main() {
    let (quad, geometric_controller, rate_controller) = simulation::make_controllers();

    let simulation_hist = simulation::run_simulation(quad, geometric_controller, rate_controller);

    simulation::write_results(simulation_hist);
}

#[cfg(not(feature = "simulation"))]
fn main() {
    eprintln!("This example requires `--features simulation` to run.");
}
