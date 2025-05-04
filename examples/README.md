# Quadrotor Control using PID

In this example, the quadrotor is modelled as a rigid body subjected to thrusts
delivered by the four propellers and torques proportional to those thrusts.

## Quadrotor Modeling

The quadrotor system dynamics equation is given by

```math
\begin{bmatrix}
\dot{\mathbf{p}} \\
\dot{\mathbf{q}} \\
\dot{\mathbf{v}} \\
\dot{\boldsymbol{\omega}} \\
\dot{\boldsymbol{\varpi}}
\end{bmatrix} = \begin{bmatrix}
\mathbf{v} \\
\frac{1}{2}\mathbf{q} \otimes \mathcal{I}^\star(\boldsymbol{\omega}) \\
f\mathbf{R}\{\mathbf{q}\}\mathbf{1}_3 + g\mathbf{1}_3 \\
\mathbf{J}^{-1}(\boldsymbol{\tau}-\boldsymbol{\omega}^\times\mathbf{J}\boldsymbol{\omega}) \\
\frac{1}{\tau_c}(\boldsymbol{\varpi}_d - \boldsymbol{\varpi})
\end{bmatrix}
```

Where the collective thrust $f$ and torques $\boldsymbol{\tau}$ are converted
from individual rotor thrusts $\mathbf{f}$ using the following logic:

1. The individual rotor speeds are converted to rotor thrusts by:

   ```math
   \mathbf{f}_i = c\boldsymbol{\varpi}_i^2,\ \text{for}\ i = 1, \ldots, 4
   ```

2. The individual rotor thrusts are converted to the collective thrust and
   torque by:

   ```math
   \begin{bmatrix}
   f \\ \boldsymbol{\tau}
   \end{bmatrix} = \begin{bmatrix}
   -b_y & -f_y & b_y & f_y \\
   b_x & -f_x & b_x & -f_x \\
   c  & -c  & -c & c
   \end{bmatrix}\boldsymbol{\varpi}
   ```

   > A Betaflight motor-ordering is assumed

## Control Design

This quadrotor is controlled under a cascaded architecture:

- The outer loop controller tracks position and velocity targets and computes a
  thrust and body rate setpoint. It is a faithful implementation of Mellinger
  and Kumar's geometric controller [^1].
- The inner loop controller tracks body rate setpoints and computes the motor
  thrusts. It consists of three PID controllers from this crate, one each for
  roll/pitch/yaw rate.

[^1]:
    D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control
    for quadrotors," in _2011 IEEE international conference on robotics and
    automation_, IEEE, 2011, pp. 2520–2525
