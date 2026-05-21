# LQR Controller Design Notes

## Purpose

This document explains the LQR controller used for the double link inverted pendulum project.

The controller is designed for the linearized state-space model around the upright equilibrium.

## State-space model

The model has the form:

```text
X_dot = A*X + B*u
Y     = C*X + D*u
```

The state vector is:

```text
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
```

The input is:

```text
u = horizontal force applied to the cart
```

## Control law

The LQR control law is:

```text
u_raw = -K*x
```

Where:

```text
K = LQR feedback gain
x = current state vector
```

The closed-loop system matrix is:

```text
A_cl = A - B*K
```

The controller is accepted when all closed-loop poles have negative real parts.

## Controllability check

Before using LQR, the model must be controllable:

```matlab
Co = ctrb(A, B);
rank_Co = rank(Co);
```

The expected result is:

```text
rank_Co = 6
```

## Selected controller

The selected controller is the C13 tuning set:

```matlab
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
```

The actuator force limit used later in Simulink is:

```matlab
u_max = 15;    % N
```

## Meaning of Q

The Q matrix weights the state errors:

```text
x          -> 8
x_dot      -> 1
theta1     -> 40
theta1_dot -> 2
theta2     -> 40
theta2_dot -> 2
```

The angle states use higher weights because the main goal is to keep both links close to the upright position.

The cart position also has a moderate weight so the cart does not move too far while stabilizing the pendulum.

## Meaning of R

The R value weights the control effort:

```text
R = 2.0
```

A smaller R allows stronger control. A larger R makes the controller use less force but may respond more slowly.

The selected value gives a good balance between link stabilization, cart displacement, and control effort.

## Phase 5 force result

With the selected C13 controller:

```text
Max abs u_raw: 16.838953 N
Max abs u_sat: 15.000000 N
```

This means the unconstrained LQR force tries to exceed 15 N slightly, but the Simulink actuator saturation limits the actual control force to +/-15 N.

## Closed-loop stability

The project checks:

```matlab
closed_loop_poles = eig(A - B*K);
```

The controller is stable when all poles have negative real parts.

The Phase 5 script reported:

```text
Closed-loop stable without saturation: 1
```

## Files

The controller is designed in:

```text
scripts/design_lqr_double.m
```

The selected controller is also used in:

```text
scripts/run_closed_loop_lqr.m
scripts/run_simulink_lqr.m
scripts/run_phase6_disturbance_tests.m
```

## Result files

LQR design results are saved in:

```text
results/phase3_lqr_design/
```

Closed-loop MATLAB results are saved in:

```text
results/phase4_closed_loop/
```

Simulink saturated closed-loop results are saved in:

```text
results/phase5_simulink_closed_loop/
```

Disturbance analysis results are saved in:

```text
results/phase6_disturbance_analysis/
```
