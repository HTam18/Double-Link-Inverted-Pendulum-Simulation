# Mathematical Model Notes

## Purpose

This document explains the linearized state-space model used for the double link inverted pendulum on a cart.

The goal of the model is to support LQR controller design and simulation around the upright equilibrium.

## System description

The system contains:

```text
1 cart
2 pendulum links
1 horizontal force input
6 state variables
```

The cart moves along the horizontal axis. Link 1 is connected to the cart. Link 2 is connected to the end of link 1.

The input is the horizontal force applied to the cart:

```text
u = horizontal force applied to the cart
```

## State vector

The state vector is:

```text
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
```

Where:

```text
x          = cart position
y_dot      = not used
x_dot      = cart velocity
theta1     = link 1 angle around the upright position
theta1_dot = link 1 angular velocity
theta2     = link 2 angle around the upright position
theta2_dot = link 2 angular velocity
```

In the MATLAB files, the state order must stay exactly:

```text
1: x
2: x_dot
3: theta1
4: theta1_dot
5: theta2
6: theta2_dot
```

## Angle convention

The project uses small angles around the upright equilibrium:

```text
theta1 = 0 means link 1 is upright
theta2 = 0 means link 2 is upright
```

Positive and negative angles are measured around this upright point.

This convention must stay consistent in:

```text
scripts/parameters_double_link.m
scripts/state_space_model_double.m
scripts/run_open_loop_double.m
scripts/design_lqr_double.m
scripts/run_closed_loop_lqr.m
models/double_link_lqr.slx
```

## Generalized coordinates

The generalized coordinates are:

```text
q = [x; theta1; theta2]
```

The generalized velocities are:

```text
q_dot = [x_dot; theta1_dot; theta2_dot]
```

The linearized second-order model around the upright equilibrium is written in the form:

```text
M*q_ddot + damping*q_dot - gravity_stiffness*q = input_matrix*u
```

So:

```text
q_ddot = inv(M)*(input_matrix*u - damping*q_dot + gravity_stiffness*q)
```

The positive gravity stiffness terms are the reason the upright equilibrium is unstable in open loop.

## Mass matrix

The mass matrix used around the upright equilibrium is:

```text
M = [mc + m1 + m2,        m1*lc1 + m2*L1,              m2*lc2;
     m1*lc1 + m2*L1,     m1*lc1^2 + m2*L1^2 + I1,     m2*L1*lc2;
     m2*lc2,             m2*L1*lc2,                   m2*lc2^2 + I2]
```

## Damping matrix

The damping matrix is:

```text
damping = diag([bc, b1, b2])
```

Where:

```text
bc = cart damping
b1 = joint 1 damping
b2 = joint 2 damping
```

## Gravity stiffness matrix

The cart position has no gravity stiffness term.

The angular gravity terms are:

```text
g1 = (m1*lc1 + m2*L1)*g
g2 = m2*lc2*g
```

The gravity stiffness matrix is:

```text
gravity_stiffness = [0,  0,  0;
                     0, g1,  0;
                     0,  0, g2]
```

## Input matrix

The force is applied to the cart:

```text
input_matrix = [1;
                0;
                0]
```

## State-space form

The model is converted to:

```text
X_dot = A*X + B*u
Y     = C*X + D*u
```

The output matrix is:

```text
C = eye(6)
```

The feedthrough matrix is:

```text
D = zeros(6, 1)
```

This means all six states are available as model outputs.

## Required matrix sizes

The expected sizes are:

```text
A = 6 x 6
B = 6 x 1
C = 6 x 6
D = 6 x 1
```

## Open-loop check

The open-loop system should have at least one pole with positive real part. This confirms the model behaves like an inverted pendulum and is unstable without control.

## Controllability check

The controllability matrix is:

```matlab
Co = ctrb(A, B);
rank_Co = rank(Co);
```

The expected result is:

```text
rank_Co = 6
```

This means the linearized system is controllable and can be used for LQR design.

## Model limitation

This model is a linearized model around the upright equilibrium. It is suitable for small-angle stabilization tests. It should not be treated as a full nonlinear model for large-angle swing-up motion.
