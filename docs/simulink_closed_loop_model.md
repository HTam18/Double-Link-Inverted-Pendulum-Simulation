# Simulink Closed-loop Model Notes

## Purpose

This document explains the Simulink closed-loop model used in Phase 5 and Phase 6.

The model is:

```text
models/double_link_lqr.slx
```

It can be created automatically by:

```matlab
run('scripts/create_double_link_lqr_model.m')
```

## Main structure

The model implements:

```text
u_raw = -K*x
u_sat = saturation(u_raw, -u_max, +u_max)
u_total = u_sat + disturbance_force
X_dot = A*X + B*u_total
```

The structure is:

```text
State-Space output X
        ↓
Gain block -K
        ↓
u_raw
        ↓
Saturation block [-u_max, +u_max]
        ↓
u_sat
        ↓
Sum block with disturbance_force
        ↓
u_total
        ↓
State-Space plant input
```

## Blocks used

The model uses these main blocks:

```text
State-Space block
Gain block
Saturation block
From Workspace block for disturbance force
Sum block
Demux block
Scope block
To Workspace blocks
```

## State-Space block

The plant uses:

```text
A = A
B = B
C = C
D = D
Initial condition = x0
```

The matrix sizes are:

```text
A = 6 x 6
B = 6 x 1
C = 6 x 6
D = 6 x 1
x0 = 6 x 1
```

## Controller block

The controller uses the selected LQR gain:

```matlab
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
```

The Gain block uses:

```text
-K
```

So the output is:

```text
u_raw = -K*x
```

## Saturation block

The actuator force limit is:

```matlab
u_max = 15;    % N
```

The Saturation block uses:

```text
Upper limit = u_max
Lower limit = -u_max
```

This creates:

```text
u_sat = saturation(u_raw, -15, +15)
```

## Disturbance force

The disturbance force is added after the actuator saturation:

```text
u_total = u_sat + disturbance_force
```

This is intentional.

The actuator limit applies only to the controller output. The disturbance is an external force acting on the cart and should not be clipped by the actuator saturation block.

## Logged signals

The model logs these signals to MATLAB workspace:

```text
sim_states
sim_u_raw
sim_u_sat
sim_disturbance_force
sim_u_total
```

The state vector order is:

```text
1: x
2: x_dot
3: theta1
4: theta1_dot
5: theta2
6: theta2_dot
```

## Phase 5 usage

Phase 5 uses zero disturbance:

```text
disturbance_force = 0
```

Run:

```matlab
run('scripts/run_simulink_lqr.m')
```

Results are saved in:

```text
results/phase5_simulink_closed_loop/
```

## Phase 6 usage

Phase 6 runs several disturbance and initial condition test cases.

Run:

```matlab
run('scripts/run_phase6_disturbance_tests.m')
```

Results are saved in:

```text
results/phase6_disturbance_analysis/
```

## Phase 5 result summary

The Phase 5 saturated Simulink response gave:

```text
Max abs theta1:        10.793403 deg
Max abs theta2:         6.827943 deg
Max abs cart position:  0.726916 m
Max abs u_raw:         16.838953 N
Max abs u_sat:         15.000000 N
Saturation used:        1
Saturation respected:   1
Final theta1:           0.002327 deg
Final theta2:           0.002560 deg
Final cart position:   -0.000397 m
```

## Important checks

The Simulink model is correct if:

```text
The model runs without error.
The State-Space block uses A, B, C, D, and x0.
The Gain block implements -K.
The Saturation block limits u_sat to +/-15 N.
The disturbance force is added after saturation.
The logged states use the correct state order.
The final state returns close to zero.
```
