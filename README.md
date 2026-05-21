# Double Link Inverted Pendulum Control Simulation

## Overview

This project simulates and controls a double link inverted pendulum on a cart using MATLAB and Simulink. The main controller is a Linear Quadratic Regulator (LQR) designed from a linearized state-space model around the upright equilibrium.

The project focuses on stabilization around the upright position. It includes open-loop testing, controllability checking, LQR design, MATLAB closed-loop simulation, Simulink closed-loop simulation with actuator saturation, disturbance testing, performance metrics, and result analysis.

```text
State vector: X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
Input:        u = horizontal force applied to the cart
Control law:  u_raw = -K*x
Actuator:     u_sat = saturation(u_raw, -u_max, +u_max)
Disturbance:  u_total = u_sat + disturbance_force
```

## Current project status

```text
Phase 0: Environment setup - Completed
Phase 1: Double link parameters - Completed
Phase 2: State-space model and open-loop test - Completed
Phase 3: LQR controller design - Completed
Phase 4: MATLAB closed-loop simulation - Completed
Phase 5: Simulink closed-loop model - Completed
Phase 6: Disturbance and performance analysis - Completed
Phase 7: Documentation and GitHub-ready cleanup - Completed
```

The technical LQR baseline is complete. The current project version is ready for GitHub documentation, report writing, and CV use.

## Selected controller

The final selected LQR controller is the C13 tuning set:

```matlab
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
```

The actuator force limit used in the Simulink model is:

```matlab
u_max = 15;    % N
```

This value keeps the actuator force limited while matching the selected C13 controller better than the earlier 10 N limit.

## Main results

### Phase 5: Simulink saturated closed-loop response

```text
Max abs theta1:          10.793403 deg
Max abs theta2:           6.827943 deg
Max abs cart position:    0.726916 m
Max abs u_raw:           16.838953 N
Max abs u_sat:           15.000000 N
Saturation used:          1
Saturation respected:     1
Final theta1:             0.002327 deg
Final theta2:             0.002560 deg
Final cart position:     -0.000397 m
```

The Simulink response matches the MATLAB closed-loop response closely. The main difference is caused by the actuator saturation at +/-15 N.

### Phase 6: Disturbance and larger initial angle tests

```text
Baseline 3 deg no disturbance:
Final state OK: 1
Max abs u_sat: 15.000000 N

Small force disturbance 2 N:
Final state OK: 1
Recovery time after disturbance: 3.358952 s

Medium force disturbance 3 N:
Final state OK: 1
Recovery time after disturbance: 3.679850 s

Larger initial angle 5 deg:
Final state OK: 1
Max abs theta1: 19.669646 deg
Max abs theta2: 12.355712 deg
Max abs cart position: 1.325372 m
```

The selected LQR controller stabilizes the system under actuator saturation and recovers from small external force disturbances.

## Project structure

```text
double_link_inverted_pendulum_control/
├── README.md
├── docs/
│   ├── project_overview.md
│   ├── mathematical_model.md
│   ├── controller_design_lqr.md
│   ├── closed_loop_simulation.md
│   ├── lqr_tuning_force_check.md
│   ├── simulink_closed_loop_model.md
│   ├── result_analysis.md
│   ├── test_plan.md
│   ├── future_work.md
│   └── references.md
├── models/
│   └── double_link_lqr.slx
├── scripts/
│   ├── check_environment.m
│   ├── parameters_double_link.m
│   ├── state_space_model_double.m
│   ├── run_open_loop_double.m
│   ├── design_lqr_double.m
│   ├── run_closed_loop_lqr.m
│   ├── tune_lqr_qr_double.m
│   ├── create_double_link_lqr_model.m
│   ├── run_simulink_lqr.m
│   ├── analyze_results_double.m
│   └── run_phase6_disturbance_tests.m
├── results/
│   ├── phase2_open_loop/
│   ├── phase3_lqr_design/
│   ├── phase4_closed_loop/
│   ├── phase45_lqr_tuning/
│   ├── phase5_simulink_closed_loop/
│   └── phase6_disturbance_analysis/
└── media/
```

## How to run

Open MATLAB and set the current folder to the project root:

```text
E:\BackupDown\PROJECT2
```

Run the project step by step:

```matlab
run('scripts/check_environment.m')
run('scripts/run_open_loop_double.m')
run('scripts/design_lqr_double.m')
run('scripts/run_closed_loop_lqr.m')
run('scripts/run_simulink_lqr.m')
run('scripts/run_phase6_disturbance_tests.m')
```

The Simulink model can be created automatically by:

```matlab
run('scripts/create_double_link_lqr_model.m')
```

## Result folders

```text
results/phase2_open_loop/
```

Stores open-loop response and instability check results.

```text
results/phase3_lqr_design/
```

Stores LQR gain, poles, controllability results, and controller design summary.

```text
results/phase4_closed_loop/
```

Stores MATLAB closed-loop simulation results.

```text
results/phase5_simulink_closed_loop/
```

Stores Simulink saturated closed-loop results and MATLAB vs Simulink comparison.

```text
results/phase6_disturbance_analysis/
```

Stores disturbance test data, metrics table, summary, and response plots.

## Main documents

```text
docs/project_overview.md
```

Explains the project goal, scope, and completed phases.

```text
docs/mathematical_model.md
```

Explains the state vector, input, linearized model, matrix sizes, and assumptions.

```text
docs/controller_design_lqr.md
```

Explains the LQR control law, selected Q/R values, gain K, and closed-loop stability.

```text
docs/simulink_closed_loop_model.md
```

Explains the Simulink block structure, saturation, disturbance input, and logged signals.

```text
docs/result_analysis.md
```

Summarizes Phase 5 and Phase 6 results using the real metrics from the simulation.

```text
docs/test_plan.md
```

Lists the tests used to verify the model and controller.

```text
docs/future_work.md
```

Lists possible future extensions such as animation, Simscape visualization, observer design, and optional AI controller experiments.

## Limitations

This project uses a linearized model around the upright equilibrium. It is suitable for small-angle stabilization tests. It does not include swing-up control, nonlinear validation, hardware implementation, observer design, Kalman filtering, MPC, reinforcement learning, or real-time deployment.

## Future work

Possible future improvements include:

```text
Simple MATLAB animation
Simscape Multibody visualization
Nonlinear model comparison
Parameter variation tests
Observer or Kalman filter design
Optional NEAT inspired AI controller comparison
Swing-up control as a separate advanced extension
```

These are future extensions only. The current LQR and Simulink stabilization version is already complete as the main project baseline.

## CV summary

Built a MATLAB/Simulink double link inverted pendulum simulation using state-space modeling and LQR control, with actuator saturation, disturbance testing, response plots, and performance metrics.
