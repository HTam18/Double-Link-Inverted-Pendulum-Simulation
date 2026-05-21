# Double Link Inverted Pendulum Control Simulation

## Overview
This project simulates and controls a double link inverted pendulum on a cart using MATLAB and Simulink. The main controller is a Linear Quadratic Regulator (LQR) designed from a linearized state-space model around the upright equilibrium.
The project focuses on stabilization around the upright position. It includes open-loop testing, controllability checking, LQR design, MATLAB closed-loop simulation, Simulink closed-loop simulation with actuator saturation, disturbance testing, performance metrics, and result analysis.

State vector: X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
Input:        u = horizontal force applied to the cart
Control law:  u_raw = -K*x
Actuator:     u_sat = saturation(u_raw, -u_max, +u_max)
Disturbance:  u_total = u_sat + disturbance_force

## Main results
### Simulink saturated closed-loop response
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

The main difference is caused by the actuator saturation at +/-15 N.

### Disturbance and larger initial angle tests
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

## Project structure
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

## How to run
Open MATLAB and set the current folder to the project root: E:\BackupDown\PROJECT2

Run the project step by step:
run('scripts/check_environment.m')
run('scripts/run_open_loop_double.m')
run('scripts/design_lqr_double.m')
run('scripts/run_closed_loop_lqr.m')
run('scripts/run_simulink_lqr.m')
run('scripts/run_phase6_disturbance_tests.m')

The Simulink model can be created automatically by: run('scripts/create_double_link_lqr_model.m')

## Limitations
This project uses a linearized model around the upright equilibrium. It is suitable for small-angle stabilization tests. It does not include swing-up control, nonlinear validation, hardware implementation, observer design, Kalman filtering, MPC, reinforcement learning, or real-time deployment.

## Future work
Simple MATLAB animation
Simscape Multibody visualization
Nonlinear model comparison
Parameter variation tests
Observer or Kalman filter design
Optional NEAT inspired AI controller comparison
Swing-up control as a separate advanced extension
