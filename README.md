# Double Link Inverted Pendulum Control Simulation

This project simulates and controls a **double link inverted pendulum on a cart** using **MATLAB** and **Simulink**.

The main controller is a **Linear Quadratic Regulator (LQR)** designed from a linearized state-space model around the upright equilibrium. The project focuses on stabilization around the upright position and includes open-loop testing, controllability checking, LQR design, MATLAB closed-loop simulation, Simulink closed-loop simulation with actuator saturation, disturbance testing, performance metrics, and result analysis.

---

## System Summary

| Item | Description |
|---|---|
| System | Double link inverted pendulum on a cart |
| Main controller | Linear Quadratic Regulator (LQR) |
| Model type | Linearized state-space model |
| Equilibrium | Upright position |
| Simulation tools | MATLAB and Simulink |
| Main focus | Small-angle stabilization and disturbance rejection |

---

## State, Input, and Control Law

```text
State vector:
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]

Input:
u = horizontal force applied to the cart

Control law:
u_raw = -K*x

Actuator saturation:
u_sat = saturation(u_raw, -u_max, +u_max)

External disturbance:
u_total = u_sat + disturbance_force
```

---

## Main Results

### Simulink Saturated Closed-loop Response

| Metric | Value |
|---|---:|
| Max abs theta1 | 10.793403 deg |
| Max abs theta2 | 6.827943 deg |
| Max abs cart position | 0.726916 m |
| Max abs u_raw | 16.838953 N |
| Max abs u_sat | 15.000000 N |
| Saturation used | 1 |
| Saturation respected | 1 |
| Final theta1 | 0.002327 deg |
| Final theta2 | 0.002560 deg |
| Final cart position | -0.000397 m |

The main difference between the MATLAB closed-loop response and the Simulink response is caused by the actuator saturation at **±15 N**.

---

## Disturbance and Larger Initial Angle Tests

### Baseline Test

| Test case | Result |
|---|---:|
| Baseline 3 deg, no disturbance | Final state OK = 1 |
| Max abs u_sat | 15.000000 N |

### Force Disturbance Tests

| Test case | Final state OK | Recovery time |
|---|---:|---:|
| Small force disturbance, 2 N | 1 | 3.358952 s |
| Medium force disturbance, 3 N | 1 | 3.679850 s |

### Larger Initial Angle Test

| Metric | Value |
|---|---:|
| Initial angle case | 5 deg |
| Final state OK | 1 |
| Max abs theta1 | 19.669646 deg |
| Max abs theta2 | 12.355712 deg |
| Max abs cart position | 1.325372 m |

---

## How to Run

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

The Simulink model can be created automatically with:

```matlab
run('scripts/create_double_link_lqr_model.m')
```

---

## Limitations

This project uses a linearized model around the upright equilibrium. It is suitable for small-angle stabilization tests.

The current version does not include:

- Swing-up control
- Nonlinear validation
- Hardware implementation
- Observer design
- Kalman filtering
- Model Predictive Control
- Reinforcement learning
- Real-time deployment

---

## Future Work

Possible future improvements include:

- Simple MATLAB animation
- Simscape Multibody visualization
- Nonlinear model comparison
- Parameter variation tests
- Observer or Kalman filter design
- Optional NEAT inspired AI controller comparison
- Swing-up control as a separate advanced extension
