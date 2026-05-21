# Phase 4.5 - LQR Q/R Tuning and Force Check

## 1. Purpose

Phase 4 confirmed that the closed-loop MATLAB simulation is stable without force saturation. However, the first LQR controller required more force than the parameter limit `u_max`.

Phase 4.5 is added before the Simulink phase to compare several Q/R choices and check the control force demand.

The purpose of this phase is not to build the Simulink model yet. The purpose is to choose a better LQR candidate or clearly document the force limitation before moving to Simulink.

## 2. Files used

Main script:

```text
scripts/tune_lqr_qr_double.m
```

Input files:

```text
scripts/parameters_double_link.m
scripts/state_space_model_double.m
results/phase3_lqr_design/phase3_lqr_design_data.mat
```

Output folder:

```text
results/phase45_lqr_tuning/
```

## 3. State vector

```text
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
```

## 4. Control law

The LQR controller uses:

```text
u = -K*x
```

The closed-loop matrix without saturation is:

```text
A_cl = A - B*K
```

## 5. Why Q/R tuning is needed

The Phase 3 baseline controller stabilized the system, but in Phase 4 the maximum force demand was higher than `u_max`.

This means the controller is mathematically stable, but it may be too aggressive for a practical force-limited system.

To reduce force demand, this phase tries:

```text
- Increasing R
- Reducing theta1 and theta2 weights in Q
- Comparing settling time, angle deviation, cart movement, and force demand
```

Increasing `R` usually reduces control effort, but it can also make the response slower and increase cart movement.

## 6. Q/R candidates

The script tests five candidates:

```text
C1_baseline_phase3
Qdiag = [5, 1, 80, 5, 80, 5]
R = 0.1

C2_medium_force
Qdiag = [5, 1, 60, 4, 60, 4]
R = 0.5

C3_high_R
Qdiag = [3, 1, 50, 3, 50, 3]
R = 1.0

C4_conservative
Qdiag = [2, 1, 40, 2, 40, 2]
R = 2.0

C5_very_conservative
Qdiag = [1, 0.5, 30, 2, 30, 2]
R = 5.0
```
Selected controller for Phase 5:

Candidate: C13_cart_weight_8_R20
Q = diag([8, 1, 40, 2, 40, 2])
R = 2.0

Reason:
C13 is selected because it gives a better balance than C5. C5 has lower force, but the cart displacement is too large. C13 has acceptable force demand, smaller cart displacement, faster cart settling, and very small final angle error under saturation.
## 7. What the script checks

For every candidate, the script checks:

```text
- Closed-loop poles
- Stability of A - B*K
- Maximum control force without saturation
- Force ratio compared with u_max
- Maximum theta1 deviation
- Maximum theta2 deviation
- Maximum cart position
- Theta1 settling time
- Theta2 settling time
- Cart settling time
- Final state with force saturation
```

The script also runs a saturated simulation using:

```text
u_sat = max(min(u, u_max), -u_max)
```

This is not a full nonlinear model. It is still based on the linearized state-space model, but with limited control force.

## 8. Output files

After running the script, these files are created:

```text
results/phase45_lqr_tuning/phase45_lqr_tuning_data.mat
results/phase45_lqr_tuning/phase45_lqr_tuning_summary.csv
results/phase45_lqr_tuning/phase45_lqr_tuning_summary.txt
results/phase45_lqr_tuning/phase45_qr_tuning_comparison.png
results/phase45_lqr_tuning/phase45_recommended_response.png
```

## 9. Completion criteria

Phase 4.5 is complete when:

```text
1. tune_lqr_qr_double.m runs without error.
2. All Q/R candidates are tested.
3. The summary table is printed in MATLAB.
4. The result files are saved in results/phase45_lqr_tuning.
5. A recommended candidate is selected.
6. The project clearly records whether the recommended candidate stays within u_max or still needs saturation.
```

## 10. Important interpretation

If no candidate can keep the force demand below `u_max`, this does not mean the project failed.

It means one of these decisions is needed:

```text
- Accept the baseline or recommended LQR for the ideal linear simulation
- Add a Saturation block in Simulink
- Reduce the initial angle test
- Increase the actuator force limit
- Continue tuning Q/R
```

For this project, the best next step is to document the force limitation and include a Saturation block in the Simulink phase.

## 11. Next phase

After Phase 4.5, the next phase is:

```text
Phase 5 - Simulink closed-loop model
```

The Simulink model should use:

```text
- State-Space block
- Gain block using K
- Sum block for u = -K*x
- Saturation block using +/- u_max
- Scope or To Workspace blocks for x, theta1, theta2, and u
```
