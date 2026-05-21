# Test Plan

## Purpose

This document lists the tests used to verify the double link inverted pendulum control project.

The tests check:

```text
Model size and consistency
Open-loop instability
Controllability
LQR closed-loop stability
MATLAB closed-loop response
Simulink saturated closed-loop response
Disturbance recovery
Larger initial angle behavior
```

## Test 1: Environment check

### Script

```matlab
run('scripts/check_environment.m')
```

### Purpose

Check that MATLAB, Simulink, and Control System Toolbox are available.

### Expected result

```text
MATLAB runs successfully.
Simulink is available.
lqr() is found.
Project folders exist.
```

## Test 2: Open-loop state-space model

### Script

```matlab
run('scripts/run_open_loop_double.m')
```

### Purpose

Check the linearized state-space model before controller design.

### Expected result

```text
A = 6 x 6
B = 6 x 1
C = 6 x 6
D = 6 x 1
Open-loop system is unstable.
Controllability rank = 6.
```

### Output folder

```text
results/phase2_open_loop/
```

## Test 3: LQR controller design

### Script

```matlab
run('scripts/design_lqr_double.m')
```

### Purpose

Design the LQR controller and check closed-loop poles.

### Expected result

```text
K is calculated.
All closed-loop poles have negative real parts.
Closed-loop stable = 1.
```

### Output folder

```text
results/phase3_lqr_design/
```

## Test 4: MATLAB closed-loop simulation

### Script

```matlab
run('scripts/run_closed_loop_lqr.m')
```

### Purpose

Simulate the closed-loop linear system in MATLAB.

### Expected result

```text
theta1 returns close to 0.
theta2 returns close to 0.
cart position returns close to 0.
control force is calculated as u = -K*x.
```

### Output folder

```text
results/phase4_closed_loop/
```

## Test 5: Simulink saturated closed-loop model

### Script

```matlab
run('scripts/run_simulink_lqr.m')
```

### Purpose

Run the Simulink model with actuator saturation and zero external disturbance.

### Expected result

```text
Simulink model runs without error.
u_sat stays within +/-15 N.
theta1 and theta2 return close to 0.
Simulink response matches MATLAB response closely.
```

### Output folder

```text
results/phase5_simulink_closed_loop/
```

## Test 6: Baseline disturbance analysis case

### Script

```matlab
run('scripts/run_phase6_disturbance_tests.m')
```

### Case

```text
baseline_3deg_no_disturbance
Initial theta1 = 3 deg
Initial theta2 = -3 deg
Disturbance = 0 N
```

### Expected result

```text
Final state OK = 1
Saturation limit respected = 1
```

## Test 7: Small force disturbance

### Case

```text
small_force_disturbance_2N
Initial theta1 = 3 deg
Initial theta2 = -3 deg
Disturbance = 2 N from 2.0 s to 2.2 s
```

### Expected result

```text
The system recovers after disturbance.
Final state OK = 1.
u_sat stays within +/-15 N.
Recovery time is calculated.
```

### Actual result

```text
Recovery time = 3.358952 s
Final state OK = 1
```

## Test 8: Medium force disturbance

### Case

```text
medium_force_disturbance_3N
Initial theta1 = 3 deg
Initial theta2 = -3 deg
Disturbance = 3 N from 2.0 s to 2.2 s
```

### Expected result

```text
The system recovers after disturbance.
Recovery time should be larger than the 2 N case.
Final state OK = 1.
```

### Actual result

```text
Recovery time = 3.679850 s
Final state OK = 1
```

## Test 9: Larger initial angle

### Case

```text
larger_initial_angle_5deg
Initial theta1 = 5 deg
Initial theta2 = -5 deg
Disturbance = 0 N
```

### Expected result

```text
The system stabilizes.
The response is larger than the 3 deg baseline.
Final state OK = 1.
```

### Actual result

```text
Max abs theta1 = 19.669646 deg
Max abs theta2 = 12.355712 deg
Max abs cart position = 1.325372 m
Final state OK = 1
```

## Completion criteria

The project test plan is complete when:

```text
All scripts run without MATLAB errors.
The open-loop model is unstable.
The controllability rank is 6.
The LQR closed-loop poles are stable.
The MATLAB closed-loop simulation stabilizes the system.
The Simulink model respects the +/-15 N saturation limit.
The disturbance tests recover to the upright equilibrium.
The results are saved as plots, summary files, MAT data, and CSV metrics.
```
