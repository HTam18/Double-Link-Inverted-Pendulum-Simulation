# Phase 4: MATLAB Closed-loop Simulation

## 1. Purpose

Phase 4 tests the LQR controller from Phase 3 in a MATLAB closed-loop simulation.

The goal is to check whether the controller can bring the double link inverted pendulum back near the upright equilibrium from the small initial angles defined in `parameters_double_link.m`.

Phase 4 does not build the Simulink model yet. Simulink is reserved for the next phase after the MATLAB closed-loop simulation works.

---

## 2. Files used

Main script:

```text
scripts/run_closed_loop_lqr.m
```

Files loaded by the script:

```text
scripts/parameters_double_link.m
scripts/state_space_model_double.m
results/phase3_lqr_design/phase3_lqr_design_data.mat
```

If the Phase 3 result file is missing, the script recalculates the LQR gain using the same Q and R values from Phase 3.

---

## 3. State vector

The project keeps the same state vector:

```text
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
```

Where:

```text
x          = cart position
x_dot      = cart velocity
theta1     = link 1 angle around upright position
theta1_dot = link 1 angular velocity
theta2     = link 2 angle around upright position
theta2_dot = link 2 angular velocity
```

The target equilibrium is:

```text
X = 0
```

This means the cart is near the origin and both links are upright.

---

## 4. Control law

The LQR control law is:

```text
u = -K*x
```

The closed-loop state equation is:

```text
x_dot = (A - B*K)x
```

So the closed-loop system matrix is:

```text
A_cl = A - B*K
```

---

## 5. Simulation method

The Phase 4 script builds a closed-loop state-space system using:

```text
A_cl = A - B*K
B_cl = zeros(6, 1)
C_cl = eye(6)
D_cl = zeros(6, 1)
```

There is no external input in this first closed-loop test. The system response comes from the initial condition `x0`.

The default initial condition from `parameters_double_link.m` is:

```text
theta1_0 = 3 deg
theta2_0 = -3 deg
```

This is a small-angle stabilization test around the upright equilibrium.

---

## 6. Results saved

After running the script, Phase 4 creates:

```text
results/phase4_closed_loop/phase4_closed_loop_data.mat
results/phase4_closed_loop/closed_loop_summary.txt
results/phase4_closed_loop/closed_loop_response.png
results/phase4_closed_loop/closed_loop_states.png
```

The MAT file stores:

```text
A, B, C, D
A_cl, B_cl, C_cl, D_cl
Q, R, K
closed-loop poles
time vector
state response
control force
simple metrics
```

---

## 7. Plots generated

The main response plot shows:

```text
theta1 response in degrees
theta2 response in degrees
cart position in meters
control force in newtons
```

The second plot shows all six states together for quick checking.

---

## 8. Metrics calculated

The script calculates simple Phase 4 metrics:

```text
maximum absolute theta1 angle
maximum absolute theta2 angle
maximum absolute cart position
maximum absolute cart velocity
maximum absolute control force
final theta1 angle
final theta2 angle
final cart position
theta1 settling time
theta2 settling time
cart position settling time
```

The default settling thresholds are:

```text
theta1 threshold = +/-0.5 deg
theta2 threshold = +/-0.5 deg
cart position threshold = +/-0.02 m
```

These thresholds are simple first checks. More detailed performance metrics can be improved in a later analysis phase.

---

## 9. Completion criteria

Phase 4 is complete when:

```text
1. scripts/run_closed_loop_lqr.m exists.
2. The script runs without path errors.
3. The script loads or recalculates K successfully.
4. A_cl = A - B*K is built.
5. Closed-loop poles have negative real parts.
6. The response is simulated from x0.
7. theta1 moves back near zero.
8. theta2 moves back near zero.
9. cart position remains reasonable.
10. control force is calculated and plotted.
11. Phase 4 data and plots are saved.
```

---

## 10. Important limitation

Phase 4 does not apply force saturation yet.

The script checks whether the calculated control force exceeds `u_max`, but it does not clip the force. Saturation should be tested in a later phase after the basic closed-loop response is working.

Phase 4 also does not include external disturbance testing yet. Disturbance testing should be added after the basic closed-loop simulation is confirmed.

---

## 11. Next phase

The next phase should be Simulink closed-loop model setup or a cleanup/testing phase before Simulink.

Suggested next file:

```text
models/double_link_lqr.slx
```

The Simulink model should use the same:

```text
A, B, C, D, K, x0, dt, sim_time
```

The MATLAB closed-loop result from Phase 4 should be used as the baseline for checking the Simulink result.
