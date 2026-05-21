# Result Analysis

## Purpose

This document summarizes the main results from the MATLAB and Simulink simulations.

The goal is to show that the selected LQR controller can stabilize the double link inverted pendulum around the upright equilibrium, even with actuator saturation and small external disturbances.

## Selected controller

The selected controller is the C13 tuning set:

```matlab
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
u_max = 15;    % N
```

The control law is:

```text
u_raw = -K*x
u_sat = saturation(u_raw, -u_max, +u_max)
```

For disturbance tests:

```text
u_total = u_sat + disturbance_force
```

## Phase 5: Simulink saturated closed-loop result

Phase 5 tested the Simulink model with zero external disturbance.

Result:

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

### Discussion

The unconstrained LQR force reached about 16.84 N, but the actuator saturation limited the real controller force to 15 N.

Even with this saturation, both pendulum links returned close to the upright position and the cart returned close to zero.

The Simulink response also matched the MATLAB Phase 4 response closely. The small difference is expected because Phase 4 did not apply actuator saturation, while Phase 5 did.

## Phase 6: Disturbance and larger initial angle tests

Phase 6 tested four cases.

## Case 1: Baseline 3 deg no disturbance

```text
Initial theta1:  3 deg
Initial theta2: -3 deg
Disturbance:     0 N
```

Result:

```text
Max abs theta1:        10.793403 deg
Max abs theta2:         6.827943 deg
Max abs cart position:  0.726916 m
Max abs u_sat:         15.000000 N
RMS u_sat:              5.611003 N
Saturation used:        1
Saturation respected:   1
Final state OK:         1
```

This case confirms that the Phase 6 model still behaves correctly when no disturbance is applied.

## Case 2: Small force disturbance 2 N

```text
Initial theta1:       3 deg
Initial theta2:      -3 deg
Disturbance force:    2 N
Disturbance time:     2.0 s to 2.2 s
```

Result:

```text
Max abs theta1:        10.793403 deg
Max abs theta2:         6.827943 deg
Max abs cart position:  0.726916 m
Max abs u_sat:         15.000000 N
RMS u_sat:              5.292896 N
Saturation used:        1
Saturation respected:   1
Final state OK:         1
Recovery time:          3.358952 s
```

The controller recovered after the disturbance. The final state returned close to the upright equilibrium.

## Case 3: Medium force disturbance 3 N

```text
Initial theta1:       3 deg
Initial theta2:      -3 deg
Disturbance force:    3 N
Disturbance time:     2.0 s to 2.2 s
```

Result:

```text
Max abs theta1:        10.793403 deg
Max abs theta2:         6.827943 deg
Max abs cart position:  0.726916 m
Max abs u_sat:         15.000000 N
RMS u_sat:              5.348297 N
Saturation used:        1
Saturation respected:   1
Final state OK:         1
Recovery time:          3.679850 s
```

The recovery time increased compared with the 2 N disturbance case. This is expected because the disturbance was larger.

## Case 4: Larger initial angle 5 deg

```text
Initial theta1:  5 deg
Initial theta2: -5 deg
Disturbance:     0 N
```

Result:

```text
Max abs theta1:        19.669646 deg
Max abs theta2:        12.355712 deg
Max abs cart position:  1.325372 m
Max abs u_sat:         15.000000 N
RMS u_sat:              6.356783 N
Saturation used:        1
Saturation respected:   1
Final state OK:         1
```

This case creates a stronger response than the 3 deg baseline. The cart moves farther and the link angles reach larger maximum values. However, the system still stabilizes and the final state is acceptable.

## Overall comparison

The disturbance test results show:

```text
The actuator force stayed within +/-15 N in all cases.
The controller recovered from both 2 N and 3 N force disturbances.
The recovery time increased when the disturbance magnitude increased.
The controller also stabilized the system from the larger +/-5 deg initial angle case.
```

## Final conclusion

The selected C13 LQR controller successfully stabilizes the double link inverted pendulum in the Simulink model.

The controller works with actuator saturation at +/-15 N and can reject small external force disturbances around the upright equilibrium.

The project now has a complete LQR baseline with simulation results, disturbance tests, plots, metrics, and documentation.

## Limitations

The model is linearized around the upright equilibrium. The results are valid mainly for small-angle stabilization.

This project does not include:

```text
Swing-up control
Large-angle nonlinear validation
Hardware testing
Sensor noise
Observer design
Kalman filter
MPC
AI controller comparison
```

These can be considered future work.
