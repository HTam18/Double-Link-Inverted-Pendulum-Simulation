# Future Work

## Purpose

This document lists possible future extensions for the project.

The current project already has a complete LQR baseline with MATLAB simulation, Simulink simulation, actuator saturation, disturbance tests, plots, metrics, and documentation.

Future work should be added only after the current version is stable and well documented.

## 1. MATLAB animation

A simple MATLAB animation can be added to visualize the cart and two links.

Possible output:

```text
Cart moving left and right
Link 1 angle response
Link 2 angle response
Optional video export
```

This would make the project easier to present.

## 2. Simscape Multibody visualization

A Simscape Multibody model can be added for visualization.

This should not replace the current state-space and LQR model. It should be treated as a visualization extension only.

## 3. Nonlinear model comparison

The current project uses a linearized model around the upright equilibrium.

A future extension can compare the LQR controller on a nonlinear model.

This would help show the limitation of the linearized controller.

## 4. Parameter variation tests

The project can test changes in:

```text
cart mass
link 1 mass
link 2 mass
link length
joint damping
cart damping
```

This can show how sensitive the controller is to modeling errors.

## 5. Sensor noise test

Sensor noise can be added to the measured states.

This would make the simulation closer to a real system.

## 6. Observer or Kalman filter

The current controller assumes all states are available.

A future extension can add an observer or Kalman filter if not all states are measured.

This is useful for a more realistic control system.

## 7. Swing-up control

The current LQR controller stabilizes the system near the upright equilibrium.

Swing-up control would be a separate advanced extension. It should not be mixed with the first LQR baseline.

A possible structure is:

```text
Swing-up controller for large angles
Switching logic near upright
LQR stabilizer near upright
```

## 8. MPC controller

Model Predictive Control can be studied after the LQR version is complete.

MPC may handle constraints more directly, but it is more complex and should be treated as an advanced comparison.

## 9. Optional AI controller experiment

A NEAT inspired AI controller can be added only as an optional experiment.

It should not replace the LQR controller.

Possible structure:

```text
Neural controller input: full state vector
Neural controller output: cart force
Fitness score: balance time - angle error - cart error - control effort
Comparison target: selected LQR controller
```

## 10. XPBD or custom physics visualization

XPBD can be studied for real-time physics visualization or custom simulation.

It is not required for the MATLAB/Simulink LQR version.

## Recommended next step

The best next extension is:

```text
Simple MATLAB animation
```

Reason:

```text
It improves project presentation.
It does not change the controller.
It is easier than NEAT, MPC, or nonlinear swing-up.
It helps explain the result visually.
```
