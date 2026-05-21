# Project Overview

## Project name

Double Link Inverted Pendulum Control Simulation using MATLAB and Simulink.

## Purpose

The purpose of this project is to build a clear control systems simulation project for a double link inverted pendulum on a cart.

The project shows these skills:

```text
Dynamic system modeling
State-space representation
Open-loop stability checking
Controllability checking
LQR controller design
MATLAB simulation
Simulink closed-loop modeling
Actuator saturation
Disturbance rejection testing
Performance metrics and result analysis
```

## System description

The system has a cart and two pendulum links. The cart moves along a horizontal track. Link 1 is connected to the cart. Link 2 is connected to the end of link 1.

The goal is to keep both links close to the upright position while also keeping the cart motion reasonable.

The input is the horizontal force applied to the cart:

```text
u = cart force
```

The state vector is:

```text
X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
```

## Main controller

The main controller is LQR.

The control law is:

```text
u_raw = -K*x
```

The Simulink model also includes actuator saturation:

```text
u_sat = saturation(u_raw, -u_max, +u_max)
```

For disturbance tests, the plant input is:

```text
u_total = u_sat + disturbance_force
```

## Selected controller

The final selected controller uses:

```matlab
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
u_max = 15;    % N
```

This controller is used for the MATLAB closed-loop simulation, the Simulink closed-loop model, and the disturbance tests.

## Completed phases

### Phase 0: Environment setup

MATLAB, Simulink, and Control System Toolbox were checked.

### Phase 1: Parameters

The cart, link, damping, gravity, initial condition, simulation time, and force limit were defined.

### Phase 2: State-space model

The linearized state-space model was created and checked. The open-loop system was unstable, which is expected for an inverted pendulum. The controllability rank was 6.

### Phase 3: LQR design

The LQR gain K was calculated. The closed-loop poles were checked and the selected controller was stable.

### Phase 4: MATLAB closed-loop simulation

The closed-loop system was simulated in MATLAB using:

```text
A_cl = A - B*K
u = -K*x
```

Both links returned close to the upright position.

### Phase 5: Simulink closed-loop model

A Simulink model was created with State-Space, Gain, Saturation, Scope, and To Workspace blocks. The saturated Simulink response matched the MATLAB response closely.

### Phase 6: Disturbance and analysis

The controller was tested with small force disturbances and a larger initial angle. The system recovered and the actuator force stayed within +/-15 N.

### Phase 7: Documentation and cleanup

The README and documentation files were cleaned up so the project is easier to understand, run, and present on GitHub.

## Current scope

This project focuses on small-angle stabilization around the upright equilibrium.

The project does not include:

```text
Swing-up control
MPC
Reinforcement learning
NEAT controller as the main controller
XPBD simulation
Hardware implementation
Arduino deployment
Real-time control
```

These topics can be added later as optional future work.

## Final project outcome

The final project has a working MATLAB/Simulink LQR baseline for the double link inverted pendulum. It includes actuator saturation, disturbance testing, plots, metrics, and documentation.
