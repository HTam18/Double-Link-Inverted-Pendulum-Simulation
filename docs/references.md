# References

## Purpose

This document lists references used to understand the double link inverted pendulum project, LQR control, MATLAB simulation, and Simulink modeling.

The project code and documentation should be written in the author's own structure and words. References are used for learning and verification, not for copying a full project.

## MATLAB and Simulink references

### MathWorks MATLAB documentation

Used for:

```text
MATLAB scripts
matrix operations
plotting
saving MAT files
writing CSV tables
```

### MathWorks Control System Toolbox documentation

Used for:

```text
state-space models
ctrb()
rank()
lqr()
eig()
closed-loop pole checking
```

### MathWorks Simulink documentation

Used for:

```text
State-Space block
Gain block
Saturation block
Sum block
From Workspace block
To Workspace block
Scope block
```

## Double inverted pendulum learning references

### APMonitor double inverted pendulum control example

Used for understanding:

```text
double inverted pendulum state variables
cart force input
control problem setup
input limits
response plots
```

### Arizona double inverted pendulum dynamics report

Used for understanding:

```text
why double link dynamics are more complex than single link dynamics
Lagrangian modeling idea
coupled motion between cart and links
```

### Kaggle double inverted pendulum simulation reference

Used for understanding:

```text
simulation structure
state feedback idea
response visualization
```

### WiredWhite inverted pendulum with Simulink and Arduino

Used for understanding:

```text
how to present an inverted pendulum control project
LQR balance control explanation
possible future hardware direction
```

## Optional future work references

### NEAT inspired balancing video

Used only as inspiration for an optional AI controller comparison after the LQR project is complete.

### Pendulum-NEAT GitHub reference

Used only to understand the idea of a neural controller and evolutionary training. It is not part of the required LQR baseline.

### XPBD paper

Used only as a future work idea for physics simulation or visualization. XPBD is not part of the current MATLAB/Simulink control version.

## Notes on reference use

The current project uses its own MATLAB scripts, Simulink model structure, documentation, test cases, and result analysis.

The main controller is LQR. Optional AI, XPBD, swing-up control, and hardware deployment are future work only.
