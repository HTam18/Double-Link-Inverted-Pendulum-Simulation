# Shared Data

This folder contains data shared by MATLAB and Python.

Important files:

```text
parameters.json
linearization/linearized_models.json
lqr/lqr_gains.json
```

`parameters.json` stores the physical parameters, state order, angle convention, force limits, target modes, default initial state, and external force limits.

Python reads the JSON files directly. MATLAB scripts regenerate the linearization and LQR JSON files when controller design is updated.
