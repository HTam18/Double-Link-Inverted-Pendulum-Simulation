# User Guide

Run MATLAB backend from the project root.

```matlab
run('matlab/startup_project.m')
run('scripts/runRecoveryMatrix.m')
run('scripts/runRobustnessSuite.m')
run('scripts/buildReleasePackage.m')
```

Run the Python GUI after MATLAB results exist.

```bash
python python/gui/run_gui.py
```

MATLAB computes and exports results. Python only loads, replays and visualizes those exported results.
