# Technical Report

The project uses MATLAB as the backend for nonlinear dynamics, controller execution, target switching, recovery evaluation and metrics export. Python is the frontend GUI for replay, plots and case inspection.

The recovery backend is exposed through `runRecoveryMatrixCore`, with centralized configuration in `defaultRecoveryConfig` and output path management in `recoveryOutputPaths`. The validated recovery implementation is split into internal MATLAB modules so the orchestration file is no longer monolithic.

The final architecture keeps one clear frontend: the Python GUI.
