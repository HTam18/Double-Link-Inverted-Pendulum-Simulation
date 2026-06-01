# Python Extension Notes - Phase 35

## Purpose

Phase 35 adds optional Python tools after the MATLAB/Simulink stage has been completed. The goal is to improve presentation, visualization, and analysis of existing result files.

The extension reads MATLAB/Simulink outputs from `results/` and creates additional figures, dashboards, summaries, and a replay viewer. It does not replace any MATLAB/Simulink logic.

## Boundary of responsibility

Python extension is allowed to:

- Read `.mat`, `.csv`, `.txt`, and image files produced by MATLAB/Simulink.
- Plot report dashboards from Phase 29, Phase 30, Phase 31, Phase 32, and Phase 33 results.
- Analyze Monte Carlo result tables from Phase 31.
- Create a simple GIF animation from logged MATLAB state trajectories.
- Replay logged data in a lightweight Matplotlib viewer.

Python extension is not allowed to:

- Define a new plant model.
- Define a new controller.
- Replace Phase 29 discrete TVLQR tracking.
- Replace Phase 30 hybrid controller logic.
- Replace Phase 31 MATLAB Monte Carlo testing.
- Replace Phase 32 Simulink model.
- Become the main evaluation pipeline.

## Main files

```text
python_extension/common/matlab_results.py
python_extension/visualization/plot_dashboard.py
python_extension/visualization/animate_from_matlab_results.py
python_extension/data_analysis/analyze_monte_carlo_results.py
python_extension/interface_demo/realtime_viewer_from_logged_data.py
python_extension/run_phase35_extension_demo.py
python_extension/README_PYTHON_EXTENSION.md
```

## Recommended run order

First run the MATLAB/Simulink phases:

```matlab
run('matlab/startup_project.m')
run('matlab/simulation/run_tvlqr_tracking_test.m')
run('matlab/simulation/run_full_hybrid_matlab.m')
phase31_profile = 'standard';
phase31_num_runs = 30;
run('matlab/simulation/run_monte_carlo_robustness.m')
run('matlab/simulink/init_simulink_model.m')
run('matlab/simulink/run_simulink_test.m')
phase33_force_rerun = false;
run('matlab/evaluation/evaluate_all_scenarios.m')
```

Then run Python extension:

```bash
python -m python_extension.run_phase35_extension_demo
```

## Expected Python output

```text
Phase 35 run_phase35_extension_demo: PASS
```

Generated outputs:

```text
results/phase35_python_extension/phase35_dashboard.png
results/phase35_python_extension/phase35_monte_carlo_analysis.txt
results/phase35_python_extension/phase35_monte_carlo_failure_counts.csv
results/phase35_python_extension/phase35_monte_carlo_analysis.png
results/phase35_python_extension/phase35_extension_demo_summary.txt
```

If Phase 30 result exists, the animation script can also generate:

```text
results/phase35_python_extension/phase35_animation.gif
```

## Known limitations

- The extension depends on MATLAB result files being present.
- Packaged ZIP files may not include all generated results, so scripts may ask the user to run MATLAB phases first.
- `.mat` field names can differ if future MATLAB scripts change logger structure. The reader is intentionally flexible, but new log formats may require small updates.
- The GIF animation is for visual explanation only and is not a physically separate validation.
- The replay viewer needs an interactive Matplotlib backend.

## Final statement

Phase 35 keeps MATLAB/Simulink as the official technical pipeline and adds Python only as an optional visualization and analysis layer.
