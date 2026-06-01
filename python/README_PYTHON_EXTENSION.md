# Phase 35 Python Extension

This folder is an optional extension after the MATLAB/Simulink stage has passed. Python is used only for visualization, data analysis, dashboard generation, and replay of MATLAB logged data.

Python does **not** replace the MATLAB/Simulink pipeline. This folder does not implement the plant, the Phase 29 TVLQR tracking controller, the Phase 30 hybrid controller, or the Simulink model.

## Folder structure

```text
python_extension/
  common/
    matlab_results.py
  visualization/
    plot_dashboard.py
    animate_from_matlab_results.py
  data_analysis/
    analyze_monte_carlo_results.py
  interface_demo/
    realtime_viewer_from_logged_data.py
  run_phase35_extension_demo.py
  README_PYTHON_EXTENSION.md
```

## Prerequisites

Install the existing Python requirements:

```bash
pip install -r requirements.txt
```

The extension uses only the packages already listed in `requirements.txt`: NumPy, SciPy, Matplotlib, and Pillow.

## Required MATLAB outputs

For the full extension demo, generate these MATLAB/Simulink results first:

```text
results/phase30_full_hybrid/full_hybrid_result.mat
results/phase31_monte_carlo/phase31_monte_carlo_metrics.csv
results/phase33_evaluation/phase33_summary_metrics.csv
```

The scripts handle missing files by reporting which prerequisite is missing.

## Run the full extension demo

From the project root:

```bash
python -m python_extension.run_phase35_extension_demo
```

Expected output:

```text
Phase 35 run_phase35_extension_demo: PASS
```

Generated files are saved to:

```text
results/phase35_python_extension/
```

## Generate the dashboard only

```bash
python -m python_extension.visualization.plot_dashboard
```

Expected output:

```text
phase35_dashboard_file = .../results/phase35_python_extension/phase35_dashboard.png
Phase 35 plot_dashboard: PASS
```

## Analyze Monte Carlo results only

```bash
python -m python_extension.data_analysis.analyze_monte_carlo_results
```

Expected output:

```text
phase35_monte_carlo_summary = .../phase35_monte_carlo_analysis.txt
phase35_monte_carlo_failure_counts = .../phase35_monte_carlo_failure_counts.csv
phase35_monte_carlo_plot = .../phase35_monte_carlo_analysis.png
Phase 35 analyze_monte_carlo_results: PASS
```

## Create animation from MATLAB result

After running Phase 30:

```bash
python -m python_extension.visualization.animate_from_matlab_results --stride 1
```

Expected output:

```text
phase35_animation_file = .../results/phase35_python_extension/phase35_animation.gif
Phase 35 animate_from_matlab_results: PASS
```

## Replay logged data in a lightweight viewer

```bash
python -m python_extension.interface_demo.realtime_viewer_from_logged_data --speed 1.0
```

This opens a Matplotlib viewer and replays the logged MATLAB state trajectory. It does not run a controller.

## Phase 35 rule

Python can read MATLAB result files and make figures, dashboards, reports, or replay viewers. Python must not become the main plant, controller, simulation, or evaluation pipeline.
