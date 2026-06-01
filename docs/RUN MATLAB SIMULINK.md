# How to Run the MATLAB/Simulink Pipeline

## 1. Start from the project root

Open MATLAB and set the current folder to the project root, for example:

```text
<project_root>
```

Then run:

```matlab
run('matlab/startup_project.m')
```

Expected output:

```text
Double Link Pendulum MATLAB path configured.
Project root: <your_project_root>
MATLAB root:  <your_project_root>\matlab
```

## 2. Phase 29 - TVLQR tracking test

Command:

```matlab
run('matlab/startup_project.m')
run('matlab/simulation/run_tvlqr_tracking_test.m')
```

Expected final output:

```text
phase29_pass = 1
ideal_tracker_pass = 1
actuator_tracker_pass = 1
Phase 29 run_phase29_clean_test: PASS
```

Important output files:

```text
results/phase29_tvlqr_tracking/tvlqr_tracking_result.mat
results/phase29_tvlqr_tracking/phase29_clean_summary.txt
results/phase29_tvlqr_tracking/phase29_clean_tracking_compare.png
results/phase29_tvlqr_tracking/phase29_clean_error_compare.png
```

## 3. Phase 30 - Full hybrid MATLAB pipeline

Command:

```matlab
run('matlab/startup_project.m')
run('matlab/simulation/run_full_hybrid_matlab.m')
```

Expected final output:

```text
phase30_handoff_success = 1
phase30_stabilize_success = 1
phase30_failsafe_triggered = 0
phase30_pass = 1
Phase 30 run_full_hybrid_matlab: PASS
```

Accepted reference metrics:

```text
phase30_tracking_end_time_s = 12
phase30_handoff_time_s = 12
phase30_final_angle_error_rad = 7.74681355917e-05
phase30_final_velocity_norm = 0.00463941585448
phase30_max_abs_x_m = 0.508490646357
phase30_saturation_fraction = 0
```

Important output files:

```text
results/phase30_full_hybrid/full_hybrid_result.mat
results/phase30_full_hybrid/phase30_full_hybrid_summary.txt
results/phase30_full_hybrid/phase30_hybrid_modes.png
results/phase30_full_hybrid/phase30_state_force.png
```

## 4. Phase 31 - Monte Carlo robustness

Standard validation profile:

```matlab
phase31_profile = 'standard';
phase31_num_runs = 30;
run('matlab/startup_project.m')
run('matlab/simulation/run_monte_carlo_robustness.m')
```

Expected final output:

```text
phase31_num_runs = 30
phase31_profile = standard
phase31_success_rate = 0.7
phase31_handoff_success_rate = 0.9
phase31_stabilize_success_rate = 0.7
Phase 31 run_monte_carlo_robustness: PASS
```

Stress profile for limitation analysis:

```matlab
phase31_profile = 'stress';
phase31_num_runs = 30;
run('matlab/startup_project.m')
run('matlab/simulation/run_monte_carlo_robustness.m')
```

The stress profile may return `REVIEW_REQUIRED`. This is acceptable because its role is to expose failure regions.

Important output files:

```text
results/phase31_monte_carlo/phase31_monte_carlo_results.mat
results/phase31_monte_carlo/phase31_monte_carlo_metrics.csv
results/phase31_monte_carlo/phase31_monte_carlo_summary.txt
results/phase31_monte_carlo/phase31_success_by_run.png
results/phase31_monte_carlo/phase31_failure_reasons.png
```

## 5. Phase 32 - Simulink model and comparison

Generate or refresh the Simulink model:

```matlab
run('matlab/startup_project.m')
run('matlab/simulink/init_simulink_model.m')
```

Run the Simulink comparison test:

```matlab
run('matlab/simulink/run_simulink_test.m')
```

Expected final output:

```text
phase32_handoff_success = 1
phase32_stabilize_success = 1
phase32_failsafe_triggered = 0
phase32_compare_max_state_error_vs_phase30 = 4.38624858006e-13
phase32_pass = 1
Phase 32 run_simulink_test: PASS
```

Important output files:

```text
matlab/simulink/Double_Link_Pendulum_Main.slx
results/phase32_simulink/phase32_simulink_result.mat
results/phase32_simulink/phase32_simulink_summary.txt
results/phase32_simulink/phase32_state_force_compare.png
results/phase32_simulink/phase32_mode_log.png
```

## 6. Phase 33 - Evaluation and report-ready summary

If all prerequisite result files already exist, run:

```matlab
phase33_force_rerun = false;
run('matlab/startup_project.m')
run('matlab/evaluation/evaluate_all_scenarios.m')
```

To rerun Phase 29, Phase 30, Phase 31 and Phase 32 before collecting the final summary, run:

```matlab
phase33_force_rerun = true;
phase33_num_monte_carlo_runs = 30;
run('matlab/startup_project.m')
run('matlab/evaluation/evaluate_all_scenarios.m')
```

Expected final output:

```text
phase33_scenario_count = 7
phase33_required_scenarios_present = 1
phase33_phase29_T4_pass = 1
phase33_phase30_pass = 1
phase33_phase31_pass = 1
phase33_phase32_pass = 1
phase33_pass = 1
Phase 33 evaluate_all_scenarios: PASS
```

Important output files:

```text
results/phase33_evaluation/phase33_evaluation_data.mat
results/phase33_evaluation/phase33_summary_metrics.csv
results/phase33_evaluation/phase33_result_summary.txt
results/phase33_evaluation/figures/
docs/MATLAB_SIMULINK_EVALUATION_REPORT.md
```

## 7. Recommended full validation command sequence

Use this sequence when preparing the final report:

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

## 8. Troubleshooting

### Simulink model file does not exist

Run:

```matlab
run('matlab/simulink/init_simulink_model.m')
```

This regenerates:

```text
matlab/simulink/Double_Link_Pendulum_Main.slx
```

### Phase 31 returns REVIEW_REQUIRED

Check which profile was used. The `standard` profile is the official validation profile. The `stress` profile is intentionally harder and may fail.

### Phase 29 T0 exact replay fails

This is acceptable. T0 is diagnostic only. Phase 29 pass is based on T2, T3 and T4.

### Simulink `u_actual` point-by-point mismatch appears

This can happen because MATLAB and Simulink log actuator force at different sample boundaries. The accepted Phase 32 pass gate uses state trajectory equivalence, peak force equivalence, handoff, stabilize, rail, saturation and failsafe metrics.
