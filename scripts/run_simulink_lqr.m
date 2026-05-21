clear; clc; close all;

%% run_simulink_lqr
% Phase 5 main script, updated to stay compatible with the Phase 6 model.
% The Phase 6 model includes a disturbance input, but this script uses zero
% disturbance so it still behaves like the normal Phase 5 closed-loop test.
%
% Required model:
% models/double_link_lqr.slx
%
% If the model does not exist, this script calls:
% scripts/create_double_link_lqr_model.m

%% Make paths robust
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase5_simulink_closed_loop');
addpath(script_dir);

if ~exist(models_folder, 'dir')
    mkdir(models_folder);
end
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

%% Load parameters
run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase5_simulink_closed_loop');
addpath(script_dir);

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

%% Build state-space model and selected LQR controller
[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g); %#ok<NASGU>

Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
closed_loop_poles = eig(A - B*K);
closed_loop_stable = all(real(closed_loop_poles) < 0);

if ~closed_loop_stable
    error('The selected LQR controller is not stable. Stop before running Simulink.');
end

if u_max <= 0
    error('u_max must be positive.');
end

fprintf('\n===== Phase 5: Simulink Closed-loop Model =====\n');
fprintf('Selected controller: C13_cart_weight_8_R20\n');
fprintf('Q = %s\n', mat2str(Q));
fprintf('R = %.4f\n', R);
fprintf('u_max = %.4f N\n', u_max);
fprintf('Closed-loop stable without saturation: %d\n', closed_loop_stable);

%% Create Simulink model if missing
model_name = 'double_link_lqr';
model_file = fullfile(models_folder, [model_name, '.slx']);

if ~exist(model_file, 'file')
    fprintf('\nSimulink model was not found. Creating it now...\n');
    run(fullfile(script_dir, 'create_double_link_lqr_model.m'));

    % Rebuild workspace variables after model creation.
    script_dir = fileparts(mfilename('fullpath'));
    if isempty(script_dir)
        script_dir = pwd;
    end
    project_root = fileparts(script_dir);
    models_folder = fullfile(project_root, 'models');
    results_folder = fullfile(project_root, 'results', 'phase5_simulink_closed_loop');
    addpath(script_dir);

    run(fullfile(script_dir, 'parameters_double_link.m'));
    script_dir = fileparts(mfilename('fullpath'));
    if isempty(script_dir)
        script_dir = pwd;
    end
    project_root = fileparts(script_dir);
    models_folder = fullfile(project_root, 'models');
    results_folder = fullfile(project_root, 'results', 'phase5_simulink_closed_loop');
    addpath(script_dir);

    [A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g); %#ok<NASGU>
    Q = diag([8, 1, 40, 2, 40, 2]);
    R = 2.0;
    K = lqr(A, B, Q, R);
    closed_loop_poles = eig(A - B*K);
    closed_loop_stable = all(real(closed_loop_poles) < 0);
end

%% Zero disturbance for the normal Phase 5 run
% The Phase 6-ready model expects disturbance_force_ts from workspace.
disturbance_force_ts = timeseries([0; 0], [0; sim_time]);

%% Run Simulink
load_system(model_file);
simOut = sim(model_name, 'StopTime', num2str(sim_time), 'ReturnWorkspaceOutputs', 'on');

%% Read logged signals
sim_x = simOut.get('sim_x');
sim_x_dot = simOut.get('sim_x_dot');
sim_theta1 = simOut.get('sim_theta1');
sim_theta1_dot = simOut.get('sim_theta1_dot');
sim_theta2 = simOut.get('sim_theta2');
sim_theta2_dot = simOut.get('sim_theta2_dot');
sim_u_raw = simOut.get('sim_u_raw');
sim_u_sat = simOut.get('sim_u_sat');

[t_sim, x_sim] = local_read_workspace_signal(sim_x);
[~, x_dot_sim] = local_read_workspace_signal(sim_x_dot);
[~, theta1_sim] = local_read_workspace_signal(sim_theta1);
[~, theta1_dot_sim] = local_read_workspace_signal(sim_theta1_dot);
[~, theta2_sim] = local_read_workspace_signal(sim_theta2);
[~, theta2_dot_sim] = local_read_workspace_signal(sim_theta2_dot);
[~, u_raw_sim] = local_read_workspace_signal(sim_u_raw);
[~, u_sat_sim] = local_read_workspace_signal(sim_u_sat);

x_state_sim = [x_sim(:), x_dot_sim(:), theta1_sim(:), theta1_dot_sim(:), theta2_sim(:), theta2_dot_sim(:)];

theta1_sim_deg = rad2deg(theta1_sim);
theta2_sim_deg = rad2deg(theta2_sim);

%% Calculate Phase 5 metrics
options = struct();
options.case_name = 'phase5_zero_disturbance';
options.u_max = u_max;
options.angle_settle_threshold_deg = 0.5;
options.cart_settle_threshold_m = 0.02;
options.final_angle_threshold_deg = 0.1;
options.final_cart_threshold_m = 0.01;
options.disturbance_start_s = NaN;
options.disturbance_end_s = NaN;

metrics = analyze_results_double(t_sim, x_state_sim, u_raw_sim, u_sat_sim, zeros(size(u_sat_sim)), options);
metrics.closed_loop_stable_without_saturation = closed_loop_stable;

fprintf('\n===== Simulink Saturated Response Metrics =====\n');
fprintf('Max abs theta1: %.6f deg\n', metrics.max_abs_theta1_deg);
fprintf('Max abs theta2: %.6f deg\n', metrics.max_abs_theta2_deg);
fprintf('Max abs cart position: %.6f m\n', metrics.max_abs_cart_position_m);
fprintf('Max abs u_raw: %.6f N\n', metrics.max_abs_u_raw_N);
fprintf('Max abs u_sat: %.6f N\n', metrics.max_abs_u_sat_N);
fprintf('Saturation used: %d\n', metrics.saturation_used);
fprintf('Saturation limit respected: %d\n', metrics.saturation_limit_respected);
fprintf('Final theta1: %.9f deg\n', metrics.final_theta1_deg);
fprintf('Final theta2: %.9f deg\n', metrics.final_theta2_deg);
fprintf('Final cart position: %.9f m\n', metrics.final_cart_position_m);

%% Load Phase 4 MATLAB result for comparison if available
phase4_file = fullfile(project_root, 'results', 'phase4_closed_loop', 'phase4_closed_loop_data.mat');
has_phase4_result = exist(phase4_file, 'file') == 2;
phase4 = struct();
comparison = struct();

if has_phase4_result
    phase4 = load(phase4_file, 't_out', 'theta1_deg', 'theta2_deg', 'cart_position', 'u_out');

    theta1_sim_interp = interp1(t_sim, theta1_sim_deg, phase4.t_out, 'linear', 'extrap');
    theta2_sim_interp = interp1(t_sim, theta2_sim_deg, phase4.t_out, 'linear', 'extrap');
    x_sim_interp = interp1(t_sim, x_sim, phase4.t_out, 'linear', 'extrap');
    u_sat_sim_interp = interp1(t_sim, u_sat_sim, phase4.t_out, 'linear', 'extrap');

    comparison.max_abs_theta1_diff_deg = max(abs(theta1_sim_interp - phase4.theta1_deg));
    comparison.max_abs_theta2_diff_deg = max(abs(theta2_sim_interp - phase4.theta2_deg));
    comparison.max_abs_cart_diff_m = max(abs(x_sim_interp - phase4.cart_position));
    comparison.max_abs_control_diff_N = max(abs(u_sat_sim_interp - phase4.u_out));

    fprintf('\n===== MATLAB Phase 4 vs Simulink Phase 5 Comparison =====\n');
    fprintf('Max abs theta1 difference: %.6f deg\n', comparison.max_abs_theta1_diff_deg);
    fprintf('Max abs theta2 difference: %.6f deg\n', comparison.max_abs_theta2_diff_deg);
    fprintf('Max abs cart difference: %.6f m\n', comparison.max_abs_cart_diff_m);
    fprintf('Max abs control difference: %.6f N\n', comparison.max_abs_control_diff_N);
else
    warning('Phase 4 result file was not found. Simulink result will be saved without MATLAB comparison.');
end

%% Plot Simulink response
figure('Name', 'Phase 5 Simulink Closed-loop Response');

subplot(4, 1, 1);
plot(t_sim, theta1_sim_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta1 (deg)');
title('Simulink Link 1 Angle');

subplot(4, 1, 2);
plot(t_sim, theta2_sim_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta2 (deg)');
title('Simulink Link 2 Angle');

subplot(4, 1, 3);
plot(t_sim, x_sim, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Simulink Cart Position');

subplot(4, 1, 4);
plot(t_sim, u_raw_sim, 'LineWidth', 1.2);
hold on;
plot(t_sim, u_sat_sim, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('u (N)');
title('Control Force');
legend('u raw', 'u saturated', 'Location', 'best');

saveas(gcf, fullfile(results_folder, 'phase5_simulink_response.png'));

%% Plot MATLAB vs Simulink comparison
if has_phase4_result
    figure('Name', 'Phase 4 MATLAB vs Phase 5 Simulink');

    subplot(4, 1, 1);
    plot(phase4.t_out, phase4.theta1_deg, 'LineWidth', 1.2);
    hold on;
    plot(t_sim, theta1_sim_deg, '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('theta1 (deg)');
    title('Theta1 Comparison');
    legend('MATLAB Phase 4', 'Simulink Phase 5', 'Location', 'best');

    subplot(4, 1, 2);
    plot(phase4.t_out, phase4.theta2_deg, 'LineWidth', 1.2);
    hold on;
    plot(t_sim, theta2_sim_deg, '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('theta2 (deg)');
    title('Theta2 Comparison');
    legend('MATLAB Phase 4', 'Simulink Phase 5', 'Location', 'best');

    subplot(4, 1, 3);
    plot(phase4.t_out, phase4.cart_position, 'LineWidth', 1.2);
    hold on;
    plot(t_sim, x_sim, '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('x (m)');
    title('Cart Position Comparison');
    legend('MATLAB Phase 4', 'Simulink Phase 5', 'Location', 'best');

    subplot(4, 1, 4);
    plot(phase4.t_out, phase4.u_out, 'LineWidth', 1.2);
    hold on;
    plot(t_sim, u_sat_sim, '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('u (N)');
    title('Control Force Comparison');
    legend('MATLAB Phase 4 raw u', 'Simulink saturated u', 'Location', 'best');

    saveas(gcf, fullfile(results_folder, 'phase5_matlab_vs_simulink.png'));
end

%% Save data and summary
save(fullfile(results_folder, 'phase5_simulink_data.mat'), ...
    't_sim', 'x_state_sim', 'u_raw_sim', 'u_sat_sim', 'theta1_sim_deg', 'theta2_sim_deg', ...
    'metrics', 'comparison', 'Q', 'R', 'K', 'u_max', 'closed_loop_poles', 'closed_loop_stable');

summary_file = fullfile(results_folder, 'phase5_simulink_summary.txt');
fid = fopen(summary_file, 'w');
if fid == -1
    error('Cannot write summary file.');
end

fprintf(fid, 'Phase 5 Simulink Closed-loop Summary\n');
fprintf(fid, '=====================================\n\n');
fprintf(fid, 'Selected controller: C13_cart_weight_8_R20\n');
fprintf(fid, 'Q = %s\n', mat2str(Q));
fprintf(fid, 'R = %.6f\n', R);
fprintf(fid, 'K = %s\n', mat2str(K, 8));
fprintf(fid, 'u_max = %.6f N\n', u_max);
fprintf(fid, 'Closed-loop stable without saturation: %d\n\n', closed_loop_stable);
fprintf(fid, 'This Phase 5 run uses zero external disturbance.\n');
fprintf(fid, 'The model is Phase 6 ready and supports disturbance_force input.\n\n');
fprintf(fid, 'Max abs theta1: %.6f deg\n', metrics.max_abs_theta1_deg);
fprintf(fid, 'Max abs theta2: %.6f deg\n', metrics.max_abs_theta2_deg);
fprintf(fid, 'Max abs cart position: %.6f m\n', metrics.max_abs_cart_position_m);
fprintf(fid, 'Max abs u_raw: %.6f N\n', metrics.max_abs_u_raw_N);
fprintf(fid, 'Max abs u_sat: %.6f N\n', metrics.max_abs_u_sat_N);
fprintf(fid, 'Saturation used: %d\n', metrics.saturation_used);
fprintf(fid, 'Saturation limit respected: %d\n', metrics.saturation_limit_respected);
fprintf(fid, 'Final theta1: %.9f deg\n', metrics.final_theta1_deg);
fprintf(fid, 'Final theta2: %.9f deg\n', metrics.final_theta2_deg);
fprintf(fid, 'Final cart position: %.9f m\n', metrics.final_cart_position_m);

if has_phase4_result
    fprintf(fid, '\nMATLAB Phase 4 vs Simulink Phase 5 comparison:\n');
    fprintf(fid, 'Max abs theta1 difference: %.6f deg\n', comparison.max_abs_theta1_diff_deg);
    fprintf(fid, 'Max abs theta2 difference: %.6f deg\n', comparison.max_abs_theta2_diff_deg);
    fprintf(fid, 'Max abs cart difference: %.6f m\n', comparison.max_abs_cart_diff_m);
    fprintf(fid, 'Max abs control difference: %.6f N\n', comparison.max_abs_control_diff_N);
end

fclose(fid);

fprintf('\n===== Phase 5 Result =====\n');
fprintf('Simulink model: %s\n', model_file);
fprintf('Data saved to %s\n', fullfile(results_folder, 'phase5_simulink_data.mat'));
fprintf('Summary saved to %s\n', summary_file);
fprintf('Plot saved to %s\n', fullfile(results_folder, 'phase5_simulink_response.png'));
if has_phase4_result
    fprintf('Comparison plot saved to %s\n', fullfile(results_folder, 'phase5_matlab_vs_simulink.png'));
end
fprintf('Phase 5 script finished successfully.\n');

%% Local helper function
function [t, y] = local_read_workspace_signal(signal_struct)
    if isa(signal_struct, 'timeseries')
        t = signal_struct.Time(:);
        y = signal_struct.Data(:);
        return;
    end

    if isstruct(signal_struct)
        if isfield(signal_struct, 'time') && isfield(signal_struct, 'signals')
            t = signal_struct.time(:);
            y = signal_struct.signals.values(:);
            return;
        end
    end

    error('Unsupported logged signal format. Use Structure With Time or Timeseries.');
end
