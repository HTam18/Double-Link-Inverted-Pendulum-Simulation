clear; clc; close all;

%% run_phase6_disturbance_tests
% Phase 6 main script.
% This script runs disturbance and larger initial angle tests using the
% Simulink closed-loop model created in Phase 5/6.
%
% Main output folder:
% results/phase6_disturbance_analysis/
%
% Main model:
% models/double_link_lqr.slx
%
% Required disturbance-ready model structure:
% u_raw = -K*x
% u_sat = saturation(u_raw, -u_max, +u_max)
% u_total = u_sat + disturbance_force

%% Make paths robust
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase6_disturbance_analysis');
addpath(script_dir);

if ~exist(models_folder, 'dir')
    mkdir(models_folder);
end
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

%% Load parameters and model
run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase6_disturbance_analysis');
addpath(script_dir);

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g); %#ok<NASGU>

% Selected C13 controller.
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
closed_loop_poles = eig(A - B*K);
closed_loop_stable = all(real(closed_loop_poles) < 0);

if ~closed_loop_stable
    error('Selected C13 LQR controller is not stable. Stop before Phase 6 tests.');
end

if u_max <= 0
    error('u_max must be positive.');
end

fprintf('\n===== Phase 6: Disturbance, Tuning, and Analysis =====\n');
fprintf('Selected controller: C13_cart_weight_8_R20\n');
fprintf('Q = %s\n', mat2str(Q));
fprintf('R = %.4f\n', R);
fprintf('u_max = %.4f N\n', u_max);
fprintf('Closed-loop stable without saturation: %d\n', closed_loop_stable);

%% Recreate disturbance-ready Simulink model
% This makes sure the model has the disturbance force path required by Phase 6.
fprintf('\nCreating/updating disturbance-ready Simulink model...\n');
run(fullfile(script_dir, 'create_double_link_lqr_model.m'));

% create_double_link_lqr_model.m may clear the workspace through the parameter file.
% Rebuild all variables needed for this script.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase6_disturbance_analysis');
addpath(script_dir);

run(fullfile(script_dir, 'parameters_double_link.m'));
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
results_folder = fullfile(project_root, 'results', 'phase6_disturbance_analysis');
addpath(script_dir);

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g); %#ok<NASGU>
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);
closed_loop_poles = eig(A - B*K);
closed_loop_stable = all(real(closed_loop_poles) < 0);

model_name = 'double_link_lqr';
model_file = fullfile(models_folder, [model_name, '.slx']);
if ~exist(model_file, 'file')
    error('Simulink model was not created: %s', model_file);
end

%% Define Phase 6 test cases
% Do not use very large angle or disturbance values in this linear LQR model.
% The controller is designed around the upright equilibrium.
base_x0 = x0;

cases = struct([]);

cases(1).name = 'baseline_3deg_no_disturbance';
cases(1).theta1_deg = 3;
cases(1).theta2_deg = -3;
cases(1).disturbance_amp_N = 0;
cases(1).disturbance_start_s = NaN;
cases(1).disturbance_duration_s = 0;

cases(2).name = 'small_force_disturbance_2N';
cases(2).theta1_deg = 3;
cases(2).theta2_deg = -3;
cases(2).disturbance_amp_N = 2;
cases(2).disturbance_start_s = 2.0;
cases(2).disturbance_duration_s = 0.2;

cases(3).name = 'medium_force_disturbance_3N';
cases(3).theta1_deg = 3;
cases(3).theta2_deg = -3;
cases(3).disturbance_amp_N = 3;
cases(3).disturbance_start_s = 2.0;
cases(3).disturbance_duration_s = 0.2;

cases(4).name = 'larger_initial_angle_5deg';
cases(4).theta1_deg = 5;
cases(4).theta2_deg = -5;
cases(4).disturbance_amp_N = 0;
cases(4).disturbance_start_s = NaN;
cases(4).disturbance_duration_s = 0;

%% Run all cases
case_results = struct([]);
metrics_list = cell(numel(cases), 1);

for i = 1:numel(cases)
    this_case = cases(i);

    fprintf('\n----- Running case %d/%d: %s -----\n', i, numel(cases), this_case.name);

    % Set initial state for this case.
    x0 = base_x0;
    x0(3) = deg2rad(this_case.theta1_deg);
    x0(5) = deg2rad(this_case.theta2_deg);

    % Build disturbance force input for Simulink From Workspace block.
    disturbance_force_ts = local_make_disturbance_timeseries(sim_time, ...
        this_case.disturbance_amp_N, this_case.disturbance_start_s, this_case.disturbance_duration_s);

    % Run Simulink.
    load_system(model_file);
    simOut = sim(model_name, 'StopTime', num2str(sim_time), 'ReturnWorkspaceOutputs', 'on');

    % Read logged signals.
    [t_sim, x_sim] = local_read_workspace_signal(simOut.get('sim_x'));
    [~, x_dot_sim] = local_read_workspace_signal(simOut.get('sim_x_dot'));
    [~, theta1_sim] = local_read_workspace_signal(simOut.get('sim_theta1'));
    [~, theta1_dot_sim] = local_read_workspace_signal(simOut.get('sim_theta1_dot'));
    [~, theta2_sim] = local_read_workspace_signal(simOut.get('sim_theta2'));
    [~, theta2_dot_sim] = local_read_workspace_signal(simOut.get('sim_theta2_dot'));
    [~, u_raw_sim] = local_read_workspace_signal(simOut.get('sim_u_raw'));
    [~, u_sat_sim] = local_read_workspace_signal(simOut.get('sim_u_sat'));
    [~, disturbance_sim] = local_read_workspace_signal(simOut.get('sim_disturbance_force'));
    [~, u_total_sim] = local_read_workspace_signal(simOut.get('sim_u_total'));

    states_sim = [x_sim(:), x_dot_sim(:), theta1_sim(:), theta1_dot_sim(:), theta2_sim(:), theta2_dot_sim(:)];

    options = struct();
    options.case_name = this_case.name;
    options.u_max = u_max;
    options.angle_settle_threshold_deg = 0.5;
    options.cart_settle_threshold_m = 0.05;
    options.final_angle_threshold_deg = 0.1;
    options.final_cart_threshold_m = 0.01;

    if isnan(this_case.disturbance_start_s)
        options.disturbance_start_s = NaN;
        options.disturbance_end_s = NaN;
    else
        options.disturbance_start_s = this_case.disturbance_start_s;
        options.disturbance_end_s = this_case.disturbance_start_s + this_case.disturbance_duration_s;
    end

    metrics = analyze_results_double(t_sim, states_sim, u_raw_sim, u_sat_sim, disturbance_sim, options);

    fprintf('Max abs theta1: %.6f deg\n', metrics.max_abs_theta1_deg);
    fprintf('Max abs theta2: %.6f deg\n', metrics.max_abs_theta2_deg);
    fprintf('Max abs cart position: %.6f m\n', metrics.max_abs_cart_position_m);
    fprintf('Max abs u_sat: %.6f N\n', metrics.max_abs_u_sat_N);
    fprintf('RMS u_sat: %.6f N\n', metrics.rms_u_sat_N);
    fprintf('Saturation used: %d\n', metrics.saturation_used);
    fprintf('Saturation limit respected: %d\n', metrics.saturation_limit_respected);
    fprintf('Final state OK: %d\n', metrics.final_state_ok);
    if ~isnan(metrics.recovery_time_s)
        fprintf('Recovery time after disturbance: %.6f s\n', metrics.recovery_time_s);
    end

    case_results(i).case = this_case; %#ok<SAGROW>
    case_results(i).t = t_sim; %#ok<SAGROW>
    case_results(i).states = states_sim; %#ok<SAGROW>
    case_results(i).u_raw = u_raw_sim; %#ok<SAGROW>
    case_results(i).u_sat = u_sat_sim; %#ok<SAGROW>
    case_results(i).disturbance_force = disturbance_sim; %#ok<SAGROW>
    case_results(i).u_total = u_total_sim; %#ok<SAGROW>
    case_results(i).metrics = metrics; %#ok<SAGROW>

    metrics_list{i} = metrics;

    % Individual response plot.
    local_plot_case_response(t_sim, states_sim, u_raw_sim, u_sat_sim, disturbance_sim, this_case.name, results_folder);
end

%% Save metrics table and summary
metrics_table = local_metrics_to_table(metrics_list);
metrics_csv_file = fullfile(results_folder, 'phase6_metrics_table.csv');
writetable(metrics_table, metrics_csv_file);

summary_file = fullfile(results_folder, 'phase6_disturbance_summary.txt');
local_write_summary(summary_file, Q, R, K, u_max, closed_loop_poles, closed_loop_stable, cases, metrics_list);

%% Save data
save(fullfile(results_folder, 'phase6_disturbance_data.mat'), ...
    'case_results', 'metrics_list', 'metrics_table', 'cases', 'Q', 'R', 'K', 'u_max', ...
    'closed_loop_poles', 'closed_loop_stable');

%% Create comparison plots
local_plot_disturbance_comparison(case_results, results_folder);

fprintf('\n===== Phase 6 Result =====\n');
fprintf('Simulink model: %s\n', model_file);
fprintf('Data saved to %s\n', fullfile(results_folder, 'phase6_disturbance_data.mat'));
fprintf('Summary saved to %s\n', summary_file);
fprintf('Metrics table saved to %s\n', metrics_csv_file);
fprintf('Response plots saved to %s\n', results_folder);
fprintf('Phase 6 script finished successfully.\n');

%% Local helper functions
function disturbance_force_ts = local_make_disturbance_timeseries(sim_time, amp, start_time, duration)
    if isnan(start_time) || amp == 0 || duration <= 0
        disturbance_force_ts = timeseries([0; 0], [0; sim_time]);
        return;
    end

    stop_time = start_time + duration;
    times = [0; start_time; start_time; stop_time; stop_time; sim_time];
    values = [0; 0; amp; amp; 0; 0];
    disturbance_force_ts = timeseries(values, times);
end

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

function local_plot_case_response(t, states, u_raw, u_sat, disturbance_force, case_name, results_folder)
    theta1_deg = rad2deg(states(:, 3));
    theta2_deg = rad2deg(states(:, 5));
    x = states(:, 1);

    figure('Name', ['Phase 6 ', case_name]);

    subplot(5, 1, 1);
    plot(t, theta1_deg, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('theta1 (deg)');
    title(['Link 1 Angle - ', strrep(case_name, '_', ' ')]);

    subplot(5, 1, 2);
    plot(t, theta2_deg, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('theta2 (deg)');
    title('Link 2 Angle');

    subplot(5, 1, 3);
    plot(t, x, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('x (m)');
    title('Cart Position');

    subplot(5, 1, 4);
    plot(t, u_raw, 'LineWidth', 1.2);
    hold on;
    plot(t, u_sat, '--', 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Force (N)');
    title('Control Force');
    legend('u raw', 'u saturated', 'Location', 'best');

    subplot(5, 1, 5);
    plot(t, disturbance_force, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Dist. (N)');
    title('External Force Disturbance');

    saveas(gcf, fullfile(results_folder, ['phase6_', case_name, '_response.png']));
end

function local_plot_disturbance_comparison(case_results, results_folder)
    figure('Name', 'Phase 6 Disturbance Comparison - theta1');
    hold on;
    for i = 1:numel(case_results)
        plot(case_results(i).t, rad2deg(case_results(i).states(:, 3)), 'LineWidth', 1.2);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('theta1 (deg)');
    title('Phase 6 theta1 Comparison');
    case_names = arrayfun(@(r) strrep(r.case.name, '_', ' '), case_results, 'UniformOutput', false);
    legend(case_names, 'Location', 'best');
    saveas(gcf, fullfile(results_folder, 'phase6_theta1_comparison.png'));

    figure('Name', 'Phase 6 Disturbance Comparison - theta2');
    hold on;
    for i = 1:numel(case_results)
        plot(case_results(i).t, rad2deg(case_results(i).states(:, 5)), 'LineWidth', 1.2);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('theta2 (deg)');
    title('Phase 6 theta2 Comparison');
    case_names = arrayfun(@(r) strrep(r.case.name, '_', ' '), case_results, 'UniformOutput', false);
    legend(case_names, 'Location', 'best');
    saveas(gcf, fullfile(results_folder, 'phase6_theta2_comparison.png'));

    figure('Name', 'Phase 6 Disturbance Comparison - cart');
    hold on;
    for i = 1:numel(case_results)
        plot(case_results(i).t, case_results(i).states(:, 1), 'LineWidth', 1.2);
    end
    grid on;
    xlabel('Time (s)');
    ylabel('x (m)');
    title('Phase 6 Cart Position Comparison');
    case_names = arrayfun(@(r) strrep(r.case.name, '_', ' '), case_results, 'UniformOutput', false);
    legend(case_names, 'Location', 'best');
    saveas(gcf, fullfile(results_folder, 'phase6_cart_comparison.png'));
end

function metrics_table = local_metrics_to_table(metrics_list)
    n = numel(metrics_list);
    case_name = strings(n, 1);
    max_abs_theta1_deg = zeros(n, 1);
    max_abs_theta2_deg = zeros(n, 1);
    max_abs_cart_position_m = zeros(n, 1);
    max_abs_cart_velocity_m_s = zeros(n, 1);
    max_abs_u_raw_N = zeros(n, 1);
    max_abs_u_sat_N = zeros(n, 1);
    rms_u_sat_N = zeros(n, 1);
    max_abs_disturbance_N = zeros(n, 1);
    max_abs_total_force_N = zeros(n, 1);
    final_theta1_deg = zeros(n, 1);
    final_theta2_deg = zeros(n, 1);
    final_cart_position_m = zeros(n, 1);
    theta1_settling_time_s = zeros(n, 1);
    theta2_settling_time_s = zeros(n, 1);
    cart_settling_time_s = zeros(n, 1);
    recovery_time_s = zeros(n, 1);
    saturation_used = false(n, 1);
    saturation_limit_respected = false(n, 1);
    final_state_ok = false(n, 1);

    for i = 1:n
        m = metrics_list{i};
        case_name(i) = string(m.case_name);
        max_abs_theta1_deg(i) = m.max_abs_theta1_deg;
        max_abs_theta2_deg(i) = m.max_abs_theta2_deg;
        max_abs_cart_position_m(i) = m.max_abs_cart_position_m;
        max_abs_cart_velocity_m_s(i) = m.max_abs_cart_velocity_m_s;
        max_abs_u_raw_N(i) = m.max_abs_u_raw_N;
        max_abs_u_sat_N(i) = m.max_abs_u_sat_N;
        rms_u_sat_N(i) = m.rms_u_sat_N;
        max_abs_disturbance_N(i) = m.max_abs_disturbance_N;
        max_abs_total_force_N(i) = m.max_abs_total_force_N;
        final_theta1_deg(i) = m.final_theta1_deg;
        final_theta2_deg(i) = m.final_theta2_deg;
        final_cart_position_m(i) = m.final_cart_position_m;
        theta1_settling_time_s(i) = m.theta1_settling_time_s;
        theta2_settling_time_s(i) = m.theta2_settling_time_s;
        cart_settling_time_s(i) = m.cart_settling_time_s;
        recovery_time_s(i) = m.recovery_time_s;
        saturation_used(i) = m.saturation_used;
        saturation_limit_respected(i) = m.saturation_limit_respected;
        final_state_ok(i) = m.final_state_ok;
    end

    metrics_table = table(case_name, max_abs_theta1_deg, max_abs_theta2_deg, ...
        max_abs_cart_position_m, max_abs_cart_velocity_m_s, max_abs_u_raw_N, ...
        max_abs_u_sat_N, rms_u_sat_N, max_abs_disturbance_N, max_abs_total_force_N, ...
        final_theta1_deg, final_theta2_deg, final_cart_position_m, ...
        theta1_settling_time_s, theta2_settling_time_s, cart_settling_time_s, ...
        recovery_time_s, saturation_used, saturation_limit_respected, final_state_ok);
end

function local_write_summary(summary_file, Q, R, K, u_max, closed_loop_poles, closed_loop_stable, cases, metrics_list)
    fid = fopen(summary_file, 'w');
    if fid == -1
        error('Cannot write summary file: %s', summary_file);
    end

    fprintf(fid, 'Phase 6 Disturbance and Analysis Summary\n');
    fprintf(fid, '========================================\n\n');
    fprintf(fid, 'Selected controller: C13_cart_weight_8_R20\n');
    fprintf(fid, 'Q = %s\n', mat2str(Q));
    fprintf(fid, 'R = %.6f\n', R);
    fprintf(fid, 'K = %s\n', mat2str(K, 8));
    fprintf(fid, 'u_max = %.6f N\n', u_max);
    fprintf(fid, 'Closed-loop stable without saturation: %d\n', closed_loop_stable);
    fprintf(fid, 'Closed-loop poles:\n');
    for i = 1:length(closed_loop_poles)
        fprintf(fid, '  %.8f%+.8fi\n', real(closed_loop_poles(i)), imag(closed_loop_poles(i)));
    end

    fprintf(fid, '\nTest cases:\n');
    for i = 1:numel(cases)
        fprintf(fid, '%d. %s\n', i, cases(i).name);
        fprintf(fid, '   theta1_0 = %.3f deg, theta2_0 = %.3f deg\n', cases(i).theta1_deg, cases(i).theta2_deg);
        fprintf(fid, '   disturbance amplitude = %.3f N\n', cases(i).disturbance_amp_N);
        if isnan(cases(i).disturbance_start_s)
            fprintf(fid, '   disturbance: none\n');
        else
            fprintf(fid, '   disturbance start = %.3f s, duration = %.3f s\n', cases(i).disturbance_start_s, cases(i).disturbance_duration_s);
        end
    end

    fprintf(fid, '\nMetrics:\n');
    for i = 1:numel(metrics_list)
        m = metrics_list{i};
        fprintf(fid, '\nCase: %s\n', m.case_name);
        fprintf(fid, '  Max abs theta1: %.6f deg\n', m.max_abs_theta1_deg);
        fprintf(fid, '  Max abs theta2: %.6f deg\n', m.max_abs_theta2_deg);
        fprintf(fid, '  Max abs cart position: %.6f m\n', m.max_abs_cart_position_m);
        fprintf(fid, '  Max abs cart velocity: %.6f m/s\n', m.max_abs_cart_velocity_m_s);
        fprintf(fid, '  Max abs u_raw: %.6f N\n', m.max_abs_u_raw_N);
        fprintf(fid, '  Max abs u_sat: %.6f N\n', m.max_abs_u_sat_N);
        fprintf(fid, '  RMS u_sat: %.6f N\n', m.rms_u_sat_N);
        fprintf(fid, '  Max abs disturbance: %.6f N\n', m.max_abs_disturbance_N);
        fprintf(fid, '  Max abs total plant force: %.6f N\n', m.max_abs_total_force_N);
        fprintf(fid, '  Final theta1: %.9f deg\n', m.final_theta1_deg);
        fprintf(fid, '  Final theta2: %.9f deg\n', m.final_theta2_deg);
        fprintf(fid, '  Final cart position: %.9f m\n', m.final_cart_position_m);
        fprintf(fid, '  theta1 settling time: %.6f s\n', m.theta1_settling_time_s);
        fprintf(fid, '  theta2 settling time: %.6f s\n', m.theta2_settling_time_s);
        fprintf(fid, '  cart settling time: %.6f s\n', m.cart_settling_time_s);
        fprintf(fid, '  recovery time: %.6f s\n', m.recovery_time_s);
        fprintf(fid, '  saturation used: %d\n', m.saturation_used);
        fprintf(fid, '  saturation limit respected: %d\n', m.saturation_limit_respected);
        fprintf(fid, '  final state OK: %d\n', m.final_state_ok);
    end

    fprintf(fid, '\nPhase 6 completion rule:\n');
    fprintf(fid, 'The phase is acceptable if the selected controller keeps u_sat within +/-u_max,\n');
    fprintf(fid, 'both link angles return close to upright, the cart position remains reasonable,\n');
    fprintf(fid, 'and the disturbance cases recover after the external force pulse.\n');

    fclose(fid);
end
