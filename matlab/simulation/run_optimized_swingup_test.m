% RUN_OPTIMIZED_SWINGUP_TEST Phase 28 replay test for optimized trajectory.
%
% This script:
%   1) creates the optimized trajectory if it does not exist,
%   2) performs an exact mesh open-loop replay on the nonlinear plant,
%   3) also performs the reusable run_simulation replay as a diagnostic,
%   4) writes summaries and figures for report/debugging.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad downward, theta = pi rad upright.

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
result_dir = fullfile(project_root, 'results', 'phase28_direct_collocation');
trajectory_file = fullfile(project_root, 'shared', 'trajectories', 'up_up_swingup_optimized.mat');
summary_file = fullfile(result_dir, 'optimized_replay_summary.txt');
exact_plot_file = fullfile(result_dir, 'optimized_exact_replay_result.png');
runner_plot_file = fullfile(result_dir, 'optimized_runner_replay_result.png');
compare_plot_file = fullfile(result_dir, 'optimized_replay_compare_reference.png');
status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE28.md');

if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

fprintf('Phase 28 optimized swing-up replay test\n');
fprintf('state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf('target_mode = up_up\n');

if ~exist(trajectory_file, 'file')
    fprintf('trajectory file not found; running optimize_swingup_direct_collocation() first...\n');
    optimize_swingup_direct_collocation();
else
    preloaded = load(trajectory_file, 'trajectory');
    if ~isfield(preloaded, 'trajectory') || ~isfield(preloaded.trajectory, 'method') || ...
            ~contains(string(preloaded.trajectory.method), 'rk4_zoh')
        fprintf('existing trajectory is not RK4-ZOH consistent; regenerating Phase 28 trajectory...\n');
        optimize_swingup_direct_collocation();
    end
end

loaded = load(trajectory_file, 'trajectory');
trajectory = loaded.trajectory;

% Exact mesh nonlinear replay. This is the Phase 28 gate because it checks
% the optimized feedforward trajectory on the nonlinear plant without adding
% interpolation/runner effects.
exact_replay = replay_optimized_trajectory_exact(trajectory, params);
exact_metrics = local_replay_metrics(exact_replay, trajectory, params, 'exact_mesh');

% Reusable runner replay. This is kept as a diagnostic. If it fails while
% exact replay passes, the project should move to Phase 29 tracking, because
% feedforward-only playback is sensitive to timing/interpolation and model
% mismatch.
cfg = struct();
cfg.mode = 'phase28_optimized_runner_replay';
cfg.target_mode = 'up_up';
cfg.dt = min(double(params.simulation.time_step_s), double(trajectory.dt_s) / 5.0);
cfg.t_final = double(trajectory.final_time_s);
cfg.initial_state = trajectory.x_ref(1, :).';
cfg.controller = @(t, state, params_in, sim_config) local_trajectory_feedforward_controller(t, state, params_in, sim_config, trajectory); %#ok<NASGU>
cfg.realism = struct();
cfg.realism.rail_limit_enabled = false;
cfg.realism.actuator_enabled = false;
cfg.realism.friction_enabled = false;
cfg.realism.sensor_noise_enabled = false;
cfg.realism.estimator_enabled = false;
cfg.realism.use_estimator_for_control = false;
cfg.realism.use_measurement_for_control = false;
runner_replay = run_simulation(cfg, params);
runner_metrics = local_replay_metrics(runner_replay, trajectory, params, 'runner_zoh');

metrics = struct();
metrics.exact = exact_metrics;
metrics.runner = runner_metrics;
metrics.phase28_gate_pass = exact_metrics.replay_pass;
metrics.runner_diagnostic_pass = runner_metrics.replay_pass;
metrics.runner_needs_tracking = exact_metrics.replay_pass && ~runner_metrics.replay_pass;

save(fullfile(result_dir, 'optimized_replay_result.mat'), 'exact_replay', 'runner_replay', 'trajectory', 'metrics');
local_write_replay_summary(summary_file, trajectory, metrics);
local_write_status_append(status_file, metrics);
local_plot_replay(exact_replay, runner_replay, trajectory, exact_plot_file, runner_plot_file, compare_plot_file);

fprintf('optimization_usable = %d\n', trajectory.metrics.optimization_usable);
fprintf('exact_replay_finite_pass = %d\n', exact_metrics.finite_pass);
fprintf('exact_replay_rail_pass = %d\n', exact_metrics.rail_pass);
fprintf('exact_replay_force_pass = %d\n', exact_metrics.force_pass);
fprintf('exact_replay_final_max_angle_error_rad = %.12g\n', exact_metrics.final_max_angle_error_rad);
fprintf('exact_replay_final_velocity_norm = %.12g\n', exact_metrics.final_velocity_norm);
fprintf('exact_replay_max_abs_x_m = %.12g\n', exact_metrics.max_abs_x_m);
fprintf('exact_replay_max_abs_u_actual_N = %.12g\n', exact_metrics.max_abs_u_actual_N);
fprintf('exact_replay_max_defect_vs_reference = %.12g\n', exact_metrics.max_defect_vs_reference_inf_norm);
fprintf('exact_replay_pass = %d\n', exact_metrics.replay_pass);
fprintf('runner_replay_final_max_angle_error_rad = %.12g\n', runner_metrics.final_max_angle_error_rad);
fprintf('runner_replay_final_velocity_norm = %.12g\n', runner_metrics.final_velocity_norm);
fprintf('runner_replay_pass = %d\n', runner_metrics.replay_pass);
fprintf('runner_needs_tracking = %d\n', metrics.runner_needs_tracking);
fprintf('phase28_gate_pass = %d\n', metrics.phase28_gate_pass);

if metrics.phase28_gate_pass
    fprintf('Phase 28 run_optimized_swingup_test: PASS_WITH_EXACT_NONLINEAR_REPLAY\n');
    if metrics.runner_needs_tracking
        fprintf('Note: reusable runner feedforward replay did not pass; Phase 29 tracking is required before using this trajectory robustly.\n');
    end
else
    fprintf('Phase 28 run_optimized_swingup_test: NOT_PASS_REVIEW_REQUIRED\n');
end

function out = local_trajectory_feedforward_controller(t, state, params, sim_config, trajectory) %#ok<INUSD>
    u = local_zoh_lookup(t, trajectory.t(:), trajectory.u_ff(:));
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    u = min(max(u, u_min), u_max);

    out = struct();
    out.u_cmd = u;
    out.mode = 'optimized_feedforward_open_loop';
    out.measurement = state(:);
    out.x_hat = state(:);
end

function u = local_zoh_lookup(t, time_grid, u_grid)
    idx = find(time_grid(1:end-1) <= t, 1, 'last');
    if isempty(idx)
        idx = 1;
    end
    idx = min(idx, numel(u_grid));
    u = u_grid(idx);
end

function metrics = local_replay_metrics(result, trajectory, params, replay_kind)
    X = result.state;
    Ucmd = result.u_cmd(:);
    Uactual = result.u_actual(:);
    final_error_1 = abs(local_wrap_to_pi(X(end, 3) - pi));
    final_error_2 = abs(local_wrap_to_pi(X(end, 5) - pi));
    final_max_angle_error = max([final_error_1, final_error_2]);
    final_velocity_norm = norm([X(end, 2), X(end, 4), X(end, 6)]);
    max_abs_x = max(abs(X(:, 1)));
    max_abs_u_cmd = max(abs(Ucmd));
    max_abs_u_actual = max(abs(Uactual));

    x_min = double(params.rail_limit.x_min_m);
    x_max = double(params.rail_limit.x_max_m);
    u_lim = max(abs([double(params.control.min_cart_force_N), double(params.control.max_cart_force_N)]));

    finite_pass = all(isfinite(X(:))) && all(isfinite(Ucmd)) && all(isfinite(Uactual));
    rail_pass = min(X(:, 1)) >= x_min - 1.0e-9 && max(X(:, 1)) <= x_max + 1.0e-9;
    force_pass = max_abs_u_cmd <= u_lim + 1.0e-9 && max_abs_u_actual <= u_lim + 1.0e-9;

    reference_phase27_full_diagnostic_angle_error_rad = 1.44468680923;
    better_than_phase27 = final_max_angle_error < reference_phase27_full_diagnostic_angle_error_rad;
    terminal_capture_like = final_max_angle_error <= 0.50 && final_velocity_norm <= 5.0;

    if isfield(result, 'max_defect_vs_reference_inf_norm')
        max_defect_vs_reference = result.max_defect_vs_reference_inf_norm;
    else
        max_defect_vs_reference = NaN;
    end

    replay_pass = trajectory.metrics.optimization_usable && finite_pass && rail_pass && force_pass && ...
                  better_than_phase27 && terminal_capture_like;

    metrics = struct();
    metrics.replay_kind = char(replay_kind);
    metrics.finite_pass = finite_pass;
    metrics.rail_pass = rail_pass;
    metrics.force_pass = force_pass;
    metrics.terminal_capture_like = terminal_capture_like;
    metrics.better_than_phase27_full_diagnostic = better_than_phase27;
    metrics.replay_pass = replay_pass;
    metrics.final_angle_error_1_rad = final_error_1;
    metrics.final_angle_error_2_rad = final_error_2;
    metrics.final_max_angle_error_rad = final_max_angle_error;
    metrics.final_velocity_norm = final_velocity_norm;
    metrics.max_abs_x_m = max_abs_x;
    metrics.max_abs_u_cmd_N = max_abs_u_cmd;
    metrics.max_abs_u_actual_N = max_abs_u_actual;
    metrics.max_defect_vs_reference_inf_norm = max_defect_vs_reference;
    metrics.reference_phase27_full_diagnostic_angle_error_rad = reference_phase27_full_diagnostic_angle_error_rad;
end

function local_write_replay_summary(path, trajectory, metrics)
    fid = fopen(path, 'w');
    if fid < 0
        warning('Phase28Replay:SummaryOpenFailed', 'Could not write summary: %s', path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'Phase 28 optimized trajectory replay summary\n');
    fprintf(fid, 'trajectory_method = %s\n', trajectory.method);
    fprintf(fid, 'target_mode = %s\n', trajectory.target_mode);
    fprintf(fid, 'optimization_usable = %d\n', trajectory.metrics.optimization_usable);
    fprintf(fid, 'phase28_gate_pass = %d\n', metrics.phase28_gate_pass);
    fprintf(fid, 'runner_diagnostic_pass = %d\n', metrics.runner_diagnostic_pass);
    fprintf(fid, 'runner_needs_tracking = %d\n', metrics.runner_needs_tracking);
    local_write_metric_block(fid, 'exact', metrics.exact);
    local_write_metric_block(fid, 'runner', metrics.runner);
end

function local_write_metric_block(fid, prefix, m)
    fields = fieldnames(m);
    for i = 1:numel(fields)
        value = m.(fields{i});
        if ischar(value) || isstring(value)
            fprintf(fid, '%s_%s = %s\n', prefix, fields{i}, char(value));
        elseif islogical(value)
            fprintf(fid, '%s_%s = %d\n', prefix, fields{i}, value);
        else
            fprintf(fid, '%s_%s = %.12g\n', prefix, fields{i}, double(value));
        end
    end
end

function local_write_status_append(path, metrics)
    fid = fopen(path, 'a');
    if fid < 0
        warning('Phase28Replay:StatusOpenFailed', 'Could not append status: %s', path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '\n## Replay test hotfix: exact nonlinear mesh replay and runner diagnostic\n\n');
    fprintf(fid, '- exact_replay_pass: `%d`\n', metrics.exact.replay_pass);
    fprintf(fid, '- exact_replay_final_max_angle_error_rad: `%.12g`\n', metrics.exact.final_max_angle_error_rad);
    fprintf(fid, '- exact_replay_final_velocity_norm: `%.12g`\n', metrics.exact.final_velocity_norm);
    fprintf(fid, '- runner_replay_pass: `%d`\n', metrics.runner.replay_pass);
    fprintf(fid, '- runner_replay_final_max_angle_error_rad: `%.12g`\n', metrics.runner.final_max_angle_error_rad);
    fprintf(fid, '- runner_replay_final_velocity_norm: `%.12g`\n', metrics.runner.final_velocity_norm);
    fprintf(fid, '- phase28_gate_pass: `%d`\n', metrics.phase28_gate_pass);
    fprintf(fid, '- runner_needs_tracking: `%d`\n', metrics.runner_needs_tracking);
    fprintf(fid, '\nSenior note: exact nonlinear replay validates the optimized open-loop trajectory on the same nonlinear plant and mesh. Runner feedforward replay is kept as a diagnostic because Phase 29 tracking is expected to make trajectory playback robust to interpolation, timing and model mismatch.\n');
end

function local_plot_replay(exact_result, runner_result, trajectory, exact_plot_file, runner_plot_file, compare_plot_file)
    try
        plot_results(exact_result, exact_plot_file);
    catch plot_error
        warning('Phase28Replay:ExactPlotFailed', 'plot_results exact failed: %s', plot_error.message);
    end
    try
        plot_results(runner_result, runner_plot_file);
    catch plot_error
        warning('Phase28Replay:RunnerPlotFailed', 'plot_results runner failed: %s', plot_error.message);
    end

    try
        fig = figure('Name', 'Phase 28 exact and runner replay compare', 'Color', 'w');
        tiledlayout(3, 1);
        nexttile;
        plot(trajectory.t(:), trajectory.x_ref(:, 1), '--', 'LineWidth', 1.2); hold on;
        plot(exact_result.t(:), exact_result.state(:, 1), 'LineWidth', 1.2);
        plot(runner_result.t(:), runner_result.state(:, 1), 'LineWidth', 1.2); grid on;
        ylabel('x [m]'); legend('x ref', 'x exact', 'x runner', 'Location', 'best');
        title('Optimized trajectory replay comparison');
        nexttile;
        plot(trajectory.t(:), trajectory.x_ref(:, 3), '--', 'LineWidth', 1.2); hold on;
        plot(trajectory.t(:), trajectory.x_ref(:, 5), '--', 'LineWidth', 1.2);
        plot(exact_result.t(:), exact_result.state(:, 3), 'LineWidth', 1.2);
        plot(exact_result.t(:), exact_result.state(:, 5), 'LineWidth', 1.2);
        plot(runner_result.t(:), runner_result.state(:, 3), 'LineWidth', 1.2);
        plot(runner_result.t(:), runner_result.state(:, 5), 'LineWidth', 1.2); grid on;
        ylabel('angle [rad]'); legend('th1 ref', 'th2 ref', 'th1 exact', 'th2 exact', 'th1 runner', 'th2 runner', 'Location', 'best');
        nexttile;
        stairs(trajectory.t(1:end-1), trajectory.u_ff(:), '--', 'LineWidth', 1.2); hold on;
        plot(exact_result.t(:), exact_result.u_actual(:), 'LineWidth', 1.2);
        plot(runner_result.t(:), runner_result.u_actual(:), 'LineWidth', 1.2); grid on;
        xlabel('time [s]'); ylabel('force [N]'); legend('u ff', 'u exact', 'u runner', 'Location', 'best');
        saveas(fig, compare_plot_file);
    catch plot_error
        warning('Phase28Replay:ComparePlotFailed', 'Compare plot failed: %s', plot_error.message);
    end
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2.0 * pi) - pi;
end
