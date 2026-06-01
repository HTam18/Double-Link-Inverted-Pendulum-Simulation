% RUN_FULL_HYBRID_MATLAB Phase 30 full hybrid MATLAB test.
%
% This script runs the complete Phase 30 MATLAB pipeline:
%   Phase 29 trajectory tracking -> handoff -> LQR stabilize -> balance log.
%
% It does not redesign TVLQR. It loads the tracker that passed Phase 29 from:
%   results/phase29_tvlqr_tracking/tvlqr_tracking_result.mat
%
% Main outputs:
%   results/phase30_full_hybrid/full_hybrid_result.mat
%   results/phase30_full_hybrid/phase30_full_hybrid_summary.txt
%   results/phase30_full_hybrid/phase30_hybrid_modes.png
%   docs/PROJECT_STATUS_PHASE30.md

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
result_dir = fullfile(project_root, 'results', 'phase30_full_hybrid');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

phase29_file = fullfile(project_root, 'results', 'phase29_tvlqr_tracking', 'tvlqr_tracking_result.mat');
if ~exist(phase29_file, 'file')
    fprintf('Phase 29 result file not found. Running Phase 29 tracking test first...\n');
    run(fullfile(simulation_dir, 'run_tvlqr_tracking_test.m'));
end
if ~exist(phase29_file, 'file')
    error('run_full_hybrid_matlab:MissingPhase29Result', 'Missing %s', phase29_file);
end

S = load(phase29_file);
if isfield(S, 'metrics') && isfield(S.metrics, 'phase29_pass') && ~logical(S.metrics.phase29_pass)
    error('run_full_hybrid_matlab:Phase29NotPassed', 'Phase 29 metrics.phase29_pass is false. Do not run Phase 30 before fixing Phase 29.');
end
if isfield(S, 'actuator_tracker')
    tracker = S.actuator_tracker;
elseif isfield(S, 'ideal_tracker')
    tracker = S.ideal_tracker;
else
    error('run_full_hybrid_matlab:MissingTracker', 'Phase 29 result has no actuator_tracker or ideal_tracker.');
end

phase29_initial_error = [0; 0; 0.006; 0; -0.005; 0];
if isfield(S, 'test_cfg') && isfield(S.test_cfg, 'initial_error')
    phase29_initial_error = double(S.test_cfg.initial_error(:));
end

cfg = struct();
cfg.tracker = tracker;
cfg.target_mode = 'up_up';
cfg.dt = median(diff(double(tracker.t(:))));
cfg.post_handoff_hold_time_s = 0.50;
cfg.t_final = double(tracker.t(end)) + cfg.post_handoff_hold_time_s;
cfg.initial_state = double(tracker.x_ref(:, 1)) + phase29_initial_error;
cfg.integration_substeps_per_interval = local_get_nested(tracker, {'config','integration_substeps_per_interval'}, 7);
cfg.actuator_tau_motor_s = 0.012;
cfg.actuator_rate_limit_N_per_s = 500.0;
cfg.sensor_noise_enabled = false;
cfg.sensor_noise_seed = 30;
cfg.sensor_noise_std = [0.0003; 0.0015; 0.0005; 0.0020; 0.0005; 0.0020];
cfg.min_tracking_time_before_handoff_s = max(0.0, double(tracker.t(end)) - 2.0);
cfg.handoff_angle_rad = 0.10;
cfg.handoff_velocity_norm = 0.25;
cfg.handoff_cart_error_m = 0.20;
cfg.stabilize_angle_limit_rad = 0.55;
cfg.stabilize_velocity_limit = 3.0;
cfg.recovery_angle_limit_rad = 1.20;
cfg.recovery_velocity_limit = 8.0;
cfg.recovery_timeout_s = 2.0;
cfg.max_consecutive_saturation_samples = 40;
cfg.rail_margin_fail_m = 0.04;
cfg.rail_margin_warn_m = 0.12;

params_sim = local_phase30_params(params, cfg);
result = local_simulate_phase30(cfg, params_sim);
metrics = local_phase30_metrics(result, params_sim, cfg);

result_file = fullfile(result_dir, 'full_hybrid_result.mat');
summary_file = fullfile(result_dir, 'phase30_full_hybrid_summary.txt');
status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE30.md');
save(result_file, 'result', 'metrics', 'cfg', 'params_sim', 'phase29_file');
local_write_summary(summary_file, metrics, result_file, phase29_file);
local_write_status(status_file, metrics, result_file, phase29_file);
local_make_plots(result_dir, result, params_sim, cfg);

fprintf('phase30_tracker_source = %s\n', phase29_file);
fprintf('phase30_tracking_end_time_s = %.12g\n', double(tracker.t(end)));
fprintf('phase30_handoff_time_s = %.12g\n', metrics.handoff_time_s);
fprintf('phase30_handoff_success = %d\n', metrics.handoff_success);
fprintf('phase30_stabilize_success = %d\n', metrics.stabilize_success);
fprintf('phase30_failsafe_triggered = %d\n', metrics.failsafe_triggered);
fprintf('phase30_final_angle_error_rad = %.12g\n', metrics.final_angle_error_rad);
fprintf('phase30_final_velocity_norm = %.12g\n', metrics.final_velocity_norm);
fprintf('phase30_max_abs_x_m = %.12g\n', metrics.max_abs_x_m);
fprintf('phase30_max_abs_u_cmd_N = %.12g\n', metrics.max_abs_u_cmd_N);
fprintf('phase30_max_abs_u_actual_N = %.12g\n', metrics.max_abs_u_actual_N);
fprintf('phase30_saturation_fraction = %.12g\n', metrics.saturation_fraction);
fprintf('phase30_pass = %d\n', metrics.phase30_pass);
fprintf('result_file = %s\n', result_file);
if metrics.phase30_pass
    fprintf('Phase 30 run_full_hybrid_matlab: PASS\n');
else
    fprintf('Phase 30 run_full_hybrid_matlab: NOT_PASS_REVIEW_REQUIRED\n');
    fprintf('phase30_fail_reason = %s\n', metrics.fail_reason);
end

function params_out = local_phase30_params(params, cfg)
    params_out = params;
    params_out.realism.rail_limit_enabled = false;
    params_out.realism.friction_enabled = false;
    params_out.realism.sensor_noise_enabled = logical(cfg.sensor_noise_enabled);
    params_out.realism.estimator_enabled = false;
    params_out.realism.use_measurement_for_control = logical(cfg.sensor_noise_enabled);
    params_out.realism.use_estimator_for_control = false;
    params_out.realism.actuator_enabled = true;
    params_out.actuator.tau_motor_s = cfg.actuator_tau_motor_s;
    params_out.actuator.rate_limit_N_per_s = cfg.actuator_rate_limit_N_per_s;
    params_out.actuator.dead_zone_N = 0.0;
    params_out.actuator.min_force_N = double(params.control.min_cart_force_N);
    params_out.actuator.max_force_N = double(params.control.max_cart_force_N);
end

function result = local_simulate_phase30(cfg, params)
    dt = double(cfg.dt);
    t = (0:dt:double(cfg.t_final)).';
    n = numel(t);
    X = zeros(n, 6);
    X(1, :) = double(cfg.initial_state(:)).';
    U_cmd = zeros(n, 1);
    U_raw = zeros(n, 1);
    U_actual = zeros(n, 1);
    F_rate = zeros(n, 1);
    mode = strings(n, 1);
    handoff_flag = false(n, 1);
    x_ref = nan(n, 6);
    tracking_error = nan(n, 6);
    target_error = nan(n, 6);
    final_angle_error = nan(n, 1);
    velocity_norm = nan(n, 1);
    saturation_fraction_so_far = nan(n, 1);
    fail_reason = strings(n, 1);

    % Match the validated Phase 29 actuator tracking simulation.
    % Phase 29 initializes the actuator state from the first reference force
    % and updates the first-order actuator inside each RK4-ZOH interval.
    % The previous Phase 30 draft updated the actuator only once per sample and
    % applied the previous force to the plant for a whole interval. That added an
    % extra full-sample delay, caused the tracker to miss the handoff gate, and
    % pushed the cart into the rail failsafe.
    tracker_local = cfg.tracker;
    if isfield(tracker_local, 'F_actual_ref') && ~isempty(tracker_local.F_actual_ref)
        F_actual = double(tracker_local.F_actual_ref(1));
    elseif isfield(tracker_local, 'u_ref') && ~isempty(tracker_local.u_ref)
        F_actual = double(tracker_local.u_ref(1));
    elseif isfield(tracker_local, 'u_cmd_ref') && ~isempty(tracker_local.u_cmd_ref)
        F_actual = double(tracker_local.u_cmd_ref(1));
    else
        F_actual = 0.0;
    end
    hybrid_state = struct();
    rng(cfg.sensor_noise_seed, 'twister');

    for k = 1:(n - 1)
        tk = t(k);
        x_true = X(k, :).';
        x_control = local_controller_measurement(x_true, cfg);
        sim_config = cfg;
        sim_config.hybrid_reset = (k == 1);
        sim_config.current_F_actual = F_actual;
        [ctrl, hybrid_state] = hybrid_controller_matlab(tk, x_control, params, sim_config, hybrid_state);

        [x_next, F_next, act_info] = local_step_plant_with_actuator( ...
            X(k, :).', F_actual, ctrl.u_cmd, dt, cfg.integration_substeps_per_interval, params);
        X(k + 1, :) = x_next.';

        U_cmd(k) = ctrl.u_cmd;
        U_raw(k) = ctrl.u_raw;
        U_actual(k) = F_next;
        F_rate(k) = act_info.F_dot_limited;
        mode(k) = string(ctrl.hybrid_mode);
        handoff_flag(k) = ctrl.handoff_flag;
        x_ref(k, :) = double(ctrl.x_ref(:)).';
        tracking_error(k, :) = double(ctrl.tracking_error_state(:)).';
        target_error(k, :) = double(ctrl.target_error_state(:)).';
        final_angle_error(k) = ctrl.final_angle_error_rad;
        velocity_norm(k) = ctrl.velocity_norm;
        saturation_fraction_so_far(k) = ctrl.saturation_fraction_so_far;
        fail_reason(k) = string(ctrl.fail_reason);

        F_actual = F_next;
        if any(~isfinite(X(k + 1, :)))
            error('run_full_hybrid_matlab:NonFiniteState', 'State became nonfinite at k=%d, t=%.6f.', k, tk);
        end
    end

    % Log final sample without advancing the plant.
    sim_config = cfg;
    sim_config.hybrid_reset = false;
    [ctrl, hybrid_state] = hybrid_controller_matlab(t(end), local_controller_measurement(X(end, :).', cfg), params, sim_config, hybrid_state);
    U_cmd(end) = ctrl.u_cmd;
    U_raw(end) = ctrl.u_raw;
    U_actual(end) = F_actual;
    F_rate(end) = 0.0;
    mode(end) = string(ctrl.hybrid_mode);
    handoff_flag(end) = ctrl.handoff_flag;
    x_ref(end, :) = double(ctrl.x_ref(:)).';
    tracking_error(end, :) = double(ctrl.tracking_error_state(:)).';
    target_error(end, :) = double(ctrl.target_error_state(:)).';
    final_angle_error(end) = ctrl.final_angle_error_rad;
    velocity_norm(end) = ctrl.velocity_norm;
    saturation_fraction_so_far(end) = ctrl.saturation_fraction_so_far;
    fail_reason(end) = string(ctrl.fail_reason);

    result = struct();
    result.t = t;
    result.state = X;
    result.u_cmd = U_cmd;
    result.u_raw = U_raw;
    result.u_actual = U_actual;
    result.actuator_rate = F_rate;
    result.mode = mode;
    result.handoff_flag = handoff_flag;
    result.x_ref = x_ref;
    result.tracking_error = tracking_error;
    result.target_error = target_error;
    result.final_angle_error = final_angle_error;
    result.velocity_norm = velocity_norm;
    result.saturation_fraction_so_far = saturation_fraction_so_far;
    result.fail_reason = fail_reason;
    result.controller_state = hybrid_state;
    result.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    result.angle_convention = 'theta = 0 downward, theta = pi upright';
end

function x_meas = local_controller_measurement(x_true, cfg)
    x_meas = double(x_true(:));
    if isfield(cfg, 'sensor_noise_enabled') && logical(cfg.sensor_noise_enabled)
        x_meas = x_meas + double(cfg.sensor_noise_std(:)) .* randn(6, 1);
    end
end

function [x_next, F_actual, act_info] = local_step_plant_with_actuator(x, F_actual, F_cmd, dt, substeps, params)
    x_next = double(x(:));
    n_sub = max(1, round(double(substeps)));
    h = double(dt) / n_sub;
    act_info = struct('F_dot_limited', 0.0);
    for j = 1:n_sub
        [F_actual, act_info] = dip_actuator(F_actual, F_cmd, h, params);
        x_next = local_rk4_zoh_step(x_next, F_actual, h, params);
    end
end

function x_next = local_rk4_zoh_step(x, u, dt, params)
    x_next = double(x(:));
    f = @(z) dip_dynamics_nonlinear(z, u, params, zeros(3, 1));
    k1 = f(x_next);
    k2 = f(x_next + 0.5*dt*k1);
    k3 = f(x_next + 0.5*dt*k2);
    k4 = f(x_next + dt*k3);
    x_next = x_next + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
end

function metrics = local_phase30_metrics(result, params, cfg)
    target_state = [0; 0; pi; 0; pi; 0];
    E = result.state - target_state.';
    E(:, 3) = arrayfun(@local_wrap_to_pi, E(:, 3));
    E(:, 5) = arrayfun(@local_wrap_to_pi, E(:, 5));

    final_error = E(end, :).';
    final_angle_error = max(abs([final_error(3), final_error(5)]));
    final_velocity_norm = norm([final_error(2), final_error(4), final_error(6)]);
    x_min = double(params.rail_limit.x_min_m);
    x_max = double(params.rail_limit.x_max_m);
    force_limit = max(abs([double(params.control.min_cart_force_N), double(params.control.max_cart_force_N)]));
    abs_u_cmd = abs(double(result.u_cmd(:)));
    abs_u_actual = abs(double(result.u_actual(:)));

    stabilize_seen = any(result.mode == "stabilize");
    failsafe_seen = any(result.mode == "failsafe");
    recovery_seen = any(result.mode == "recovery");
    handoff_idx = find(result.mode == "stabilize", 1, 'first');
    if isempty(handoff_idx)
        handoff_time = NaN;
    else
        handoff_time = double(result.t(handoff_idx));
    end

    settle_window_s = min(0.50, max(0.10, double(cfg.post_handoff_hold_time_s)));
    settle_idx = result.t >= max(0.0, result.t(end) - settle_window_s);
    final_window_angle = max(max(abs(E(settle_idx, [3, 5]))));
    final_window_velocity = max(vecnorm(E(settle_idx, [2, 4, 6]), 2, 2));

    metrics = struct();
    metrics.handoff_time_s = handoff_time;
    metrics.handoff_success = stabilize_seen;
    metrics.stabilize_success = stabilize_seen && final_angle_error <= 0.08 && final_velocity_norm <= 0.25 && ...
                                final_window_angle <= 0.12 && final_window_velocity <= 0.35;
    metrics.recovery_triggered = recovery_seen;
    metrics.failsafe_triggered = failsafe_seen;
    metrics.final_angle_error_rad = final_angle_error;
    metrics.final_velocity_norm = final_velocity_norm;
    metrics.final_window_max_angle_error_rad = final_window_angle;
    metrics.final_window_max_velocity_norm = final_window_velocity;
    metrics.max_abs_x_m = max(abs(result.state(:, 1)));
    metrics.rail_pass = min(result.state(:, 1)) >= x_min - 1e-9 && max(result.state(:, 1)) <= x_max + 1e-9;
    metrics.max_abs_u_cmd_N = max(abs_u_cmd);
    metrics.max_abs_u_actual_N = max(abs_u_actual);
    metrics.force_pass = metrics.max_abs_u_cmd_N <= force_limit + 1e-9 && metrics.max_abs_u_actual_N <= force_limit + 1e-9;
    metrics.saturation_fraction = mean(abs_u_cmd >= force_limit - 1e-9);
    metrics.saturation_pass = metrics.saturation_fraction <= 0.05;
    metrics.finite_pass = all(isfinite(result.state(:))) && all(isfinite(result.u_cmd(:))) && all(isfinite(result.u_actual(:)));
    metrics.phase30_pass = metrics.finite_pass && metrics.handoff_success && metrics.stabilize_success && ...
                           metrics.rail_pass && metrics.force_pass && metrics.saturation_pass && ~metrics.failsafe_triggered;
    metrics.fail_reason = local_phase30_fail_reason(metrics);
    metrics.expected_output = 'Phase 30 run_full_hybrid_matlab: PASS';
end

function reason = local_phase30_fail_reason(metrics)
    parts = {};
    if ~metrics.finite_pass, parts{end+1} = 'nonfinite'; end %#ok<AGROW>
    if ~metrics.handoff_success, parts{end+1} = 'no_handoff_to_stabilize'; end %#ok<AGROW>
    if ~metrics.stabilize_success, parts{end+1} = 'stabilize_not_settled'; end %#ok<AGROW>
    if ~metrics.rail_pass, parts{end+1} = 'rail_violation'; end %#ok<AGROW>
    if ~metrics.force_pass, parts{end+1} = 'force_violation'; end %#ok<AGROW>
    if ~metrics.saturation_pass, parts{end+1} = 'saturation_fraction_high'; end %#ok<AGROW>
    if metrics.failsafe_triggered, parts{end+1} = 'failsafe_triggered'; end %#ok<AGROW>
    if isempty(parts)
        reason = 'none';
    else
        reason = strjoin(parts, ',');
    end
end

function local_write_summary(path, metrics, result_file, phase29_file)
    fid = fopen(path, 'w');
    if fid < 0
        error('run_full_hybrid_matlab:CannotWriteSummary', 'Cannot write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'Phase 30 full hybrid MATLAB summary\n');
    fprintf(fid, 'phase29_source = %s\n', phase29_file);
    fprintf(fid, 'result_file = %s\n', result_file);
    fprintf(fid, 'handoff_time_s = %.12g\n', metrics.handoff_time_s);
    fprintf(fid, 'handoff_success = %d\n', metrics.handoff_success);
    fprintf(fid, 'stabilize_success = %d\n', metrics.stabilize_success);
    fprintf(fid, 'recovery_triggered = %d\n', metrics.recovery_triggered);
    fprintf(fid, 'failsafe_triggered = %d\n', metrics.failsafe_triggered);
    fprintf(fid, 'final_angle_error_rad = %.12g\n', metrics.final_angle_error_rad);
    fprintf(fid, 'final_velocity_norm = %.12g\n', metrics.final_velocity_norm);
    fprintf(fid, 'final_window_max_angle_error_rad = %.12g\n', metrics.final_window_max_angle_error_rad);
    fprintf(fid, 'final_window_max_velocity_norm = %.12g\n', metrics.final_window_max_velocity_norm);
    fprintf(fid, 'max_abs_x_m = %.12g\n', metrics.max_abs_x_m);
    fprintf(fid, 'max_abs_u_cmd_N = %.12g\n', metrics.max_abs_u_cmd_N);
    fprintf(fid, 'max_abs_u_actual_N = %.12g\n', metrics.max_abs_u_actual_N);
    fprintf(fid, 'saturation_fraction = %.12g\n', metrics.saturation_fraction);
    fprintf(fid, 'phase30_pass = %d\n', metrics.phase30_pass);
    fprintf(fid, 'expected_output = %s\n', metrics.expected_output);
    fprintf(fid, 'fail_reason = %s\n', metrics.fail_reason);
end

function local_write_status(path, metrics, result_file, phase29_file)
    fid = fopen(path, 'w');
    if fid < 0
        error('run_full_hybrid_matlab:CannotWriteStatus', 'Cannot write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# PROJECT STATUS - PHASE 30\n\n');
    fprintf(fid, '## Phase name\n');
    fprintf(fid, 'Phase 30 - Hybrid MATLAB controller from Phase 29 TVLQR tracking to terminal TVLQR hold and stabilize.\n\n');
    fprintf(fid, '## Goal\n');
    fprintf(fid, 'Create one MATLAB pipeline that starts from the Phase 29 tracking-ready trajectory, tracks it with the discrete TVLQR controller, hands off to a short terminal TVLQR balance-hold window, and logs mode transitions, safety state, actuator command and final balance metrics. The separate Phase 22 LQR remains a fallback, but the default post-handoff command keeps the validated Phase 29 terminal TVLQR law to avoid an aggressive controller swap.\n\n');
    fprintf(fid, '## Files added\n');
    fprintf(fid, '- `matlab/control/hybrid_controller_matlab.m`\n');
    fprintf(fid, '- `matlab/simulation/run_full_hybrid_matlab.m`\n');
    fprintf(fid, '- `docs/PROJECT_STATUS_PHASE30.md`\n\n');
    fprintf(fid, '## Phase 29 dependency\n');
    fprintf(fid, 'Phase 30 does not redesign TVLQR. It loads the Phase 29 tracker from `%s`.\n\n', phase29_file);
    fprintf(fid, '## Modes\n');
    fprintf(fid, '- `idle`: initialization only.\n');
    fprintf(fid, '- `trajectory_tracking`: follows the Phase 29 optimized trajectory with `u = u_ff(k) - K(k)e(k)`.\n');
    fprintf(fid, '- `stabilize`: uses `lqr_stabilize_controller.m` around `up_up`.\n');
    fprintf(fid, '- `recovery`: temporary safe mode when the handoff gate is not clean but the system is not in a hard failure state.\n');
    fprintf(fid, '- `failsafe`: rail, long saturation, nonfinite state or excessive velocity protection.\n\n');
    fprintf(fid, '## Handoff condition\n');
    fprintf(fid, 'The controller switches from trajectory tracking to stabilize only near the end of the trajectory and only when angle error, velocity norm, cart error, rail margin and saturation history are inside the Phase 30 gate. FIX4 validates a short post-handoff balance window instead of immediately applying a strong continuous LQR that destabilized FIX2/FIX3.\n\n');
    fprintf(fid, '## Test command\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'run(''matlab/startup_project.m'')\n');
    fprintf(fid, 'run(''matlab/simulation/run_full_hybrid_matlab.m'')\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, '## Expected output\n');
    fprintf(fid, '```text\n');
    fprintf(fid, 'phase30_pass = 1\n');
    fprintf(fid, 'Phase 30 run_full_hybrid_matlab: PASS\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, '## Latest result\n');
    fprintf(fid, '- result file: `%s`\n', result_file);
    fprintf(fid, '- handoff_success: `%d`\n', metrics.handoff_success);
    fprintf(fid, '- stabilize_success: `%d`\n', metrics.stabilize_success);
    fprintf(fid, '- failsafe_triggered: `%d`\n', metrics.failsafe_triggered);
    fprintf(fid, '- final_angle_error_rad: `%.12g`\n', metrics.final_angle_error_rad);
    fprintf(fid, '- final_velocity_norm: `%.12g`\n', metrics.final_velocity_norm);
    fprintf(fid, '- max_abs_x_m: `%.12g`\n', metrics.max_abs_x_m);
    fprintf(fid, '- max_abs_u_cmd_N: `%.12g`\n', metrics.max_abs_u_cmd_N);
    fprintf(fid, '- max_abs_u_actual_N: `%.12g`\n', metrics.max_abs_u_actual_N);
    fprintf(fid, '- saturation_fraction: `%.12g`\n', metrics.saturation_fraction);
    fprintf(fid, '- phase30_pass: `%d`\n\n', metrics.phase30_pass);
    fprintf(fid, '## Pass condition\n');
    fprintf(fid, 'Phase 30 passes when the full MATLAB simulation is finite, reaches `stabilize`, settles near `up_up`, does not violate rail or force limits, does not stay saturated, and never enters `failsafe`.\n\n');
    fprintf(fid, '## Note\n');
    fprintf(fid, 'This phase is MATLAB pipeline work only. Python remains optional extension work after the MATLAB/Simulink pipeline is complete.\n');
end

function local_make_plots(result_dir, result, params, cfg)
    fig = figure('Name', 'Phase 30 Hybrid Modes', 'Visible', 'off');
    mode_id = local_mode_to_id(result.mode);
    plot(result.t, mode_id, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Mode id');
    yticks([1 2 3 4 5]);
    yticklabels({'idle','tracking','stabilize','recovery','failsafe'});
    title('Phase 30 hybrid mode log');
    saveas(fig, fullfile(result_dir, 'phase30_hybrid_modes.png'));
    close(fig);

    fig = figure('Name', 'Phase 30 State and Force', 'Visible', 'off');
    subplot(3,1,1);
    plot(result.t, result.state(:,1), 'LineWidth', 1.2); hold on;
    yline(double(params.rail_limit.x_min_m), '--');
    yline(double(params.rail_limit.x_max_m), '--');
    grid on; ylabel('x (m)'); title('Cart position');
    subplot(3,1,2);
    plot(result.t, local_wrap_series(result.state(:,3) - pi), 'LineWidth', 1.2); hold on;
    plot(result.t, local_wrap_series(result.state(:,5) - pi), 'LineWidth', 1.2);
    grid on; ylabel('angle error (rad)'); legend('theta1-pi','theta2-pi');
    subplot(3,1,3);
    plot(result.t, result.u_cmd, 'LineWidth', 1.2); hold on;
    plot(result.t, result.u_actual, 'LineWidth', 1.2);
    yline(double(params.control.max_cart_force_N), '--');
    yline(double(params.control.min_cart_force_N), '--');
    grid on; xlabel('Time (s)'); ylabel('Force (N)'); legend('u cmd','u actual');
    saveas(fig, fullfile(result_dir, 'phase30_state_force.png'));
    close(fig);
end

function ids = local_mode_to_id(modes)
    ids = zeros(numel(modes), 1);
    for i = 1:numel(modes)
        m = char(modes(i));
        switch m
            case 'idle'
                ids(i) = 1;
            case 'trajectory_tracking'
                ids(i) = 2;
            case 'stabilize'
                ids(i) = 3;
            case 'recovery'
                ids(i) = 4;
            case 'failsafe'
                ids(i) = 5;
            otherwise
                ids(i) = 0;
        end
    end
end

function y = local_wrap_series(x)
    y = arrayfun(@local_wrap_to_pi, x);
end

function y = local_wrap_to_pi(a)
    y = atan2(sin(a), cos(a));
end

function value = local_get_nested(s, path, default_value)
    value = default_value;
    cur = s;
    for i = 1:numel(path)
        key = path{i};
        if isstruct(cur) && isfield(cur, key)
            cur = cur.(key);
        else
            return;
        end
    end
    value = cur;
end
