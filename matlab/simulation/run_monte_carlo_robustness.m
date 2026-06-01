% RUN_MONTE_CARLO_ROBUSTNESS Phase 31 robustness and Monte Carlo testing.
%
% This script evaluates the Phase 30 MATLAB hybrid pipeline that inherits the
% Phase 29 discrete TVLQR tracker. The controller is kept nominal, while the
% plant is randomized to test robustness against parameter uncertainty,
% actuator delay, sensor noise, initial state error, friction, and a short cart
% disturbance.
%
% Usage from project root:
%   run('matlab/startup_project.m')
%   run('matlab/simulation/run_monte_carlo_robustness.m')
%
% Optional variables before running:
%   phase31_num_runs = 30;
%   phase31_seed = 31;
%   phase31_profile = 'standard';   % 'standard' or 'stress'
%
% Outputs:
%   results/phase31_monte_carlo/phase31_monte_carlo_results.mat
%   results/phase31_monte_carlo/phase31_monte_carlo_summary.txt
%   results/phase31_monte_carlo/phase31_monte_carlo_metrics.csv
%   docs/PROJECT_STATUS_PHASE31.md

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

if ~exist('phase31_num_runs', 'var') || isempty(phase31_num_runs)
    phase31_num_runs = 30;
end
if ~exist('phase31_seed', 'var') || isempty(phase31_seed)
    phase31_seed = 31;
end
if ~exist('phase31_profile', 'var') || isempty(phase31_profile)
    phase31_profile = 'standard';
end
phase31_profile = char(string(phase31_profile));
if ~ismember(lower(phase31_profile), {'standard', 'stress'})
    error('run_monte_carlo_robustness:BadProfile', ...
          'phase31_profile must be ''standard'' or ''stress''.');
end

nominal_params = load_params();
result_dir = fullfile(project_root, 'results', 'phase31_monte_carlo');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

phase29_file = fullfile(project_root, 'results', 'phase29_tvlqr_tracking', 'tvlqr_tracking_result.mat');
phase30_file = fullfile(project_root, 'results', 'phase30_full_hybrid', 'full_hybrid_result.mat');
if ~exist(phase29_file, 'file')
    error('run_monte_carlo_robustness:MissingPhase29', ...
          'Missing Phase 29 result file: %s. Run run_tvlqr_tracking_test.m first.', phase29_file);
end
if ~exist(phase30_file, 'file')
    fprintf('Phase 30 result file not found. Running Phase 30 full hybrid test first...\n');
    run(fullfile(simulation_dir, 'run_full_hybrid_matlab.m'));
end

S29 = load(phase29_file);
if isfield(S29, 'metrics') && isfield(S29.metrics, 'phase29_pass') && ~logical(S29.metrics.phase29_pass)
    error('run_monte_carlo_robustness:Phase29NotPassed', ...
          'Phase 29 has not passed. Do not run Phase 31 before fixing Phase 29.');
end
if isfield(S29, 'actuator_tracker')
    tracker = S29.actuator_tracker;
elseif isfield(S29, 'ideal_tracker')
    tracker = S29.ideal_tracker;
else
    error('run_monte_carlo_robustness:MissingTracker', ...
          'Phase 29 result has no actuator_tracker or ideal_tracker.');
end

phase29_initial_error = [0; 0; 0.006; 0; -0.005; 0];
if isfield(S29, 'test_cfg') && isfield(S29.test_cfg, 'initial_error')
    phase29_initial_error = double(S29.test_cfg.initial_error(:));
end

base_cfg = local_phase31_base_cfg(tracker, phase29_initial_error);
base_cfg.profile = lower(phase31_profile);
rng(double(phase31_seed), 'twister');

run_metrics = repmat(local_empty_metrics(), double(phase31_num_runs), 1);
case_cfgs = cell(double(phase31_num_runs), 1);
case_results = cell(double(phase31_num_runs), 1);

fprintf('Phase 31 Monte Carlo robustness test\n');
fprintf('phase31_num_runs = %d\n', double(phase31_num_runs));
fprintf('phase31_seed = %d\n', double(phase31_seed));
fprintf('phase31_profile = %s\n', phase31_profile);
fprintf('phase31_tracker_source = %s\n', phase29_file);

for i = 1:double(phase31_num_runs)
    case_cfg = local_random_case_cfg(base_cfg, i, phase31_profile);
    [params_control, params_plant] = local_make_case_params(nominal_params, case_cfg);
    try
        result = local_simulate_phase31(case_cfg, params_control, params_plant);
        metrics = local_phase31_metrics(result, params_control, case_cfg);
        metrics.run_index = i;
        metrics.error_message = "";
    catch ME
        result = struct();
        metrics = local_empty_metrics();
        metrics.run_index = i;
        metrics.success = false;
        metrics.phase31_case_pass = false;
        metrics.failure_reason = "simulation_error";
        metrics.error_message = string(ME.message);
    end
    run_metrics(i) = metrics;
    case_cfgs{i} = case_cfg;
    case_results{i} = local_light_result(result);

    fprintf('run %02d/%02d: success=%d handoff=%d stabilize=%d fail=%s final_angle=%.4g vel=%.4g max_x=%.4g sat=%.4g\n', ...
        i, double(phase31_num_runs), metrics.success, metrics.handoff_success, metrics.stabilize_success, ...
        char(metrics.failure_reason), metrics.final_angle_error_rad, metrics.final_velocity_norm, ...
        metrics.max_abs_x_m, metrics.saturation_fraction);
end

summary = local_build_summary(run_metrics, double(phase31_num_runs));
metrics_table = local_metrics_table(run_metrics);

mat_file = fullfile(result_dir, 'phase31_monte_carlo_results.mat');
csv_file = fullfile(result_dir, 'phase31_monte_carlo_metrics.csv');
summary_file = fullfile(result_dir, 'phase31_monte_carlo_summary.txt');
status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE31.md');

save(mat_file, 'run_metrics', 'case_cfgs', 'case_results', 'summary', 'base_cfg', ...
     'phase29_file', 'phase30_file', 'phase31_num_runs', 'phase31_seed', 'phase31_profile');
writetable(metrics_table, csv_file);
local_write_summary(summary_file, summary, mat_file, csv_file, phase29_file, phase30_file, phase31_profile);
local_write_status(status_file, summary, mat_file, csv_file, phase29_file, phase30_file, phase31_profile);

try
    plot_robustness_summary(mat_file);
catch ME
    warning('run_monte_carlo_robustness:PlotFailed', 'Plot summary failed: %s', ME.message);
end

fprintf('phase31_num_runs = %d\n', summary.num_runs);
fprintf('phase31_profile = %s\n', phase31_profile);
fprintf('phase31_success_count = %d\n', summary.success_count);
fprintf('phase31_success_rate = %.6g\n', summary.success_rate);
fprintf('phase31_handoff_success_rate = %.6g\n', summary.handoff_success_rate);
fprintf('phase31_stabilize_success_rate = %.6g\n', summary.stabilize_success_rate);
fprintf('phase31_fail_count = %d\n', summary.fail_count);
fprintf('phase31_mean_saturation_fraction = %.6g\n', summary.mean_saturation_fraction);
fprintf('phase31_worst_final_angle_error_rad = %.6g\n', summary.worst_final_angle_error_rad);
fprintf('phase31_worst_final_velocity_norm = %.6g\n', summary.worst_final_velocity_norm);
fprintf('result_file = %s\n', mat_file);
if summary.phase31_pass
    fprintf('Phase 31 run_monte_carlo_robustness: PASS\n');
else
    fprintf('Phase 31 run_monte_carlo_robustness: REVIEW_REQUIRED\n');
end

function cfg = local_phase31_base_cfg(tracker, phase29_initial_error)
    cfg = struct();
    cfg.tracker = tracker;
    cfg.target_mode = 'up_up';
    cfg.dt = median(diff(double(tracker.t(:))));
    cfg.post_handoff_hold_time_s = 0.50;
    cfg.t_final = double(tracker.t(end)) + cfg.post_handoff_hold_time_s;
    cfg.initial_state_nominal = double(tracker.x_ref(:, 1)) + double(phase29_initial_error(:));
    cfg.integration_substeps_per_interval = local_get_nested(tracker, {'config','integration_substeps_per_interval'}, 7);
    cfg.actuator_tau_motor_s = 0.012;
    cfg.actuator_rate_limit_N_per_s = 500.0;
    cfg.sensor_noise_enabled = true;
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
    cfg.max_abs_velocity_fail = 20.0;
    cfg.disturbance_enabled = true;
    cfg.disturbance_force_N = 0.0;
    cfg.disturbance_start_s = 6.0;
    cfg.disturbance_duration_s = 0.20;
    cfg.case_label = 'nominal';
end

function case_cfg = local_random_case_cfg(base_cfg, run_index, profile)
    case_cfg = base_cfg;
    case_cfg.run_index = run_index;
    profile = lower(char(string(profile)));
    case_cfg.profile = profile;
    case_cfg.case_label = sprintf('%s_mc_%03d', profile, run_index);

    if strcmp(profile, 'stress')
        % Stress profile: intentionally broad uncertainty range. This is useful
        % to discover failure modes, but it is not the default pass gate.
        mass_range = [0.90, 1.10];
        length_range = [0.95, 1.05];
        damping_range = [0.50, 1.80];
        friction_range = [0.0, 2.0];
        tau_range = [0.006, 0.040];
        rate_range = [250.0, 650.0];
        noise_range = [0.5, 3.0];
        init_range = [0.025, 0.025, 0.018, 0.050, 0.018, 0.050];
        disturbance_probability = 0.70;
        disturbance_force_range = [-2.0, 2.0];
        disturbance_duration_range = [0.08, 0.35];
    else
        % Standard profile: moderate uncertainty for the Phase 31 pass gate.
        % It checks practical robustness without asking the fixed Phase 29
        % trajectory to solve a new global swing-up problem under large model
        % mismatch. Use the stress profile separately for limitation analysis.
        mass_range = [0.97, 1.03];
        length_range = [0.985, 1.015];
        damping_range = [0.80, 1.25];
        friction_range = [0.0, 0.75];
        tau_range = [0.008, 0.020];
        rate_range = [400.0, 650.0];
        noise_range = [0.5, 1.5];
        init_range = [0.010, 0.010, 0.006, 0.020, 0.006, 0.020];
        disturbance_probability = 0.40;
        disturbance_force_range = [-0.6, 0.6];
        disturbance_duration_range = [0.05, 0.15];
    end

    case_cfg.mass_scale_cart = local_uniform(mass_range(1), mass_range(2));
    case_cfg.mass_scale_link1 = local_uniform(mass_range(1), mass_range(2));
    case_cfg.mass_scale_link2 = local_uniform(mass_range(1), mass_range(2));
    case_cfg.length_scale_link1 = local_uniform(length_range(1), length_range(2));
    case_cfg.length_scale_link2 = local_uniform(length_range(1), length_range(2));
    case_cfg.damping_scale = local_uniform(damping_range(1), damping_range(2));
    case_cfg.extra_friction_scale = local_uniform(friction_range(1), friction_range(2));
    case_cfg.actuator_tau_motor_s = local_uniform(tau_range(1), tau_range(2));
    case_cfg.actuator_rate_limit_N_per_s = local_uniform(rate_range(1), rate_range(2));
    case_cfg.noise_scale = local_uniform(noise_range(1), noise_range(2));
    case_cfg.sensor_noise_std = base_cfg.sensor_noise_std * case_cfg.noise_scale;

    init = base_cfg.initial_state_nominal;
    init = init + [local_uniform(-init_range(1), init_range(1)); ...
                   local_uniform(-init_range(2), init_range(2)); ...
                   local_uniform(-init_range(3), init_range(3)); ...
                   local_uniform(-init_range(4), init_range(4)); ...
                   local_uniform(-init_range(5), init_range(5)); ...
                   local_uniform(-init_range(6), init_range(6))];
    case_cfg.initial_state = init;

    if rand() < disturbance_probability
        case_cfg.disturbance_enabled = true;
        case_cfg.disturbance_force_N = local_uniform(disturbance_force_range(1), disturbance_force_range(2));
        case_cfg.disturbance_start_s = local_uniform(3.0, max(3.1, double(base_cfg.tracker.t(end)) - 1.0));
        case_cfg.disturbance_duration_s = local_uniform(disturbance_duration_range(1), disturbance_duration_range(2));
    else
        case_cfg.disturbance_enabled = false;
        case_cfg.disturbance_force_N = 0.0;
        case_cfg.disturbance_start_s = NaN;
        case_cfg.disturbance_duration_s = 0.0;
    end

    case_cfg.sensor_noise_seed = 31000 + run_index;
end

function [params_control, params_plant] = local_make_case_params(nominal_params, case_cfg)
    params_control = nominal_params;
    params_control.realism.rail_limit_enabled = false;
    params_control.realism.friction_enabled = false;
    params_control.realism.sensor_noise_enabled = true;
    params_control.realism.estimator_enabled = false;
    params_control.realism.use_measurement_for_control = true;
    params_control.realism.use_estimator_for_control = false;
    params_control.realism.actuator_enabled = true;
    params_control.actuator.tau_motor_s = case_cfg.actuator_tau_motor_s;
    params_control.actuator.rate_limit_N_per_s = case_cfg.actuator_rate_limit_N_per_s;
    params_control.actuator.dead_zone_N = 0.0;
    params_control.actuator.min_force_N = double(nominal_params.control.min_cart_force_N);
    params_control.actuator.max_force_N = double(nominal_params.control.max_cart_force_N);

    params_plant = params_control;
    p = params_plant.physical_parameters;
    p.cart_mass_kg = double(p.cart_mass_kg) * case_cfg.mass_scale_cart;
    p.link1_mass_kg = double(p.link1_mass_kg) * case_cfg.mass_scale_link1;
    p.link2_mass_kg = double(p.link2_mass_kg) * case_cfg.mass_scale_link2;
    p.link1_length_m = double(p.link1_length_m) * case_cfg.length_scale_link1;
    p.link2_length_m = double(p.link2_length_m) * case_cfg.length_scale_link2;
    p.cart_damping_n_s_per_m = double(p.cart_damping_n_s_per_m) * case_cfg.damping_scale;
    p.link1_damping_n_m_s_per_rad = double(p.link1_damping_n_m_s_per_rad) * case_cfg.damping_scale;
    p.link2_damping_n_m_s_per_rad = double(p.link2_damping_n_m_s_per_rad) * case_cfg.damping_scale;
    params_plant.physical_parameters = p;

    params_plant.realism.friction_enabled = true;
    if isfield(params_plant, 'friction')
        params_plant.friction.cart_viscous_extra_N_s_per_m = double(params_plant.friction.cart_viscous_extra_N_s_per_m) * case_cfg.extra_friction_scale;
        params_plant.friction.link1_viscous_extra_N_m_s_per_rad = double(params_plant.friction.link1_viscous_extra_N_m_s_per_rad) * case_cfg.extra_friction_scale;
        params_plant.friction.link2_viscous_extra_N_m_s_per_rad = double(params_plant.friction.link2_viscous_extra_N_m_s_per_rad) * case_cfg.extra_friction_scale;
        params_plant.friction.cart_coulomb_N = double(params_plant.friction.cart_coulomb_N) * case_cfg.extra_friction_scale;
        params_plant.friction.link1_coulomb_N_m = double(params_plant.friction.link1_coulomb_N_m) * case_cfg.extra_friction_scale;
        params_plant.friction.link2_coulomb_N_m = double(params_plant.friction.link2_coulomb_N_m) * case_cfg.extra_friction_scale;
    end
end

function result = local_simulate_phase31(cfg, params_control, params_plant)
    dt = double(cfg.dt);
    t = (0:dt:double(cfg.t_final)).';
    n = numel(t);
    X = zeros(n, 6);
    X(1, :) = double(cfg.initial_state(:)).';
    X_meas = zeros(n, 6);
    U_cmd = zeros(n, 1);
    U_raw = zeros(n, 1);
    U_actual = zeros(n, 1);
    F_rate = zeros(n, 1);
    F_dist = zeros(n, 1);
    mode = strings(n, 1);
    handoff_flag = false(n, 1);
    x_ref = nan(n, 6);
    tracking_error = nan(n, 6);
    target_error = nan(n, 6);
    final_angle_error = nan(n, 1);
    velocity_norm = nan(n, 1);
    fail_reason = strings(n, 1);

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
        X_meas(k, :) = x_control.';

        sim_config = cfg;
        sim_config.hybrid_reset = (k == 1);
        sim_config.current_F_actual = F_actual;
        [ctrl, hybrid_state] = hybrid_controller_matlab(tk, x_control, params_control, sim_config, hybrid_state);

        F_external_cart = local_disturbance_force(tk, cfg);
        [x_next, F_next, act_info] = local_step_plant_with_actuator( ...
            X(k, :).', F_actual, ctrl.u_cmd, F_external_cart, dt, cfg.integration_substeps_per_interval, params_plant);
        X(k + 1, :) = x_next.';

        U_cmd(k) = ctrl.u_cmd;
        U_raw(k) = ctrl.u_raw;
        U_actual(k) = F_next;
        F_rate(k) = act_info.F_dot_limited;
        F_dist(k) = F_external_cart;
        mode(k) = string(ctrl.hybrid_mode);
        handoff_flag(k) = ctrl.handoff_flag;
        x_ref(k, :) = double(ctrl.x_ref(:)).';
        tracking_error(k, :) = double(ctrl.tracking_error_state(:)).';
        target_error(k, :) = double(ctrl.target_error_state(:)).';
        final_angle_error(k) = ctrl.final_angle_error_rad;
        velocity_norm(k) = ctrl.velocity_norm;
        fail_reason(k) = string(ctrl.fail_reason);

        F_actual = F_next;
        if any(~isfinite(X(k + 1, :)))
            error('run_monte_carlo_robustness:NonFiniteState', ...
                  'State became nonfinite at run %d, k=%d, t=%.6f.', cfg.run_index, k, tk);
        end
    end

    sim_config = cfg;
    sim_config.hybrid_reset = false;
    sim_config.current_F_actual = F_actual;
    x_control = local_controller_measurement(X(end, :).', cfg);
    X_meas(end, :) = x_control.';
    [ctrl, hybrid_state] = hybrid_controller_matlab(t(end), x_control, params_control, sim_config, hybrid_state);

    U_cmd(end) = ctrl.u_cmd;
    U_raw(end) = ctrl.u_raw;
    U_actual(end) = F_actual;
    F_rate(end) = 0.0;
    F_dist(end) = local_disturbance_force(t(end), cfg);
    mode(end) = string(ctrl.hybrid_mode);
    handoff_flag(end) = ctrl.handoff_flag;
    x_ref(end, :) = double(ctrl.x_ref(:)).';
    tracking_error(end, :) = double(ctrl.tracking_error_state(:)).';
    target_error(end, :) = double(ctrl.target_error_state(:)).';
    final_angle_error(end) = ctrl.final_angle_error_rad;
    velocity_norm(end) = ctrl.velocity_norm;
    fail_reason(end) = string(ctrl.fail_reason);

    result = struct();
    result.t = t;
    result.state = X;
    result.measurement = X_meas;
    result.u_cmd = U_cmd;
    result.u_raw = U_raw;
    result.u_actual = U_actual;
    result.actuator_rate = F_rate;
    result.disturbance_force = F_dist;
    result.mode = mode;
    result.handoff_flag = handoff_flag;
    result.x_ref = x_ref;
    result.tracking_error = tracking_error;
    result.target_error = target_error;
    result.final_angle_error = final_angle_error;
    result.velocity_norm = velocity_norm;
    result.fail_reason = fail_reason;
    result.controller_state = hybrid_state;
end

function x_meas = local_controller_measurement(x_true, cfg)
    x_meas = double(x_true(:));
    if isfield(cfg, 'sensor_noise_enabled') && logical(cfg.sensor_noise_enabled)
        x_meas = x_meas + double(cfg.sensor_noise_std(:)) .* randn(6, 1);
    end
end

function [x_next, F_actual, act_info] = local_step_plant_with_actuator(x, F_actual, F_cmd, F_external_cart, dt, substeps, params)
    x_next = double(x(:));
    n_sub = max(1, round(double(substeps)));
    h = double(dt) / n_sub;
    act_info = struct('F_dot_limited', 0.0);
    ext = [double(F_external_cart); 0; 0];
    for j = 1:n_sub
        [F_actual, act_info] = dip_actuator(F_actual, F_cmd, h, params);
        x_next = local_rk4_zoh_step(x_next, F_actual, ext, h, params);
    end
end

function x_next = local_rk4_zoh_step(x, u, external_generalized_force, dt, params)
    x_next = double(x(:));
    f = @(z) dip_dynamics_nonlinear(z, u, params, external_generalized_force);
    k1 = f(x_next);
    k2 = f(x_next + 0.5*dt*k1);
    k3 = f(x_next + 0.5*dt*k2);
    k4 = f(x_next + dt*k3);
    x_next = x_next + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
end

function F = local_disturbance_force(t, cfg)
    if ~isfield(cfg, 'disturbance_enabled') || ~logical(cfg.disturbance_enabled)
        F = 0.0;
        return;
    end
    t0 = double(cfg.disturbance_start_s);
    t1 = t0 + double(cfg.disturbance_duration_s);
    if double(t) >= t0 && double(t) <= t1
        F = double(cfg.disturbance_force_N);
    else
        F = 0.0;
    end
end

function metrics = local_phase31_metrics(result, params, cfg)
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

    metrics = local_empty_metrics();
    metrics.run_index = cfg.run_index;
    metrics.handoff_time_s = handoff_time;
    metrics.handoff_success = stabilize_seen;
    metrics.stabilize_success = stabilize_seen && final_angle_error <= 0.12 && final_velocity_norm <= 0.45 && ...
                                final_window_angle <= 0.18 && final_window_velocity <= 0.70;
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
    metrics.saturation_pass = metrics.saturation_fraction <= 0.08;
    metrics.finite_pass = all(isfinite(result.state(:))) && all(isfinite(result.u_cmd(:))) && all(isfinite(result.u_actual(:)));
    metrics.success = metrics.finite_pass && metrics.handoff_success && metrics.stabilize_success && ...
                      metrics.rail_pass && metrics.force_pass && metrics.saturation_pass && ~metrics.failsafe_triggered;
    metrics.phase31_case_pass = metrics.success;
    metrics.failure_reason = string(local_classify_failure(metrics));
    metrics.error_message = "";
    metrics.mass_scale_cart = cfg.mass_scale_cart;
    metrics.mass_scale_link1 = cfg.mass_scale_link1;
    metrics.mass_scale_link2 = cfg.mass_scale_link2;
    metrics.length_scale_link1 = cfg.length_scale_link1;
    metrics.length_scale_link2 = cfg.length_scale_link2;
    metrics.damping_scale = cfg.damping_scale;
    metrics.extra_friction_scale = cfg.extra_friction_scale;
    metrics.actuator_tau_motor_s = cfg.actuator_tau_motor_s;
    metrics.actuator_rate_limit_N_per_s = cfg.actuator_rate_limit_N_per_s;
    metrics.noise_scale = cfg.noise_scale;
    metrics.disturbance_force_N = cfg.disturbance_force_N;
    metrics.disturbance_start_s = cfg.disturbance_start_s;
    metrics.disturbance_duration_s = cfg.disturbance_duration_s;
    metrics.profile = string(cfg.profile);
end

function metrics = local_empty_metrics()
    metrics = struct();
    metrics.run_index = NaN;
    metrics.success = false;
    metrics.phase31_case_pass = false;
    metrics.handoff_success = false;
    metrics.stabilize_success = false;
    metrics.recovery_triggered = false;
    metrics.failsafe_triggered = false;
    metrics.finite_pass = false;
    metrics.rail_pass = false;
    metrics.force_pass = false;
    metrics.saturation_pass = false;
    metrics.handoff_time_s = NaN;
    metrics.final_angle_error_rad = NaN;
    metrics.final_velocity_norm = NaN;
    metrics.final_window_max_angle_error_rad = NaN;
    metrics.final_window_max_velocity_norm = NaN;
    metrics.max_abs_x_m = NaN;
    metrics.max_abs_u_cmd_N = NaN;
    metrics.max_abs_u_actual_N = NaN;
    metrics.saturation_fraction = NaN;
    metrics.failure_reason = "not_run";
    metrics.error_message = "";
    metrics.mass_scale_cart = NaN;
    metrics.mass_scale_link1 = NaN;
    metrics.mass_scale_link2 = NaN;
    metrics.length_scale_link1 = NaN;
    metrics.length_scale_link2 = NaN;
    metrics.damping_scale = NaN;
    metrics.extra_friction_scale = NaN;
    metrics.actuator_tau_motor_s = NaN;
    metrics.actuator_rate_limit_N_per_s = NaN;
    metrics.noise_scale = NaN;
    metrics.disturbance_force_N = NaN;
    metrics.disturbance_start_s = NaN;
    metrics.disturbance_duration_s = NaN;
    metrics.profile = "unknown";
end

function reason = local_classify_failure(metrics)
    if metrics.success
        reason = 'success';
    elseif ~metrics.finite_pass
        reason = 'nonfinite_or_simulation_error';
    elseif ~metrics.rail_pass
        reason = 'rail_violation';
    elseif ~metrics.force_pass
        reason = 'force_limit_violation';
    elseif ~metrics.saturation_pass
        reason = 'saturation_fraction_high';
    elseif metrics.failsafe_triggered
        reason = 'failsafe_triggered';
    elseif ~metrics.handoff_success
        reason = 'tracking_not_in_handoff_gate';
    elseif ~metrics.stabilize_success
        reason = 'not_stabilized_after_handoff';
    else
        reason = 'unknown_failure';
    end
end

function summary = local_build_summary(run_metrics, n_runs)
    success = [run_metrics.success];
    handoff = [run_metrics.handoff_success];
    stabilize = [run_metrics.stabilize_success];
    failsafe = [run_metrics.failsafe_triggered];
    sat = [run_metrics.saturation_fraction];
    angle = [run_metrics.final_angle_error_rad];
    vel = [run_metrics.final_velocity_norm];
    max_x = [run_metrics.max_abs_x_m];

    summary = struct();
    summary.num_runs = n_runs;
    summary.success_count = sum(success);
    summary.fail_count = n_runs - summary.success_count;
    summary.success_rate = summary.success_count / max(1, n_runs);
    summary.handoff_success_rate = sum(handoff) / max(1, n_runs);
    summary.stabilize_success_rate = sum(stabilize) / max(1, n_runs);
    summary.failsafe_count = sum(failsafe);
    summary.failsafe_rate = summary.failsafe_count / max(1, n_runs);
    summary.mean_saturation_fraction = local_finite_mean(sat);
    summary.max_saturation_fraction = local_finite_max(sat);
    summary.worst_final_angle_error_rad = local_finite_max(angle);
    summary.worst_final_velocity_norm = local_finite_max(vel);
    summary.max_abs_x_m = local_finite_max(max_x);
    summary.phase31_pass = n_runs >= 30 && summary.success_rate >= 0.70 && ...
                           summary.handoff_success_rate >= 0.80 && ...
                           summary.mean_saturation_fraction <= 0.08;
    summary.failure_reasons = local_failure_reason_counts(run_metrics);
end

function counts = local_failure_reason_counts(run_metrics)
    reasons = string([run_metrics.failure_reason]);
    unique_reasons = unique(reasons);
    counts = struct();
    for i = 1:numel(unique_reasons)
        name = char(unique_reasons(i));
        safe_name = matlab.lang.makeValidName(name);
        counts.(safe_name) = sum(reasons == unique_reasons(i));
    end
end


function y = local_finite_mean(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = mean(x);
    end
end

function y = local_finite_max(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = max(x);
    end
end

function T = local_metrics_table(run_metrics)
    T = struct2table(run_metrics);
end

function light = local_light_result(result)
    light = struct();
    if isempty(fieldnames(result))
        return;
    end
    light.t = result.t;
    light.state = result.state;
    light.u_cmd = result.u_cmd;
    light.u_actual = result.u_actual;
    light.disturbance_force = result.disturbance_force;
    light.mode = result.mode;
    light.handoff_flag = result.handoff_flag;
end

function local_write_summary(path, summary, mat_file, csv_file, phase29_file, phase30_file, phase31_profile)
    fid = fopen(path, 'w');
    if fid < 0
        error('run_monte_carlo_robustness:CannotWriteSummary', 'Cannot write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'Phase 31 Monte Carlo robustness summary\n');
    fprintf(fid, 'phase29_source = %s\n', phase29_file);
    fprintf(fid, 'phase30_source = %s\n', phase30_file);
    fprintf(fid, 'profile = %s\n', phase31_profile);
    fprintf(fid, 'result_file = %s\n', mat_file);
    fprintf(fid, 'csv_file = %s\n', csv_file);
    fprintf(fid, 'num_runs = %d\n', summary.num_runs);
    fprintf(fid, 'success_count = %d\n', summary.success_count);
    fprintf(fid, 'fail_count = %d\n', summary.fail_count);
    fprintf(fid, 'success_rate = %.12g\n', summary.success_rate);
    fprintf(fid, 'handoff_success_rate = %.12g\n', summary.handoff_success_rate);
    fprintf(fid, 'stabilize_success_rate = %.12g\n', summary.stabilize_success_rate);
    fprintf(fid, 'failsafe_count = %d\n', summary.failsafe_count);
    fprintf(fid, 'failsafe_rate = %.12g\n', summary.failsafe_rate);
    fprintf(fid, 'mean_saturation_fraction = %.12g\n', summary.mean_saturation_fraction);
    fprintf(fid, 'max_saturation_fraction = %.12g\n', summary.max_saturation_fraction);
    fprintf(fid, 'worst_final_angle_error_rad = %.12g\n', summary.worst_final_angle_error_rad);
    fprintf(fid, 'worst_final_velocity_norm = %.12g\n', summary.worst_final_velocity_norm);
    fprintf(fid, 'max_abs_x_m = %.12g\n', summary.max_abs_x_m);
    fprintf(fid, 'phase31_pass = %d\n', summary.phase31_pass);
    fprintf(fid, '\nFailure reason counts:\n');
    names = fieldnames(summary.failure_reasons);
    for i = 1:numel(names)
        fprintf(fid, '%s = %d\n', names{i}, summary.failure_reasons.(names{i}));
    end
end

function local_write_status(path, summary, mat_file, csv_file, phase29_file, phase30_file, phase31_profile)
    fid = fopen(path, 'w');
    if fid < 0
        error('run_monte_carlo_robustness:CannotWriteStatus', 'Cannot write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# PROJECT STATUS - PHASE 31\n\n');
    fprintf(fid, '## Phase name\n');
    fprintf(fid, 'Phase 31 - Robustness and Monte Carlo testing on MATLAB.\n\n');
    fprintf(fid, '## Profile\n');
    fprintf(fid, 'Current Monte Carlo profile: `%s`. Use `standard` for the pass gate and `stress` for limitation discovery.\n\n', phase31_profile);
    fprintf(fid, '## Goal\n');
    fprintf(fid, 'Evaluate the Phase 30 hybrid MATLAB pipeline inherited from Phase 29 under non-ideal conditions: parameter uncertainty, actuator delay variation, sensor noise, friction, initial state error and short cart disturbance.\n\n');
    fprintf(fid, '## Dependency\n');
    fprintf(fid, '- Phase 29 TVLQR tracking source: `%s`\n', phase29_file);
    fprintf(fid, '- Phase 30 hybrid source/result: `%s`\n\n', phase30_file);
    fprintf(fid, '## Files added\n');
    fprintf(fid, '- `matlab/simulation/run_monte_carlo_robustness.m`\n');
    fprintf(fid, '- `matlab/visualization/plot_robustness_summary.m`\n');
    fprintf(fid, '- `docs/PROJECT_STATUS_PHASE31.md`\n\n');
    fprintf(fid, '## Randomized factors\n');
    fprintf(fid, '- cart, link 1 and link 2 mass scale\n');
    fprintf(fid, '- link 1 and link 2 length scale\n');
    fprintf(fid, '- damping and additional friction scale\n');
    fprintf(fid, '- actuator time constant and rate limit\n');
    fprintf(fid, '- sensor noise scale\n');
    fprintf(fid, '- initial state error around the Phase 29 initial condition\n');
    fprintf(fid, '- optional short cart disturbance during tracking\n\n');
    fprintf(fid, '## Test command\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'run(''matlab/startup_project.m'')\n');
    fprintf(fid, 'run(''matlab/simulation/run_monte_carlo_robustness.m'')\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, 'Optional quick debug run:\n\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'phase31_profile = ''standard'';\n');
    fprintf(fid, 'phase31_num_runs = 5;\n');
    fprintf(fid, 'run(''matlab/startup_project.m'')\n');
    fprintf(fid, 'run(''matlab/simulation/run_monte_carlo_robustness.m'')\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, '## Latest result\n');
    fprintf(fid, '- result file: `%s`\n', mat_file);
    fprintf(fid, '- metric csv: `%s`\n', csv_file);
    fprintf(fid, '- profile: `%s`\n', phase31_profile);
    fprintf(fid, '- num_runs: `%d`\n', summary.num_runs);
    fprintf(fid, '- success_count: `%d`\n', summary.success_count);
    fprintf(fid, '- fail_count: `%d`\n', summary.fail_count);
    fprintf(fid, '- success_rate: `%.12g`\n', summary.success_rate);
    fprintf(fid, '- handoff_success_rate: `%.12g`\n', summary.handoff_success_rate);
    fprintf(fid, '- stabilize_success_rate: `%.12g`\n', summary.stabilize_success_rate);
    fprintf(fid, '- failsafe_count: `%d`\n', summary.failsafe_count);
    fprintf(fid, '- mean_saturation_fraction: `%.12g`\n', summary.mean_saturation_fraction);
    fprintf(fid, '- worst_final_angle_error_rad: `%.12g`\n', summary.worst_final_angle_error_rad);
    fprintf(fid, '- worst_final_velocity_norm: `%.12g`\n', summary.worst_final_velocity_norm);
    fprintf(fid, '- phase31_pass: `%d`\n\n', summary.phase31_pass);
    fprintf(fid, '## Failure classification\n');
    names = fieldnames(summary.failure_reasons);
    for i = 1:numel(names)
        fprintf(fid, '- `%s`: `%d`\n', names{i}, summary.failure_reasons.(names{i}));
    end
    fprintf(fid, '\n## Pass condition\n');
    fprintf(fid, 'Phase 31 standard profile passes when at least 30 Monte Carlo runs are executed, success rate is at least 70 percent, handoff success rate is at least 80 percent, and mean saturation fraction is not higher than 8 percent. The stress profile is allowed to return REVIEW_REQUIRED because its purpose is to expose the current controller limit. Failures must be kept and classified.\n\n');
    fprintf(fid, '## Notes\n');
    fprintf(fid, 'The controller is kept nominal while the plant is randomized. This makes the robustness test stricter than retuning the controller for every sampled plant. FIX1 adds two profiles: standard for realistic pass gate and stress for limitation analysis. It also prioritizes rail, force, saturation and failsafe in failure classification before generic handoff failure.\n');
end

function value = local_uniform(a, b)
    value = double(a) + (double(b) - double(a)) * rand();
end

function value = local_get_nested(s, fields, default_value)
    value = default_value;
    current = s;
    for i = 1:numel(fields)
        f = fields{i};
        if isstruct(current) && isfield(current, f)
            current = current.(f);
        else
            return;
        end
    end
    if ~isempty(current)
        value = current;
    end
end

function y = local_wrap_to_pi(a)
    y = atan2(sin(a), cos(a));
end
