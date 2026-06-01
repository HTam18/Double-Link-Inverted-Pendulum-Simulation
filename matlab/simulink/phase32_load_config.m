function cfg = phase32_load_config(project_root)
% PHASE32_LOAD_CONFIG Load the validated Phase 29/30 configuration for Simulink.
%
% This helper is used by Phase 32 Simulink scripts and by the interpreted
% MATLAB Function block generated in Double_Link_Pendulum_Main.slx.
%
% Pipeline source:
%   Phase 29: trajectory ready + discrete TVLQR tracking result
%   Phase 30: hybrid tracking -> handoff -> terminal hold pass settings
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.

    if nargin < 1 || isempty(project_root)
        this_file = mfilename('fullpath');
        simulink_dir = fileparts(this_file);
        matlab_root = fileparts(simulink_dir);
        project_root = fileparts(matlab_root);
    end

    matlab_root = fullfile(project_root, 'matlab');
    if exist(fullfile(matlab_root, 'startup_project.m'), 'file')
        run(fullfile(matlab_root, 'startup_project.m'));
    end

    params = load_params();
    phase29_file = fullfile(project_root, 'results', 'phase29_tvlqr_tracking', 'tvlqr_tracking_result.mat');
    if ~exist(phase29_file, 'file')
        error('phase32_load_config:MissingPhase29Result', ...
            'Missing Phase 29 tracking result: %s. Run matlab/simulation/run_tvlqr_tracking_test.m first.', phase29_file);
    end

    S = load(phase29_file);
    if isfield(S, 'metrics') && isfield(S.metrics, 'phase29_pass') && ~logical(S.metrics.phase29_pass)
        error('phase32_load_config:Phase29NotPassed', ...
            'Phase 29 metrics.phase29_pass is false. Do not build Phase 32 before fixing Phase 29.');
    end
    if isfield(S, 'actuator_tracker')
        tracker = S.actuator_tracker;
    elseif isfield(S, 'ideal_tracker')
        tracker = S.ideal_tracker;
    else
        error('phase32_load_config:MissingTracker', ...
            'Phase 29 result has no actuator_tracker or ideal_tracker.');
    end

    phase29_initial_error = [0; 0; 0.006; 0; -0.005; 0];
    if isfield(S, 'test_cfg') && isfield(S.test_cfg, 'initial_error')
        phase29_initial_error = double(S.test_cfg.initial_error(:));
    end

    dt = median(diff(double(tracker.t(:))));

    cfg = struct();
    cfg.project_root = project_root;
    cfg.matlab_root = matlab_root;
    cfg.phase29_file = phase29_file;
    cfg.tracker = tracker;
    cfg.params = params;
    cfg.target_mode = 'up_up';
    cfg.dt = dt;
    cfg.post_handoff_hold_time_s = 0.50;
    cfg.t_final = double(tracker.t(end)) + cfg.post_handoff_hold_time_s;
    cfg.initial_state = double(tracker.x_ref(:, 1)) + phase29_initial_error;
    cfg.integration_substeps_per_interval = local_get_nested(tracker, {'config','integration_substeps_per_interval'}, 7);
    cfg.actuator_tau_motor_s = 0.012;
    cfg.actuator_rate_limit_N_per_s = 500.0;
    cfg.sensor_noise_enabled = false;
    cfg.sensor_noise_seed = 32;
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
    cfg.log_width = 34;
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
