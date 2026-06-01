function out = lqr_stabilize_controller(t, state, params, sim_config)
% LQR_STABILIZE_CONTROLLER Local LQR stabilize controller for Phase 22.
%
% Purpose:
%   Port the existing local LQR stabilize logic into the MATLAB pipeline.
%   This controller is only valid near a selected balance target. It is not a
%   swing-up controller.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.
%
% Control law:
%   error = [x, x_dot, wrap(theta1-target1), theta1_dot,
%            wrap(theta2-target2), theta2_dot]
%   u_raw = -K * error
%   u_cmd = saturate(u_raw, min_cart_force_N, max_cart_force_N)
%
% Usage with run_simulation:
%   cfg.target_mode = 'up_up';
%   cfg.controller = @lqr_stabilize_controller;
%   result = run_simulation(cfg, params);

    %#ok<NASGU> % t is kept for the standard controller interface.
    if nargin < 4 || isempty(sim_config)
        sim_config = struct();
    end
    if nargin < 3 || isempty(params)
        params = load_params();
    end

    x = double(state(:));
    if numel(x) ~= 6
        error('lqr_stabilize_controller:InvalidState', 'state must have 6 elements.');
    end

    target_mode = local_get_target_mode(sim_config);
    target_state = local_get_target_state(params, target_mode);
    K = local_load_lqr_gain(params, sim_config, target_mode);

    error_state = x - target_state;
    error_state(3) = local_wrap_to_pi(x(3) - target_state(3));
    error_state(5) = local_wrap_to_pi(x(5) - target_state(5));

    u_raw = -K * error_state;
    u_cmd = local_saturate_force(u_raw, params);

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = double(u_raw);
    out.mode = sprintf('lqr_%s', target_mode);
    out.target_mode = target_mode;
    out.target_state = target_state;
    out.error_state = error_state;
    out.measurement = x;
    out.x_hat = x;
end

function target_mode = local_get_target_mode(sim_config)
    if isfield(sim_config, 'target_mode') && ~isempty(sim_config.target_mode)
        target_mode = char(sim_config.target_mode);
    elseif isfield(sim_config, 'lqr_target_mode') && ~isempty(sim_config.lqr_target_mode)
        target_mode = char(sim_config.lqr_target_mode);
    else
        target_mode = 'up_up';
    end
end

function target_state = local_get_target_state(params, target_mode)
    if ~isfield(params, 'target_modes') || ~isfield(params.target_modes, target_mode)
        error('lqr_stabilize_controller:UnknownTargetMode', ...
              'Target mode "%s" is not found in params.target_modes.', target_mode);
    end

    target = params.target_modes.(target_mode);
    target_state = zeros(6, 1);
    target_state(1) = 0.0;
    target_state(2) = 0.0;
    target_state(3) = double(target.theta1_target_rad);
    target_state(4) = 0.0;
    target_state(5) = double(target.theta2_target_rad);
    target_state(6) = 0.0;
end

function K = local_load_lqr_gain(params, sim_config, target_mode)
    gain_path = local_get_gain_path(params, sim_config);
    if ~isfile(gain_path)
        error('lqr_stabilize_controller:GainFileNotFound', ...
              ['LQR gain file not found: %s\n' ...
               'Run matlab/design_lqr_all_targets.m before Phase 22 test.'], gain_path);
    end

    raw_text = fileread(gain_path);
    data = jsondecode(raw_text);

    if ~isfield(data, 'controllers') || ~isfield(data.controllers, target_mode)
        error('lqr_stabilize_controller:MissingGain', ...
              'No LQR gain found for target mode "%s" in %s.', target_mode, gain_path);
    end

    item = data.controllers.(target_mode);
    if ~isfield(item, 'K')
        error('lqr_stabilize_controller:MissingK', ...
              'Controller "%s" does not include field K in %s.', target_mode, gain_path);
    end

    K = local_json_matrix(item.K, [1, 6], sprintf('%s.K', target_mode));
end

function gain_path = local_get_gain_path(params, sim_config)
    if isfield(sim_config, 'lqr_gains_path') && ~isempty(sim_config.lqr_gains_path)
        gain_path = char(sim_config.lqr_gains_path);
        if ~isfile(gain_path)
            gain_path = fullfile(pwd, gain_path);
        end
        return;
    end

    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        project_root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        control_dir = fileparts(this_file);
        matlab_root = fileparts(control_dir);
        project_root = fileparts(matlab_root);
    end

    gain_path = fullfile(project_root, 'shared', 'lqr', 'lqr_gains.json');
end

function M = local_json_matrix(value, expected_size, name)
    if isnumeric(value)
        M = double(value);
    elseif iscell(value)
        if isempty(value)
            M = [];
        elseif all(cellfun(@isnumeric, value))
            row_count = numel(value);
            first_row = value{1};
            col_count = numel(first_row);
            M = zeros(row_count, col_count);
            for r = 1:row_count
                row = value{r};
                if numel(row) ~= col_count
                    error('lqr_stabilize_controller:InvalidJsonMatrix', ...
                          'Invalid JSON matrix %s: inconsistent row length.', name);
                end
                M(r, :) = reshape(double(row), 1, []);
            end
        else
            M = cell2mat(value);
        end
    else
        error('lqr_stabilize_controller:InvalidJsonMatrix', ...
              'Invalid JSON matrix %s: unsupported type %s.', name, class(value));
    end

    if isvector(M) && numel(M) == prod(expected_size)
        M = reshape(M, expected_size);
    end

    if ~isequal(size(M), expected_size)
        error('lqr_stabilize_controller:InvalidJsonMatrixSize', ...
              'Invalid JSON matrix %s: expected %dx%d but got %dx%d.', ...
              name, expected_size(1), expected_size(2), size(M, 1), size(M, 2));
    end

    if any(~isfinite(M(:)))
        error('lqr_stabilize_controller:InvalidJsonMatrixValue', ...
              'Invalid JSON matrix %s: contains NaN or Inf.', name);
    end
end

function y = local_wrap_to_pi(angle_rad)
    y = atan2(sin(angle_rad), cos(angle_rad));
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end
