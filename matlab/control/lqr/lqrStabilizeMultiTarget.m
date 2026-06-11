function out = lqrStabilizeMultiTarget(t, state, params, sim_config)

    %#ok<NASGU>
    if nargin < 3 || isempty(params)
        params = loadParams();
    end
    if nargin < 4 || isempty(sim_config)
        sim_config = struct();
    end

    x = double(state(:));
    if numel(x) ~= 6
        error('lqrStabilizeMultiTarget:InvalidState', 'state must have 6 elements.');
    end

    target_name = local_target_name(sim_config);
    lib = targetEquilibriumLibrary(params);
    if ~isfield(lib.targets, target_name)
        error('lqrStabilizeMultiTarget:UnknownTarget', 'Unknown target "%s".', target_name);
    end

    gains = local_load_gains(params, sim_config);
    if ~isfield(gains, target_name)
        error('lqrStabilizeMultiTarget:MissingGain', ...
              'No LQR baseline LQR gain found for target "%s". Run designLqrAllEquilibria first.', target_name);
    end

    item = gains.(target_name);
    K = double(item.K);
    target_state = double(item.target_state(:));
    if numel(target_state) ~= 6
        target_state = lib.targets.(target_name).target_state(:);
    end

    error_state = x - target_state;
    error_state(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
    error_state(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

    u_raw = -K * error_state;
    u_cmd = local_saturate_force(u_raw, params);

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = double(u_raw);
    out.mode = sprintf('lqrBaseline_lqr_%s', target_name);
    out.target_mode = target_name;
    out.target_state = target_state;
    out.error_state = error_state;
    out.measurement = x;
    out.x_hat = x;
    out.saturation_flag = abs(double(u_raw) - double(u_cmd)) > 1e-9;
    out.controller_source = 'matlab/control/lqrStabilizeMultiTarget.m';
end

function target_name = local_target_name(sim_config)
    candidate_fields = {'target_mode', 'active_target', 'requested_target', 'lqr_target_mode'};
    target_name = 'up_up';
    for i = 1:numel(candidate_fields)
        f = candidate_fields{i};
        if isfield(sim_config, f) && ~isempty(sim_config.(f))
            target_name = char(sim_config.(f));
            return;
        end
    end
end

function gains = local_load_gains(params, sim_config)
    gains_path = '';
    if isfield(sim_config, 'lqr_gains_all_equilibria_path') && ~isempty(sim_config.lqr_gains_all_equilibria_path)
        gains_path = char(sim_config.lqr_gains_all_equilibria_path);
    elseif isfield(sim_config, 'lqr_gains_path') && ~isempty(sim_config.lqr_gains_path)
        gains_path = char(sim_config.lqr_gains_path);
    end

    if isempty(gains_path)
        project_root = RecoveryUtils.projectRoot(params);
        gains_path = fullfile(project_root, 'shared', 'lqr_gains_all_equilibria.mat');
    elseif ~isfile(gains_path)
        gains_path = fullfile(pwd, gains_path);
    end

    if ~isfile(gains_path)
        error('lqrStabilizeMultiTarget:GainFileNotFound', ...
              'LQR baseline gain file not found: %s', gains_path);
    end

    loaded = load(gains_path);
    if isfield(loaded, 'gains')
        gains = loaded.gains;
    elseif isfield(loaded, 'report') && isfield(loaded.report, 'gains')
        gains = loaded.report.gains;
    else
        error('lqrStabilizeMultiTarget:InvalidGainFile', ...
              'File %s does not include gains or report.gains.', gains_path);
    end
end

function project_root = projectRoot(params)
    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        project_root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        control_dir = fileparts(this_file);
        matlab_root = fileparts(control_dir);
        project_root = fileparts(matlab_root);
    end
end

function y = wrapToPi(angle_rad)
    y = atan2(sin(angle_rad), cos(angle_rad));
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end
