function [out, ctx] = hybridMultitargetControllerCore(t, state, ctx, params)

    if nargin < 4 || isempty(params)
        params = loadParams();
    end
    if nargin < 3 || isempty(ctx)
        ctx = struct();
    end
    if ~isfield(ctx, 'initialized') || ~ctx.initialized
        ctx = local_init_ctx(ctx, params);
    end

    x = double(state(:));
    if numel(x) ~= 6
        error('hybridMultitargetController:InvalidState', 'state must have 6 elements.');
    end

    if isfield(ctx, 'pending_target') && ~isempty(ctx.pending_target)
        requested = char(ctx.pending_target);
        ctx.pending_target = '';
        ctx = local_handle_request(t, x, requested, ctx, params);
    end

    switch char(ctx.mode)
        case 'transition_tracking'
            [u_cmd, ctx, detail] = local_transition_control(t, x, ctx, params);
        case {'handoff', 'stabilize_current'}
            [u_cmd, detail] = local_stabilize_control(t, x, ctx, params); %#ok<ASGLU>
            ctx.mode = 'stabilize_current';
        case 'safe_reject'
            [u_cmd, detail] = local_stabilize_control(t, x, ctx, params);
            ctx.mode = 'stabilize_current';
        otherwise
            [u_cmd, detail] = local_stabilize_control(t, x, ctx, params);
            ctx.mode = 'failsafe';
    end

    out = struct();
    out.u_cmd = local_saturate_force(u_cmd, params);
    out.u_raw = double(u_cmd);
    out.mode = char(ctx.mode);
    out.mode_id = local_mode_id(ctx.mode);
    out.active_target = char(ctx.active_target);
    out.requested_target = char(ctx.requested_target);
    out.current_edge = char(ctx.current_edge);
    out.handoff_ready = local_handoff_ready(x, ctx.active_target, ctx.config, params);
    out.saturation_flag = abs(out.u_raw - out.u_cmd) > 1e-9;
    out.safe_reject_reason = char(ctx.safe_reject_reason);
    out.failsafe_reason = char(ctx.failsafe_reason);
    out.detail = detail;
    out.controller_source = 'matlab/control/hybridMultitargetController.m';
end

function ctx = local_init_ctx(ctx, params)
    cfg = RecoveryUtils.defaultCfg(ctx, params);
    if ~isfield(ctx, 'active_target') || isempty(ctx.active_target)
        ctx.active_target = 'down_down';
    end
    ctx.initialized = true;
    ctx.mode = 'stabilize_current';
    ctx.requested_target = ctx.active_target;
    ctx.pending_target = '';
    ctx.current_edge = '';
    ctx.route = {};
    ctx.route_index = 0;
    ctx.tracker = [];
    ctx.transition_start_time = NaN;
    ctx.edge_start_state = [];
    ctx.transition_last_interval_idx = 0;
    ctx.transition_hold_u_cmd = NaN;
    ctx.transition_hold_u_ff = NaN;
    ctx.transition_hold_error_state = zeros(6,1);
    ctx.safe_reject_reason = '';
    ctx.failsafe_reason = '';
    ctx.last_switch_time = -inf;
    ctx.config = cfg;
end

function cfg = defaultCfg(ctx, params)
    cfg = struct();
    project_root = RecoveryUtils.projectRoot(params);
    cfg.project_root = project_root;
    cfg.trajectory_dir = fullfile(project_root, 'shared', 'trajectories');
    cfg.tvlqr_gain_dir = fullfile(project_root, 'shared', 'tvlqr_gains');
    cfg.min_dwell_time_s = 0.20;
    cfg.handoff_angle_rad = 0.08;
    cfg.handoff_velocity_norm = 0.50;
    cfg.handoff_cart_abs_m = 0.60;
    cfg.integration_substeps_per_interval = 7;
    cfg.max_saturation_fraction = 0.08;
    cfg.terminal_capture_enabled = true;
    cfg.terminal_capture_angle_rad = 0.20;
    cfg.terminal_capture_velocity_norm = 1.20;
    cfg.terminal_capture_cart_abs_m = 0.80;
    cfg.allow_early_handoff = false;
    if isfield(ctx, 'config') && isstruct(ctx.config)
        fields = fieldnames(ctx.config);
        for i = 1:numel(fields)
            cfg.(fields{i}) = ctx.config.(fields{i});
        end
    end
end

function ctx = local_handle_request(t, x, requested, ctx, params)
    ctx.requested_target = requested;
    ctx.safe_reject_reason = '';
    ctx.failsafe_reason = '';

    if t - ctx.last_switch_time < ctx.config.min_dwell_time_s
        ctx.mode = 'safe_reject';
        ctx.safe_reject_reason = 'DWELL_TIME_NOT_SATISFIED';
        return;
    end

    if strcmp(requested, ctx.active_target)
        ctx.mode = 'stabilize_current';
        return;
    end

    planner_cfg = struct();
    planner_cfg.trajectory_dir = ctx.config.trajectory_dir;
    planner_cfg.tvlqr_gain_dir = ctx.config.tvlqr_gain_dir;
    planner_cfg.allow_multihop = true;
    planner_cfg.require_tvlqr_gain_file = false;
    route = transitionRoutePlanner(ctx.active_target, requested, params, planner_cfg);
    if ~route.pass
        ctx.mode = 'safe_reject';
        ctx.safe_reject_reason = route.reject_reason;
        return;
    end

    ctx.route = route.edge_names;
    ctx.route_index = 1;
    ctx = local_start_edge(t, x, ctx, params);
end

function ctx = local_start_edge(t, x, ctx, params)
    if ctx.route_index > numel(ctx.route)
        ctx.mode = 'handoff';
        ctx.current_edge = '';
        return;
    end
    edge_name = ctx.route{ctx.route_index};
    [tracker, trajectory] = local_load_tracker(edge_name, ctx, params); %#ok<ASGLU>
    ctx.tracker = tracker;
    ctx.current_edge = edge_name;
    ctx.transition_start_time = double(t);
    ctx.edge_start_state = x(:);
    ctx.transition_last_interval_idx = 0;
    ctx.transition_hold_u_cmd = NaN;
    ctx.transition_hold_u_ff = NaN;
    ctx.transition_hold_error_state = zeros(6,1);
    ctx.mode = 'transition_tracking';
    ctx.last_switch_time = double(t);
end

function [u_cmd, ctx, detail] = local_transition_control(t, x, ctx, params)
    tracker = ctx.tracker;
    tau = double(t) - double(ctx.transition_start_time);
    ref_t = double(tracker.t(:));
    k = find(ref_t(1:end-1) <= tau, 1, 'last');
    if isempty(k), k = 1; end
    k = max(1, min(k, size(tracker.K, 3)));

    eval_state = struct();
    if isfield(ctx, 'actual_force_N') && ~isempty(ctx.actual_force_N)
        eval_state.F_actual = double(ctx.actual_force_N);
    end

    need_update = ~isfield(ctx, 'transition_last_interval_idx') || ...
                  isempty(ctx.transition_last_interval_idx) || ...
                  ctx.transition_last_interval_idx ~= k || ...
                  ~isfield(ctx, 'transition_hold_u_cmd') || ...
                  ~isfinite(double(ctx.transition_hold_u_cmd));
    if need_update
        tv = tvlqrTrackingEvalClean(ref_t(k), x, tracker, params, eval_state);
        ctx.transition_last_interval_idx = k;
        ctx.transition_hold_u_cmd = double(tv.u_cmd);
        ctx.transition_hold_u_ff = double(tv.u_ff);
        ctx.transition_hold_error_state = tv.error_state;
    end
    u_cmd = double(ctx.transition_hold_u_cmd);
    u_ff = double(ctx.transition_hold_u_ff);
    e = double(ctx.transition_hold_error_state(:));

    target = char(tracker.destination_target);
    finished_time = tau >= ref_t(end);
    handoff_ready = local_handoff_ready(x, target, ctx.config, params);
    allow_early_handoff = isfield(ctx.config, 'allow_early_handoff') && ctx.config.allow_early_handoff;

    if (allow_early_handoff && handoff_ready) || finished_time
        if handoff_ready || local_terminal_capture_allowed(x, target, ctx.config, params)
            ctx.active_target = target;
            ctx.route_index = ctx.route_index + 1;
            if ctx.route_index <= numel(ctx.route)
                ctx = local_start_edge(t, x, ctx, params);
            else
                ctx.mode = 'handoff';
                ctx.current_edge = '';
                ctx.tracker = [];
                ctx.transition_last_interval_idx = 0;
                ctx.transition_hold_u_cmd = NaN;
                ctx.transition_hold_u_ff = NaN;
                ctx.transition_hold_error_state = zeros(6,1);
            end
            ctx.safe_reject_reason = '';
            ctx.failsafe_reason = '';
        else
            ctx.mode = 'failsafe';
            ctx.failsafe_reason = 'TRANSITION_FINISHED_OUTSIDE_TERMINAL_CAPTURE_BASIN';
        end
    end

    detail = struct();
    detail.k = k;
    detail.edge_time_s = tau;
    detail.reference_time_s = ref_t(k);
    detail.u_ff = u_ff;
    detail.error_state = e;
    detail.destination_target = target;
end

function [u_cmd, detail] = local_stabilize_control(t, x, ctx, params)
    sim_config = struct();
    sim_config.target_mode = ctx.active_target;
    lqr = lqrStabilizeMultiTarget(t, x, params, sim_config);
    u_cmd = lqr.u_cmd;
    detail = struct();
    detail.error_state = lqr.error_state;
    detail.destination_target = ctx.active_target;
    detail.u_ff = 0.0;
    detail.k = 0;
    detail.edge_time_s = 0.0;
    detail.reference_time_s = 0.0;
end

function [tracker, trajectory] = local_load_tracker(edge_name, ctx, params)
    gain_file = fullfile(ctx.config.tvlqr_gain_dir, sprintf('transition_%s_K.mat', edge_name));
    traj_file = fullfile(ctx.config.trajectory_dir, sprintf('transition_%s_tracking_ready.mat', edge_name));
    if ~isfile(traj_file)
        error('hybridMultitargetController:TrajectoryMissing', 'Missing trajectory file: %s', traj_file);
    end
    loaded_traj = load(traj_file, 'trajectory');
    trajectory = loaded_traj.trajectory;

    if isfile(gain_file)
        loaded = load(gain_file);
        if isfield(loaded, 'actuator_tracker')
            tracker = loaded.actuator_tracker;
        elseif isfield(loaded, 'tracker')
            tracker = loaded.tracker;
        else
            tracker = tvlqrDiscreteTrackingMultiEdge(trajectory, params, struct());
        end
    else
        tv_cfg = struct();
        tv_cfg.terminal_capture_enabled = true;
        tv_cfg.terminal_capture_time_s = 2.0;
        tv_cfg.integration_substeps_per_interval = local_get_nested(trajectory, {'config','integration_substeps_per_interval'}, 7);
        tracker = tvlqrDiscreteTrackingMultiEdge(trajectory, params, tv_cfg);
        if ~exist(ctx.config.tvlqr_gain_dir, 'dir'), mkdir(ctx.config.tvlqr_gain_dir); end
        save(gain_file, 'tracker', 'trajectory');
    end
end

function ready = local_handoff_ready(x, target_name, cfg, params)
    target = local_target_state(params, target_name);
    e = local_wrapped_state_error(x, target);
    angle_err = max(abs([e(3), e(5)]));
    vel_norm = norm([e(2), e(4), e(6)]);
    ready = angle_err <= cfg.handoff_angle_rad && ...
            vel_norm <= cfg.handoff_velocity_norm && ...
            abs(e(1)) <= cfg.handoff_cart_abs_m;
end

function ready = local_terminal_capture_allowed(x, target_name, cfg, params)
    if isfield(cfg, 'terminal_capture_enabled') && ~cfg.terminal_capture_enabled
        ready = false;
        return;
    end
    target = local_target_state(params, target_name);
    e = local_wrapped_state_error(x, target);
    angle_err = max(abs([e(3), e(5)]));
    vel_norm = norm([e(2), e(4), e(6)]);
    angle_gate = local_get_cfg_scalar(cfg, 'terminal_capture_angle_rad', cfg.handoff_angle_rad);
    velocity_gate = local_get_cfg_scalar(cfg, 'terminal_capture_velocity_norm', cfg.handoff_velocity_norm);
    cart_gate = local_get_cfg_scalar(cfg, 'terminal_capture_cart_abs_m', cfg.handoff_cart_abs_m);
    ready = angle_err <= angle_gate && vel_norm <= velocity_gate && abs(e(1)) <= cart_gate;
end

function value = local_get_cfg_scalar(cfg, name, default_value)
    value = default_value;
    if isfield(cfg, name) && ~isempty(cfg.(name))
        value = double(cfg.(name));
    end
end

function target = local_target_state(params, target_name)
    lib = targetEquilibriumLibrary(params);
    target = lib.targets.(char(target_name)).target_state(:);
end

function e = local_wrapped_state_error(x, ref)
    e = double(x(:)) - double(ref(:));
    e(3) = atan2(sin(e(3)), cos(e(3)));
    e(5) = atan2(sin(e(5)), cos(e(5)));
end

function force = local_saturate_force(u, params)
    force = min(max(double(u), double(params.control.min_cart_force_N)), double(params.control.max_cart_force_N));
end

function id = local_mode_id(mode)
    switch char(mode)
        case 'stabilize_current', id = 1;
        case 'transition_tracking', id = 2;
        case 'handoff', id = 3;
        case 'safe_reject', id = 4;
        case 'failsafe', id = 5;
        otherwise, id = 0;
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

function value = local_get_nested(s, names, default_value)
    value = default_value;
    cur = s;
    for i = 1:numel(names)
        name = names{i};
        if isstruct(cur) && isfield(cur, name)
            cur = cur.(name);
        else
            return;
        end
    end
    value = cur;
end
