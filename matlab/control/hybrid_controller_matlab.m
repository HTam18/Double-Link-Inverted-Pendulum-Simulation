function [out, hybrid_state] = hybrid_controller_matlab(t, state, params, sim_config, hybrid_state)
% HYBRID_CONTROLLER_MATLAB Phase 30 hybrid MATLAB controller.
%
% Purpose:
%   Combine the Phase 29 discrete TVLQR trajectory tracker with local LQR
%   stabilization around the up_up target. The controller is intentionally
%   built on the Phase 29 tracker instead of redesigning trajectory tracking.
%
% Main mode sequence:
%   idle -> trajectory_tracking -> stabilize
%
% Safety modes:
%   recovery  - temporary mode when tracking/stabilize quality is outside the
%               normal capture gate but still inside safe physical limits.
%   failsafe  - zero or rail-safe command when rail, saturation, nonfinite
%               state or large velocity safety rules are violated.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.
%
% Compatible call patterns:
%   out = hybrid_controller_matlab(t, state, params, sim_config)
%   [out, hybrid_state] = hybrid_controller_matlab(t, state, params, sim_config, hybrid_state)
%
% The second pattern is preferred for Phase 30 because the simulation script
% owns the controller state and logs mode transitions clearly. The four input
% pattern uses a persistent state only for compatibility with run_simulation.m.

    persistent persistent_state

    if nargin < 4 || isempty(sim_config)
        sim_config = struct();
    end
    if nargin < 3 || isempty(params)
        params = load_params();
    end
    explicit_state = nargin >= 5 && ~isempty(hybrid_state);

    cfg = local_fill_hybrid_defaults(sim_config, params);

    if explicit_state
        hs = hybrid_state;
    else
        if isempty(persistent_state) || local_should_reset(t, sim_config)
            persistent_state = local_init_state(cfg);
        end
        hs = persistent_state;
    end

    if local_should_reset(t, sim_config) || ~isfield(hs, 'initialized') || ~hs.initialized
        hs = local_init_state(cfg);
    end

    x = double(state(:));
    if numel(x) ~= 6
        error('hybrid_controller_matlab:InvalidState', 'state must have 6 elements.');
    end

    safety = local_safety_check(t, x, hs, cfg, params);
    if safety.failsafe
        hs.mode = 'failsafe';
        hs.fail_reason = safety.reason;
    end

    if strcmp(hs.mode, 'idle')
        hs.mode = 'trajectory_tracking';
        hs.mode_entry_time = double(t);
    end

    if strcmp(hs.mode, 'trajectory_tracking')
        [cmd, tracking_info] = local_tracking_command(t, x, cfg.tracker, params, cfg, sim_config);
        u_cmd = cmd.u_cmd;
        gate = local_handoff_gate(t, x, cfg.tracker, tracking_info, hs, cfg, params);

        if gate.ready
            hs.mode = 'stabilize';
            hs.mode_entry_time = double(t);
            hs.handoff_time = double(t);
            hs.handoff_count = hs.handoff_count + 1;
            hs.handoff_state = x;
            hs.balance_target_state = local_balance_target_state(cfg, tracking_info, x);
            hs.handoff_reason = gate.reason;
            [u_cmd, lqr_info] = local_terminal_stabilize_command(t, x, params, cfg, hs, sim_config);
            hs.stabilize_gain_source = lqr_info.gain_source;
            tracking_info.lqr_error_state = lqr_info.error_state;
        elseif gate.recovery
            hs.mode = 'recovery';
            hs.mode_entry_time = double(t);
            hs.recovery_count = hs.recovery_count + 1;
            hs.recovery_reason = gate.reason;
        end

    elseif strcmp(hs.mode, 'stabilize')
        [u_cmd, lqr_info] = local_terminal_stabilize_command(t, x, params, cfg, hs, sim_config);
        tracking_info = local_empty_tracking_info(cfg.tracker, x);
        tracking_info.lqr_error_state = lqr_info.error_state;
        tracking_info.x_ref = lqr_info.target_state;
        tracking_info.error_state = lqr_info.error_state;

        stab_gate = local_stabilize_quality(x, cfg);
        if ~stab_gate.safe_for_stabilize
            hs.mode = 'recovery';
            hs.mode_entry_time = double(t);
            hs.recovery_count = hs.recovery_count + 1;
            hs.recovery_reason = stab_gate.reason;
        end

    elseif strcmp(hs.mode, 'recovery')
        [u_cmd, tracking_info, recovered] = local_recovery_command(t, x, params, cfg);
        if recovered
            hs.mode = 'stabilize';
            hs.mode_entry_time = double(t);
            hs.balance_target_state = local_balance_target_state(cfg, tracking_info, x);
        elseif local_recovery_timeout(t, hs, cfg)
            hs.mode = 'failsafe';
            hs.fail_reason = 'recovery_timeout';
            [u_cmd, tracking_info] = local_failsafe_command(x, params, cfg, hs.fail_reason);
        end

    elseif strcmp(hs.mode, 'failsafe')
        [u_cmd, tracking_info] = local_failsafe_command(x, params, cfg, safety.reason);
    else
        error('hybrid_controller_matlab:UnknownMode', 'Unknown hybrid mode: %s', hs.mode);
    end

    u_raw = double(u_cmd);
    u_cmd = local_saturate_force(u_cmd, params);
    hs.sample_count = hs.sample_count + 1;
    hs.last_time = double(t);
    hs.last_u_cmd = u_cmd;
    hs.last_mode = hs.mode;
    if abs(u_cmd) >= cfg.force_limit_N - cfg.saturation_tolerance_N
        hs.saturation_count = hs.saturation_count + 1;
        hs.consecutive_saturation_count = hs.consecutive_saturation_count + 1;
    else
        hs.consecutive_saturation_count = 0;
    end

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = u_raw;
    out.mode = hs.mode;
    out.hybrid_mode = hs.mode;
    out.handoff_flag = strcmp(hs.mode, 'stabilize') && isfinite(hs.handoff_time);
    out.handoff_time = hs.handoff_time;
    out.handoff_count = hs.handoff_count;
    out.recovery_count = hs.recovery_count;
    out.fail_reason = hs.fail_reason;
    out.saturation_fraction_so_far = hs.saturation_count / max(1, hs.sample_count);
    out.consecutive_saturation_count = hs.consecutive_saturation_count;
    out.x_ref = tracking_info.x_ref;
    out.tracking_error_state = tracking_info.error_state;
    out.target_error_state = local_target_error_state(x, cfg.target_state);
    out.final_angle_error_rad = local_final_angle_error(x, cfg.target_state);
    out.velocity_norm = local_velocity_norm(x);
    out.measurement = x;
    out.x_hat = x;
    out.controller_state = hs;

    if explicit_state
        hybrid_state = hs;
    else
        persistent_state = hs;
        hybrid_state = hs;
    end
end

function reset = local_should_reset(t, sim_config)
    reset = false;
    if double(t) <= 1e-12
        reset = true;
    end
    if isfield(sim_config, 'hybrid_reset') && logical(sim_config.hybrid_reset)
        reset = true;
    end
end

function hs = local_init_state(cfg)
    hs = struct();
    hs.initialized = true;
    hs.mode = 'idle';
    hs.mode_entry_time = 0.0;
    hs.last_time = NaN;
    hs.last_u_cmd = 0.0;
    hs.last_mode = 'idle';
    hs.handoff_time = NaN;
    hs.handoff_count = 0;
    hs.handoff_state = nan(6, 1);
    hs.balance_target_state = nan(6, 1);
    hs.handoff_reason = '';
    hs.stabilize_gain_source = '';
    hs.recovery_count = 0;
    hs.recovery_reason = '';
    hs.fail_reason = '';
    hs.sample_count = 0;
    hs.saturation_count = 0;
    hs.consecutive_saturation_count = 0;
    hs.cfg_summary = cfg.summary;
end

function cfg = local_fill_hybrid_defaults(sim_config, params)
    cfg = struct();
    if isfield(sim_config, 'tracker') && ~isempty(sim_config.tracker)
        cfg.tracker = sim_config.tracker;
    else
        cfg.tracker = local_load_phase29_tracker(params);
    end

    cfg.target_mode = local_get_field(sim_config, 'target_mode', 'up_up');
    cfg.target_state = local_target_state(params, cfg.target_mode);
    cfg.lqr_config = struct();
    cfg.lqr_config.target_mode = cfg.target_mode;
    cfg.stabilize_gain_source = local_get_field(sim_config, 'stabilize_gain_source', 'phase29_terminal_tvlqr_hold');
    if isfield(sim_config, 'lqr_gains_path')
        cfg.lqr_config.lqr_gains_path = sim_config.lqr_gains_path;
    end

    cfg.tracking_start_time_s = local_get_field(sim_config, 'tracking_start_time_s', 0.0);
    cfg.tracking_end_time_s = double(cfg.tracker.t(end));
    cfg.min_tracking_time_before_handoff_s = local_get_field(sim_config, 'min_tracking_time_before_handoff_s', max(0.0, cfg.tracking_end_time_s - 2.0));
    cfg.handoff_angle_rad = local_get_field(sim_config, 'handoff_angle_rad', 0.10);
    cfg.handoff_velocity_norm = local_get_field(sim_config, 'handoff_velocity_norm', 0.25);
    cfg.handoff_cart_error_m = local_get_field(sim_config, 'handoff_cart_error_m', 0.20);
    cfg.stabilize_angle_limit_rad = local_get_field(sim_config, 'stabilize_angle_limit_rad', 0.55);
    cfg.stabilize_velocity_limit = local_get_field(sim_config, 'stabilize_velocity_limit', 3.0);
    cfg.recovery_angle_limit_rad = local_get_field(sim_config, 'recovery_angle_limit_rad', 1.20);
    cfg.recovery_velocity_limit = local_get_field(sim_config, 'recovery_velocity_limit', 8.0);
    cfg.recovery_timeout_s = local_get_field(sim_config, 'recovery_timeout_s', 2.0);
    cfg.rail_margin_fail_m = local_get_field(sim_config, 'rail_margin_fail_m', 0.04);
    cfg.rail_margin_warn_m = local_get_field(sim_config, 'rail_margin_warn_m', 0.12);
    cfg.max_abs_velocity_fail = local_get_field(sim_config, 'max_abs_velocity_fail', 20.0);
    cfg.max_consecutive_saturation_samples = local_get_field(sim_config, 'max_consecutive_saturation_samples', 40);
    cfg.saturation_tolerance_N = local_get_field(sim_config, 'saturation_tolerance_N', 1e-6);
    cfg.force_limit_N = max(abs([double(params.control.min_cart_force_N), double(params.control.max_cart_force_N)]));
    cfg.failsafe_rail_push_gain = local_get_field(sim_config, 'failsafe_rail_push_gain', 12.0);
    cfg.summary = sprintf('Phase30 hybrid: tracker until %.3f s, then LQR %s', cfg.tracking_end_time_s, cfg.target_mode);
end

function tracker = local_load_phase29_tracker(params)
    project_root = local_project_root(params);
    result_file = fullfile(project_root, 'results', 'phase29_tvlqr_tracking', 'tvlqr_tracking_result.mat');
    if ~exist(result_file, 'file')
        error('hybrid_controller_matlab:MissingPhase29Result', ...
              'Missing Phase 29 result file: %s. Run matlab/simulation/run_tvlqr_tracking_test.m first.', result_file);
    end
    S = load(result_file);
    if isfield(S, 'actuator_tracker')
        tracker = S.actuator_tracker;
    elseif isfield(S, 'ideal_tracker')
        tracker = S.ideal_tracker;
    else
        error('hybrid_controller_matlab:MissingTracker', 'Phase 29 result does not contain actuator_tracker or ideal_tracker.');
    end
end

function [cmd, info] = local_tracking_command(t, x, tracker, params, cfg, sim_config)
    %#ok<INUSD>
    eval_state = struct();
    if isfield(sim_config, 'current_F_actual')
        eval_state.F_actual = double(sim_config.current_F_actual);
    end
    cmd = tvlqr_tracking_eval_clean(t, x, tracker, params, eval_state);
    info = struct();
    info.x_ref = cmd.x_ref;
    info.error_state = cmd.error_state;
    info.interval_index = cmd.interval_index;
    info.mode = cmd.mode;
end

function gate = local_handoff_gate(t, x, tracker, tracking_info, hs, cfg, params)
    %#ok<INUSD>
    target_error = local_target_error_state(x, cfg.target_state);
    angle_error = max(abs([target_error(3), target_error(5)]));
    velocity_norm = local_velocity_norm(x);
    cart_error = abs(target_error(1));
    rail_ok = local_rail_margin(x, params) > cfg.rail_margin_warn_m;
    saturation_ok = hs.consecutive_saturation_count <= cfg.max_consecutive_saturation_samples;
    time_ok = double(t) >= cfg.min_tracking_time_before_handoff_s;
    tracker_end_ok = double(t) >= double(tracker.t(end)) - 1e-9;

    gate = struct();
    gate.ready = time_ok && tracker_end_ok && rail_ok && saturation_ok && ...
                 angle_error <= cfg.handoff_angle_rad && ...
                 velocity_norm <= cfg.handoff_velocity_norm && ...
                 cart_error <= cfg.handoff_cart_error_m;
    gate.recovery = false;
    gate.reason = sprintf('angle=%.4g velocity=%.4g cart=%.4g rail_ok=%d sat_ok=%d', ...
                          angle_error, velocity_norm, cart_error, rail_ok, saturation_ok);

    if tracker_end_ok && ~gate.ready
        if angle_error <= cfg.recovery_angle_limit_rad && velocity_norm <= cfg.recovery_velocity_limit && rail_ok
            gate.recovery = true;
            gate.reason = ['tracking_end_but_not_handoff_ready: ' gate.reason];
        else
            gate.recovery = true;
            gate.reason = ['tracking_end_outside_gate: ' gate.reason];
        end
    end

    if isfield(tracking_info, 'error_state') && any(~isfinite(tracking_info.error_state(:)))
        gate.ready = false;
        gate.recovery = true;
        gate.reason = 'nonfinite_tracking_error';
    end
end


function target_state = local_balance_target_state(cfg, tracking_info, x)
    target_state = cfg.target_state;
    if isfield(cfg.tracker, 'x_ref') && size(cfg.tracker.x_ref, 2) >= 1
        xref_end = double(cfg.tracker.x_ref(:, end));
        if numel(xref_end) == 6 && all(isfinite(xref_end))
            target_state(1) = xref_end(1);
        else
            target_state(1) = double(x(1));
        end
    elseif isfield(tracking_info, 'x_ref') && numel(tracking_info.x_ref) == 6 && all(isfinite(tracking_info.x_ref(:)))
        target_state(1) = double(tracking_info.x_ref(1));
    else
        target_state(1) = double(x(1));
    end
end

function [u_cmd, info] = local_terminal_stabilize_command(t, x, params, cfg, hs, sim_config)
    % Phase 30 FIX4:
    % After Phase 29 handoff, do not immediately replace the validated TVLQR
    % terminal law by a separate continuous LQR.  The Phase 29 trajectory already
    % contains a terminal capture segment.  For the first balance-hold window,
    % keep using the last valid discrete TVLQR interval with its feedforward
    % force.  This avoids the aggressive post-handoff kick observed in FIX2/FIX3.
    if strcmp(char(cfg.stabilize_gain_source), 'phase29_terminal_tvlqr_hold') || ...
       strcmp(char(cfg.stabilize_gain_source), 'phase29_terminal_tvlqr')
        hold_t = min(double(t), double(cfg.tracker.t(end)) - 1.0e-12);
        eval_state = struct();
        if isfield(sim_config, 'current_F_actual')
            eval_state.F_actual = double(sim_config.current_F_actual);
        end
        tv = tvlqr_tracking_eval_clean(hold_t, x, cfg.tracker, params, eval_state);
        u_raw = tv.u_raw;
        u_cmd = tv.u_cmd;
        error_state = tv.error_state;
        target_state = tv.x_ref;
        gain_source = 'phase29_terminal_tvlqr_hold';
    else
        target_state = hs.balance_target_state;
        if ~isfield(hs, 'balance_target_state') || numel(target_state) ~= 6 || any(~isfinite(target_state))
            target_state = cfg.target_state;
            if isfield(cfg.tracker, 'x_ref') && size(cfg.tracker.x_ref, 2) >= 1
                target_state(1) = double(cfg.tracker.x_ref(1, end));
            else
                target_state(1) = double(x(1));
            end
        end

        lqr_out = lqr_stabilize_controller(0.0, x, params, cfg.lqr_config);
        u_raw = lqr_out.u_raw;
        u_cmd = lqr_out.u_cmd;
        error_state = lqr_out.error_state;
        target_state = lqr_out.target_state;
        gain_source = 'phase22_lqr_fallback';
    end

    info = struct();
    info.u_raw = double(u_raw);
    info.u_cmd = double(u_cmd);
    info.error_state = error_state;
    info.target_state = target_state;
    info.gain_source = gain_source;
end

function q = local_stabilize_quality(x, cfg)
    target_error = local_target_error_state(x, cfg.target_state);
    angle_error = max(abs([target_error(3), target_error(5)]));
    velocity_norm = local_velocity_norm(x);
    q = struct();
    q.safe_for_stabilize = angle_error <= cfg.stabilize_angle_limit_rad && velocity_norm <= cfg.stabilize_velocity_limit;
    q.reason = sprintf('stabilize_quality angle=%.4g velocity=%.4g', angle_error, velocity_norm);
end

function [u_cmd, info, recovered] = local_recovery_command(t, x, params, cfg)
    %#ok<INUSD>
    target_error = local_target_error_state(x, cfg.target_state);
    angle_error = max(abs([target_error(3), target_error(5)]));
    velocity_norm = local_velocity_norm(x);
    info = local_empty_tracking_info(cfg.tracker, x);
    recovered = angle_error <= cfg.handoff_angle_rad && velocity_norm <= cfg.handoff_velocity_norm;

    if angle_error <= cfg.recovery_angle_limit_rad
        lqr_out = lqr_stabilize_controller(t, x, params, cfg.lqr_config);
        u_cmd = lqr_out.u_cmd;
        info.lqr_error_state = lqr_out.error_state;
    else
        % Conservative damping command when outside the LQR capture region.
        u_cmd = -2.0 * x(2) - 0.2 * x(1);
    end
end

function timeout = local_recovery_timeout(t, hs, cfg)
    timeout = double(t) - double(hs.mode_entry_time) > cfg.recovery_timeout_s;
end

function safety = local_safety_check(t, x, hs, cfg, params)
    %#ok<INUSD>
    safety = struct();
    safety.failsafe = false;
    safety.reason = '';
    if any(~isfinite(x))
        safety.failsafe = true;
        safety.reason = 'nonfinite_state';
        return;
    end
    if local_rail_margin(x, params) < cfg.rail_margin_fail_m
        safety.failsafe = true;
        safety.reason = 'rail_margin_fail';
        return;
    end
    if max(abs([x(2), x(4), x(6)])) > cfg.max_abs_velocity_fail
        safety.failsafe = true;
        safety.reason = 'velocity_fail';
        return;
    end
    if hs.consecutive_saturation_count > cfg.max_consecutive_saturation_samples
        safety.failsafe = true;
        safety.reason = 'long_saturation';
        return;
    end
end

function [u_cmd, info] = local_failsafe_command(x, params, cfg, reason)
    %#ok<INUSD>
    info = local_empty_tracking_info(cfg.tracker, x);
    rail = params.rail_limit;
    x_pos = double(x(1));
    x_dot = double(x(2));
    if x_pos > double(rail.x_max_m) - cfg.rail_margin_warn_m
        u_cmd = -cfg.failsafe_rail_push_gain - 2.0*x_dot;
    elseif x_pos < double(rail.x_min_m) + cfg.rail_margin_warn_m
        u_cmd = cfg.failsafe_rail_push_gain - 2.0*x_dot;
    else
        u_cmd = -1.5*x_dot;
    end
    info.fail_reason = reason;
end

function info = local_empty_tracking_info(tracker, x)
    idx = local_interval_index(double(tracker.t(end)), tracker.t);
    if isfield(tracker, 'x_ref')
        x_ref = double(tracker.x_ref(:, end));
    else
        x_ref = nan(6, 1);
    end
    info = struct();
    info.x_ref = x_ref;
    info.error_state = x(:) - x_ref(:);
    if numel(info.error_state) == 6
        info.error_state(3) = local_wrap_to_pi(info.error_state(3));
        info.error_state(5) = local_wrap_to_pi(info.error_state(5));
    end
    info.interval_index = idx;
    info.mode = '';
end

function err = local_target_error_state(x, target_state)
    err = double(x(:)) - double(target_state(:));
    err(3) = local_wrap_to_pi(err(3));
    err(5) = local_wrap_to_pi(err(5));
end

function e = local_final_angle_error(x, target_state)
    err = local_target_error_state(x, target_state);
    e = max(abs([err(3), err(5)]));
end

function v = local_velocity_norm(x)
    v = norm([double(x(2)), double(x(4)), double(x(6))]);
end

function margin = local_rail_margin(x, params)
    if isfield(params, 'rail_limit')
        margin = min(double(x(1)) - double(params.rail_limit.x_min_m), double(params.rail_limit.x_max_m) - double(x(1)));
    else
        margin = inf;
    end
end

function target_state = local_target_state(params, target_mode)
    if ~isfield(params, 'target_modes') || ~isfield(params.target_modes, target_mode)
        error('hybrid_controller_matlab:UnknownTargetMode', 'Unknown target mode: %s', target_mode);
    end
    target = params.target_modes.(target_mode);
    target_state = [0; 0; double(target.theta1_target_rad); 0; double(target.theta2_target_rad); 0];
end

function value = local_get_field(s, name, default_value)
    if isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = default_value;
    end
end

function force = local_saturate_force(u, params)
    force = min(max(double(u), double(params.control.min_cart_force_N)), double(params.control.max_cart_force_N));
end

function idx = local_interval_index(t, time_grid)
    tg = double(time_grid(:));
    idx = find(tg(1:end-1) <= double(t), 1, 'last');
    if isempty(idx)
        idx = 1;
    end
    idx = min(idx, numel(tg) - 1);
end

function y = local_wrap_to_pi(a)
    y = atan2(sin(a), cos(a));
end

function project_root = local_project_root(params)
    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        project_root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        control_dir = fileparts(this_file);
        matlab_root = fileparts(control_dir);
        project_root = fileparts(matlab_root);
    end
end
