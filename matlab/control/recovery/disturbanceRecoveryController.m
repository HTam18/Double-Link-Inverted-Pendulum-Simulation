function out = disturbanceRecoveryController(t, state, params, cfg)

    if nargin < 3 || isempty(params), params = loadParams(); end
    if nargin < 4 || isempty(cfg), cfg = struct(); end

    x = double(state(:));
    target_name = RecoveryUtils.getChar(cfg, 'target_mode', 'up_up');

    base_cfg = cfg;
    base_cfg.target_mode = target_name;
    base_lqr = lqrStabilizeMultiTarget(t, x, params, base_cfg);

    rec_lqr = base_lqr;
    recovery_gain_available = false;
    rec_gain_path = local_recovery_gain_path(params, cfg);
    if ~isempty(rec_gain_path) && isfile(rec_gain_path)
        try
            rec_cfg = cfg;
            rec_cfg.target_mode = target_name;
            rec_cfg.lqr_gains_path = rec_gain_path;
            rec_lqr = lqrStabilizeMultiTarget(t, x, params, rec_cfg);
            recovery_gain_available = true;
        catch
            rec_lqr = base_lqr;
            recovery_gain_available = false;
        end
    end

    target_state = double(base_lqr.target_state(:));
    e = x - target_state;
    e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
    e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

    gate = local_gate(cfg);
    angle_error_norm = norm([e(3), e(5)]);
    velocity_norm = norm([e(2), e(4), e(6)]);
    angular_velocity_norm = norm([e(4), e(6)]);
    cart_abs_m = abs(x(1));
    inside_gate = angle_error_norm <= gate.angle_rad && velocity_norm <= gate.velocity_norm && cart_abs_m <= gate.cart_abs_m;

    severity = local_severity(angle_error_norm, velocity_norm, cart_abs_m, gate);
    mode = local_mode(e, angle_error_norm, velocity_norm, cart_abs_m, gate, cfg);

    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    hard_limit = max(abs([u_min, u_max]));

    if inside_gate
        u_lqr_raw = double(base_lqr.u_raw);
        lqr_source = 'lqrBaseline_local_lqr';
    else
        u_lqr_raw = double(rec_lqr.u_raw);
        if abs(u_lqr_raw) < 0.85 * abs(double(base_lqr.u_raw))
            u_lqr_raw = double(base_lqr.u_raw);
            lqr_source = 'lqrBaseline_local_lqr_protected';
        elseif recovery_gain_available
            lqr_source = 'recovery_recovery_lqr';
        else
            lqr_source = 'lqrBaseline_local_lqr_fallback';
        end
    end

    u_aux = 0.0;
    if RecoveryUtils.getLogical(cfg, 'recovery_enable_state_machine', true) && ~inside_gate
        switch mode
            case 'cart_safety_recenter'
                u_aux = local_cart_safety_term(e, cfg);
            case 'disturbance_absorb'
                u_aux = local_absorb_term(e, cfg);
            case 'angle_realign'
                u_aux = local_angle_realign_term(e, cfg);
            case 'large_disturbance_recovery'
                u_aux = local_cart_safety_term(e, cfg) + local_absorb_term(e, cfg) + local_angle_realign_term(e, cfg);
            otherwise
                u_aux = 0.0;
        end
    end

    aux_fraction = RecoveryUtils.getDouble(cfg, 'recovery_aux_limit_fraction', 0.18);
    u_aux = local_saturate(u_aux, -aux_fraction * hard_limit, aux_fraction * hard_limit);

    cart_safety_priority = cart_abs_m >= RecoveryUtils.getDouble(cfg, 'recovery_cart_priority_abs_m', 0.62);
    if ~cart_safety_priority && abs(u_lqr_raw) > 1e-9 && sign(u_aux) ~= sign(u_lqr_raw)
        u_aux = 0.25 * u_aux;
    end

    scale_min = RecoveryUtils.getDouble(cfg, 'recovery_lqr_scale_min', 1.00);
    scale_max = RecoveryUtils.getDouble(cfg, 'recovery_lqr_scale_max', 1.35);
    if inside_gate
        scale = 1.0;
    else
        scale = min(scale_max, scale_min + (scale_max - scale_min) * min(1.0, severity));
    end

    u_traj = RecoveryUtils.getDouble(cfg, 'recovery_recovery_trajectory_u_ff', 0.0);
    traj_enabled = RecoveryUtils.getLogical(cfg, 'recovery_enable_recovery_trajectory_hook', false);
    if ~traj_enabled, u_traj = 0.0; end

    u_raw = scale * u_lqr_raw + u_aux + u_traj;
    u_cmd = local_saturate(u_raw, u_min, u_max);

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = u_raw;
    out.u_lqr_raw = u_lqr_raw;
    out.u_lqr_cmd = local_saturate(u_lqr_raw, u_min, u_max);
    out.u_base_lqr_raw = double(base_lqr.u_raw);
    out.u_recovery_lqr_raw = double(rec_lqr.u_raw);
    out.u_aux = u_aux;
    out.u_cart_recenter = local_cart_safety_term(e, cfg);
    out.u_angle_damping = local_absorb_term(e, cfg);
    out.u_angle_realign = local_angle_realign_term(e, cfg);
    out.u_recovery_trajectory = u_traj;
    out.mode = mode;
    out.recovery_state = mode;
    out.target_mode = target_name;
    out.target_state = target_state;
    out.error_state = e;
    out.angle_error_norm = angle_error_norm;
    out.velocity_norm = velocity_norm;
    out.angular_velocity_norm = angular_velocity_norm;
    out.cart_abs_m = cart_abs_m;
    out.recovery_severity = severity;
    out.lqr_gain_scale = scale;
    out.recovery_gain_available = recovery_gain_available;
    out.lqr_source = lqr_source;
    out.inside_recovery_gate = inside_gate;
    out.saturation_flag = abs(u_cmd - u_raw) > 1e-9 || abs(u_cmd) >= hard_limit - 1e-8;
    out.controller_variant = 'recovery_extended_state_machine_recovery';
    out.controller_source = 'matlab/control/disturbanceRecoveryController.m';
    out.note = 'Recovery EXTENDED: 1/2/3/4/5 N matrix, recovery LQR hook, RoA diagnostics, state machine and trajectory recovery scaffold.';
end

function path = local_recovery_gain_path(params, cfg)
    if isstruct(cfg) && isfield(cfg, 'recovery_lqr_gains_path') && ~isempty(cfg.recovery_lqr_gains_path)
        path = char(cfg.recovery_lqr_gains_path);
        return;
    end
    root = RecoveryUtils.projectRoot(params);
    path = fullfile(root, 'shared', 'lqr_recovery_gains_all_equilibria.mat');
end

function root = projectRoot(params)
    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        root = fileparts(fileparts(fileparts(this_file)));
    end
end

function gate = local_gate(cfg)
    gate.angle_rad = RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10);
    gate.velocity_norm = RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.75);
    gate.cart_abs_m = RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 0.75);
end

function severity = local_severity(angle_error_norm, velocity_norm, cart_abs_m, gate)
    severity = max([angle_error_norm / max(gate.angle_rad, eps), velocity_norm / max(gate.velocity_norm, eps), cart_abs_m / max(gate.cart_abs_m, eps)]);
    severity = min(max(severity - 1.0, 0.0), 2.0) / 2.0;
end

function mode = local_mode(e, angle_error_norm, velocity_norm, cart_abs_m, gate, cfg)
    if angle_error_norm <= gate.angle_rad && velocity_norm <= gate.velocity_norm && cart_abs_m <= gate.cart_abs_m
        mode = 'local_lqr_handoff';
        return;
    end
    rail_warn = RecoveryUtils.getDouble(cfg, 'recovery_cart_safety_abs_m', 0.62);
    large_angle = RecoveryUtils.getDouble(cfg, 'recovery_large_angle_rad', 0.22);
    large_vel = RecoveryUtils.getDouble(cfg, 'recovery_large_velocity_norm', 1.65);
    if cart_abs_m >= rail_warn
        mode = 'cart_safety_recenter';
    elseif angle_error_norm >= large_angle || velocity_norm >= large_vel
        mode = 'large_disturbance_recovery';
    elseif norm([e(4), e(6)]) >= 0.75 * gate.velocity_norm
        mode = 'disturbance_absorb';
    else
        mode = 'angle_realign';
    end
end

function u = local_cart_safety_term(e, cfg)
    kx = RecoveryUtils.getDouble(cfg, 'recovery_cart_kp', 4.0);
    kvx = RecoveryUtils.getDouble(cfg, 'recovery_cart_kd', 1.4);
    dead = RecoveryUtils.getDouble(cfg, 'recovery_cart_deadband_m', 0.05);
    x = e(1);
    if abs(x) < dead
        x_eff = 0.0;
    else
        x_eff = x - sign(x) * dead;
    end
    u = -kx * x_eff - kvx * e(2);
end

function u = local_absorb_term(e, cfg)
    kd1 = RecoveryUtils.getDouble(cfg, 'recovery_theta1_damping', 0.45);
    kd2 = RecoveryUtils.getDouble(cfg, 'recovery_theta2_damping', 0.35);
    kvx = RecoveryUtils.getDouble(cfg, 'recovery_xdot_damping', 0.20);
    u = -kvx * e(2) - kd1 * e(4) - kd2 * e(6);
end

function u = local_angle_realign_term(e, cfg)
    kp1 = RecoveryUtils.getDouble(cfg, 'recovery_theta1_assist_kp', 0.25);
    kp2 = RecoveryUtils.getDouble(cfg, 'recovery_theta2_assist_kp', 0.18);
    kd1 = RecoveryUtils.getDouble(cfg, 'recovery_theta1_assist_kd', 0.12);
    kd2 = RecoveryUtils.getDouble(cfg, 'recovery_theta2_assist_kd', 0.10);
    u = -kp1 * e(3) - kp2 * e(5) - kd1 * e(4) - kd2 * e(6);
end

function y = local_saturate(x, lo, hi)
    y = min(max(double(x), double(lo)), double(hi));
end

function y = wrapToPi(a)
    y = atan2(sin(a), cos(a));
end

function value = getDouble(s, name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        raw = s.(name);
        if isnumeric(raw) || islogical(raw)
            value = double(raw(1));
        else
            parsed = str2double(char(raw));
            if isfinite(parsed), value = parsed; end
        end
    end
end

function value = getLogical(s, name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        raw = s.(name);
        if islogical(raw)
            value = raw(1);
        elseif isnumeric(raw)
            value = raw(1) ~= 0;
        else
            txt = lower(strtrim(char(raw)));
            value = any(strcmp(txt, {'true','1','yes','on'}));
        end
    end
end

function value = getChar(s, name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = char(s.(name));
    end
end
