function ctrl = recoveryRailBrakeFallbackController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)

    if nargin < 5 || isempty(params), params = loadParams(); end
    if nargin < 6 || isempty(cfg), cfg = struct(); end
    if nargin < 7 || isempty(anchor_state), anchor_state = x; end
    if nargin < 8 || isempty(anchor_time), anchor_time = 0.0; end
    if nargin < 9 || isempty(variant), variant = 'railBrakeFallback_preemptive_brake'; end

    x = double(x(:));
    anchor_state = double(anchor_state(:));
    t_rel = max(0.0, double(t) - double(anchor_time)); %#ok<NASGU>
    vname = char(variant);

    base_cfg = cfg;
    base_cfg.target_mode = char(target_name);
    base = lqrStabilizeMultiTarget(t, x, params, base_cfg);
    target_state = double(base.target_state(:));

    e = x - target_state;
    e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
    e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

    [u_raw, state_label] = local_preemptive_rail_brake(x, e, base, params, cfg, vname);

    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    u_cmd = min(max(double(u_raw), u_min), u_max);

    ctrl = struct();
    ctrl.u_cmd = u_cmd;
    ctrl.u_raw = double(u_raw);
    ctrl.u_lqr_raw = double(base.u_raw);
    ctrl.recovery_state = [state_label, '_', vname];
    ctrl.mode = ctrl.recovery_state;
    ctrl.target_mode = char(target_name);
    ctrl.target_state = target_state;
    ctrl.error_state = e;
    ctrl.anchor_state = anchor_state;
    ctrl.anchor_time = anchor_time;
    ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
    ctrl.controller_variant = vname;
    ctrl.controller_source = 'matlab/control/recoveryRailBrakeFallbackController.m';
end

function [u_raw, state_label] = local_preemptive_rail_brake(x, e, base, params, cfg, vname)
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min,u_max]));
    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);

    xcart = double(x(1));
    vcart = double(x(2));
    sx = sign(xcart); if sx == 0, sx = sign(vcart); end; if sx == 0, sx = 1; end

    p = struct();
    p.horizon = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_predict_horizon_s', 1.18);
    p.guard = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_guard_abs_m', 1.56);
    p.release = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_release_abs_m', 1.32);
    p.hard = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_hard_abs_m', 1.78);
    p.kp = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_kp', 72.0);
    p.kd = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_kd', 33.0);
    p.center_kp = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_center_kp', 2.10);
    p.center_kd = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_center_kd', 4.40);
    p.rate_damp = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_rate_damp', 1.55);
    p.lqr_blend = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_lqr_blend', 0.34);
    p.soft = RecoveryUtils.getDouble(cfg, 'recovery_railBrakeFallback_soft_fraction', 0.985);

    if contains(vname, 'early')
        p.horizon = max(p.horizon, 1.45); p.guard = min(p.guard, 1.42); p.release = min(p.release, 1.22);
        p.kp = p.kp * 0.92; p.kd = p.kd * 1.18; p.center_kd = p.center_kd * 1.15;
    elseif contains(vname, 'strong')
        p.horizon = max(p.horizon, 1.28); p.guard = min(p.guard, 1.50);
        p.kp = p.kp * 1.22; p.kd = p.kd * 1.28; p.soft = 0.995; p.lqr_blend = 0.20;
    elseif contains(vname, 'soft')
        p.horizon = max(p.horizon, 1.05); p.guard = max(p.guard, 1.62);
        p.kp = p.kp * 0.72; p.kd = p.kd * 0.88; p.lqr_blend = 0.52; p.soft = 0.94;
    end

    if contains(vname, 'downup_l2_r100_left')
        p.horizon = max(p.horizon, 1.55); p.guard = min(p.guard, 1.38); p.release = min(p.release, 1.18);
        p.kd = p.kd * 1.32; p.center_kd = p.center_kd * 1.25; p.lqr_blend = min(p.lqr_blend, 0.26);
    elseif contains(vname, 'after_upup_l1_r100_right')
        p.horizon = max(p.horizon, 1.35); p.guard = min(p.guard, 1.46); p.release = min(p.release, 1.24);
        p.kp = p.kp * 0.88; p.kd = p.kd * 1.22; p.lqr_blend = max(p.lqr_blend, 0.42);
    end

    xpred = xcart + p.horizon * vcart;
    sp = sign(xpred); if sp == 0, sp = sign(vcart); end; if sp == 0, sp = sx; end
    outward_now = sx * vcart > 0;
    outward_pred = sp * vcart > 0;
    risk = abs(xpred) > p.guard || (abs(xcart) > p.release && outward_now) || (abs(xcart) > p.guard);

    pred_excess = max(0.0, abs(xpred) - p.guard);
    pos_excess = max(0.0, abs(xcart) - p.release);
    out_vel = max(0.0, sp * vcart);

    u_brake = -sp * (p.kp * pred_excess + p.kd * out_vel);

    u_center = -p.center_kp * xcart - p.center_kd * vcart;
    u_rate = -p.rate_damp * (0.18*e(2) + 0.68*e(4) + 0.68*e(6));

    if abs(xcart) > p.release && outward_now
        u_brake = u_brake - sx * (0.40*p.kp*pos_excess + 0.70*p.kd*abs(vcart));
    end
    if abs(xcart) > p.hard && outward_now
        u_brake = -sx * (0.86*umax + 2.80*abs(vcart) + 12.0*max(0.0, abs(xcart)-p.hard));
    end
    if abs(xcart) > rail - 0.045 && outward_now
        u_brake = -sx * 0.998 * umax;
    end

    if risk || outward_pred
        state_label = 'railBrakeFallback_preemptive_predictive_brake';
        u_raw = u_brake + 0.16*u_center + 0.10*u_rate + p.lqr_blend*double(base.u_raw);
        if outward_now && sign(u_raw) == sx && abs(xpred) > p.release
            u_raw = u_brake + 0.22*u_center + 0.08*u_rate;
            state_label = 'railBrakeFallback_outward_lqr_blocked';
        end
    else
        state_label = 'railBrakeFallback_local_settle_after_velocity_kill';
        u_raw = 0.88*double(base.u_raw) + 0.24*u_center + 0.10*u_rate;
    end

    u_raw = min(max(double(u_raw), -p.soft*umax), p.soft*umax);
end

function y = wrapToPi(a)
    y = mod(a + pi, 2*pi) - pi;
end

function val = getDouble(s, name, default_val)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isnumeric(s.(name))
        val = double(s.(name));
    else
        val = double(default_val);
    end
end
