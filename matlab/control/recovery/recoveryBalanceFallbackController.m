function ctrl = recoveryBalanceFallbackController(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)

    if nargin < 5 || isempty(params), params = loadParams(); end
    if nargin < 6 || isempty(cfg), cfg = struct(); end
    if nargin < 8 || isempty(anchor_time), anchor_time = 0.0; end
    if nargin < 9 || isempty(variant), variant = 'balanceFallback_unknown'; end

    x = double(x(:));
    if nargin < 7 || isempty(anchor_state), anchor_state = x; end
    anchor_state = double(anchor_state(:)); %#ok<NASGU>
    t_rel = max(0.0, double(t) - double(anchor_time));
    vname = char(variant);

    base_cfg = cfg;
    base_cfg.target_mode = char(target_name);
    base = lqrStabilizeMultiTarget(t, x, params, base_cfg);
    target_state = double(base.target_state(:));

    e = x - target_state;
    e(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
    e(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

    if startsWith(vname, 'balanceFallbacka_')
        [u_raw, state_label] = local_balanceFallbacka_rail_only(t_rel, x, e, base, params, cfg, vname);
    else
        [u_raw, state_label] = local_balanceFallbackb_upup_directed(t_rel, x, e, base, event, params, cfg, vname);
    end

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
    ctrl.controller_source = 'matlab/control/recoveryBalanceFallbackController.m';
end

function [u_raw, state_label] = local_balanceFallbacka_rail_only(t_rel, x, e, base, params, cfg, vname)
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min,u_max]));
    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);

    xcart = x(1); vcart = x(2);
    sx = sign(xcart); if sx == 0, sx = sign(vcart); end; if sx == 0, sx = 1; end
    outward = sx * vcart > 0;
    angle_norm = norm([e(3), e(5)]);
    vel_norm = norm([e(2), e(4), e(6)]);
    near_balanced = angle_norm <= RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_near_angle_rad', 0.32) && ...
                    vel_norm <= RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_near_velocity_norm', 4.20);

    pred_h = RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_predict_horizon_s', 0.72);
    guard = RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_guard_abs_m', 1.46);
    hard_guard = RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_hard_guard_abs_m', 1.76);
    xpred = xcart + pred_h * vcart;
    sp = sign(xpred); if sp == 0, sp = sx; end
    pred_excess = max(0.0, abs(xpred) - guard);
    pos_excess = max(0.0, abs(xcart) - guard);

    kp = RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_predictive_kp', 52.0);
    kd = RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_predictive_kd', 21.0);
    u_rail = -sp * (kp * pred_excess + kd * max(0.0, sp*vcart));
    if abs(xcart) > guard && outward
        u_rail = u_rail - sx * (0.45*kp*pos_excess + 0.55*kd*abs(vcart));
    end
    if abs(xcart) > hard_guard && outward
        u_rail = -sx * (0.78*umax + 10.0*max(0.0, abs(xcart)-hard_guard) + 2.8*abs(vcart));
    end
    if abs(xcart) > rail - 0.035 && outward
        u_rail = -sx * 0.995 * umax;
    end

    u_damp = -RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_velocity_damp', 1.70) * (0.55*e(2) + 0.80*e(4) + 0.72*e(6));
    u_center = -RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_center_kp', 1.45) * xcart - RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_center_kd', 2.30) * vcart;

    if near_balanced || contains(vname, 'railonly')
        if abs(xpred) > guard || abs(xcart) > guard || outward
            state_label = 'balanceFallbacka_predictive_rail_clamp';
            u_raw = u_rail + 0.22*u_damp + 0.12*double(base.u_raw);
        else
            state_label = 'balanceFallbacka_local_settle_after_rail';
            u_raw = 0.86*double(base.u_raw) + 0.18*u_center + 0.12*u_damp;
        end
    else
        state_label = 'balanceFallbacka_rail_guard_with_terminal_damping';
        u_raw = u_rail + 0.35*u_damp + 0.22*u_center + 0.20*double(base.u_raw);
    end
    u_raw = local_soft_bound(u_raw, params, RecoveryUtils.getDouble(cfg, 'recovery_balanceFallbacka_soft_fraction', 0.98));
end

function [u_raw, state_label] = local_balanceFallbackb_upup_directed(t_rel, x, e, base, event, params, cfg, vname)
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min,u_max]));
    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);

    xcart = x(1); vcart = x(2);
    sx = sign(xcart); if sx == 0, sx = sign(vcart); end; if sx == 0, sx = 1; end
    outward = sx * vcart > 0;
    angle_norm = norm([e(3), e(5)]);
    rate_norm = norm([e(4), e(6)]);
    vel_norm = norm([e(2), e(4), e(6)]);
    near_lqr = angle_norm <= 0.18 && vel_norm <= 1.35 && abs(xcart) <= 1.55;

    p = struct('pre',0.15,'rail_t',1.20,'catch_t',4.80,'kill_t',2.20,'handoff_t',2.60, ...
               'barrier',1.52,'hard',1.82,'kp_rail',48.0,'kd_rail',18.0, ...
               'timing_gain',0.64,'lqr_gain',0.36,'rate_kill',3.25,'cart_kp',1.25,'cart_kd',2.65, ...
               'soft',0.96,'pulse',0.34);
    if contains(vname, 'l1_r100')
        p.barrier = 1.68; p.hard = 1.87; p.kp_rail = 34.0; p.kd_rail = 13.5;
        p.catch_t = 6.20; p.kill_t = 2.60; p.timing_gain = 0.74; p.lqr_gain = 0.32;
        p.rate_kill = 3.35; p.cart_kp = 0.82; p.cart_kd = 2.05; p.pulse = 0.42;
    elseif contains(vname, 'l2_r075')
        p.barrier = 1.48; p.hard = 1.78; p.kp_rail = 54.0; p.kd_rail = 21.0;
        p.catch_t = 5.30; p.kill_t = 2.85; p.timing_gain = 0.66; p.lqr_gain = 0.34;
        p.rate_kill = 3.75; p.cart_kp = 1.45; p.cart_kd = 3.15; p.pulse = 0.32;
    elseif contains(vname, 'l2_r100')
        p.barrier = 1.44; p.hard = 1.75; p.kp_rail = 60.0; p.kd_rail = 23.0;
        p.catch_t = 5.60; p.kill_t = 3.10; p.timing_gain = 0.70; p.lqr_gain = 0.32;
        p.rate_kill = 4.05; p.cart_kp = 1.55; p.cart_kd = 3.55; p.pulse = 0.30;
    end
    if contains(vname, 'after_')
        p.rail_t = p.rail_t + 0.35; p.catch_t = p.catch_t + 1.00; p.kill_t = p.kill_t + 0.50;
        p.barrier = min(p.barrier, 1.50); p.kd_rail = p.kd_rail * 1.12;
        p.rate_kill = p.rate_kill * 1.10;
    end
    if contains(vname, 'railfirst')
        p.rail_t = p.rail_t + 0.75; p.barrier = min(p.barrier, 1.42); p.kp_rail = p.kp_rail * 1.18; p.kd_rail = p.kd_rail * 1.18;
        p.timing_gain = p.timing_gain * 0.82;
    end
    if contains(vname, 'ratekill')
        p.rate_kill = p.rate_kill * 1.35; p.kill_t = p.kill_t + 1.10; p.lqr_gain = p.lqr_gain * 0.85;
    end
    if contains(vname, 'longcatch')
        p.catch_t = p.catch_t + 2.00; p.handoff_t = p.handoff_t + 1.00; p.timing_gain = p.timing_gain * 1.10;
    end

    u_rail = 0.0;
    if abs(xcart) > p.barrier || (abs(xcart) > 1.28 && outward)
        u_rail = u_rail - sx * p.kp_rail * max(0.0, abs(xcart)-p.barrier) - p.kd_rail * vcart;
    end
    if abs(xcart) > p.hard || (abs(xcart) > p.barrier && outward)
        u_rail = u_rail - sx * (0.68*umax + 8.0*max(0.0, abs(xcart)-p.hard) + 2.2*abs(vcart));
    end
    if abs(xcart) > rail - 0.025 && outward
        u_rail = -sx * 0.995 * umax;
    end
    u_rail = min(max(u_rail, u_min), u_max);

    timing_measure = e(3)*e(4) + 0.94*e(5)*e(6) + 0.12*(e(3)+e(5));
    timing_dir = -sign(timing_measure);
    if timing_dir == 0, timing_dir = -sign(double(base.u_raw)); end
    if timing_dir == 0, timing_dir = sign(double(event.force_N)); end
    if timing_dir == 0, timing_dir = 1; end
    force_dir_bias = 0.0;
    if isfield(event, 'direction')
        d = char(event.direction);
        if strcmp(d, 'right'), force_dir_bias = -0.06; elseif strcmp(d, 'left'), force_dir_bias = 0.06; end
    end
    energy = min(1.0, (angle_norm + 0.18*rate_norm) / 2.60);
    u_timing = timing_dir * p.pulse * umax * energy + force_dir_bias * umax * energy;
    u_lqr = double(base.u_raw);
    u_ratekill = -p.rate_kill * (0.20*e(2) + 0.92*e(4) + 0.92*e(6));
    u_cart = -p.cart_kp * xcart - p.cart_kd * vcart;

    if near_lqr
        state_label = 'balanceFallbackb_terminal_lqr_handoff';
        u_raw = 0.92*u_lqr + 0.18*u_cart + 0.18*u_ratekill + 0.18*u_rail;
    elseif t_rel <= p.pre
        state_label = 'balanceFallbackb_initial_absorb';
        u_raw = 0.52*u_ratekill + 0.20*u_cart + 0.20*u_lqr + 0.30*u_rail;
    elseif t_rel <= p.pre + p.rail_t || (abs(xcart) > p.barrier && outward)
        state_label = 'balanceFallbackb_rail_first_window';
        assist = p.timing_gain * 0.35*u_timing + p.lqr_gain * 0.22*u_lqr;
        if abs(u_rail) > 0.12*umax && sign(assist) == sx && outward
            assist = 0.08*assist;
        end
        u_raw = u_rail + assist + 0.18*u_ratekill;
    elseif t_rel <= p.pre + p.rail_t + p.catch_t
        tau = (t_rel - p.pre - p.rail_t) / max(p.catch_t, eps);
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        state_label = 'balanceFallbackb_family_energy_catch';
        u_raw = (0.22-0.08*s)*u_rail + (p.timing_gain*(0.85-0.20*s))*u_timing + (p.lqr_gain*(0.45+0.45*s))*u_lqr + 0.16*u_cart + 0.18*u_ratekill;
        if abs(xcart) > p.barrier && outward && sign(u_raw)==sx
            u_raw = 0.92*u_rail + 0.16*u_ratekill + 0.05*u_lqr;
            state_label = 'balanceFallbackb_catch_limited_by_rail';
        end
    elseif t_rel <= p.pre + p.rail_t + p.catch_t + p.kill_t
        tau = (t_rel - p.pre - p.rail_t - p.catch_t) / max(p.kill_t, eps);
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        state_label = 'balanceFallbackb_ratekill_before_handoff';
        u_raw = 0.24*u_rail + (0.48+0.34*s)*u_lqr + (0.95-0.25*s)*u_ratekill + 0.18*u_cart;
    else
        tau = min(max((t_rel - p.pre - p.rail_t - p.catch_t - p.kill_t) / max(p.handoff_t, eps), 0.0), 1.0);
        s = 10*tau^3 - 15*tau^4 + 6*tau^5;
        state_label = 'balanceFallbackb_terminal_settle';
        u_raw = 0.14*u_rail + (0.82+0.18*s)*u_lqr + (0.24-0.10*s)*u_ratekill + 0.14*u_cart;
    end
    if abs(xcart) > p.hard && outward && sign(u_raw) == sx
        u_raw = u_rail;
        state_label = [state_label, '_absolute_rail_override'];
    end
    u_raw = local_soft_bound(u_raw, params, p.soft);
end

function u = local_soft_bound(u_raw, params, frac)
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min,u_max]));
    lim = max(1e-6, double(frac)*umax);
    u = lim * tanh(double(u_raw) / lim);
    u = min(max(u, u_min), u_max);
end

function y = wrapToPi(a)
    y = atan2(sin(a), cos(a));
end

function v = getDouble(s, name, default_value)
    v = default_value;
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = double(s.(name));
    end
end
