function ctrl = constrainedRecoveryControllerCore(t, x, target_name, event, params, cfg, anchor_state, anchor_time, variant)

    if nargin < 5 || isempty(params), params = loadParams(); end
    if nargin < 6 || isempty(cfg), cfg = struct(); end
    if nargin < 7 || isempty(anchor_state), anchor_state = x; end
    if nargin < 8 || isempty(anchor_time), anchor_time = 0.0; end
    if nargin < 9 || isempty(variant), variant = 'constrainedRecovery_unknown'; end

    x = double(x(:));
    anchor_state = double(anchor_state(:));
    t_rel = max(0.0, double(t) - double(anchor_time));
    vname = char(variant);

    base_cfg = cfg;
    base_cfg.target_mode = char(target_name);
    base = lqrStabilizeMultiTarget(t, x, params, base_cfg);
    target_state = double(base.target_state(:));

    e_target = x - target_state;
    e_target(3) = RecoveryUtils.wrapToPi(x(3) - target_state(3));
    e_target(5) = RecoveryUtils.wrapToPi(x(5) - target_state(5));

    if startsWith(vname, 'constrainedRecoverya2_')
        [u_raw, state_label, ref] = local_constrainedRecoverya2_microbrake_wrapper(t_rel, x, e_target, base, target_state, params, cfg, vname);
    else
        lib = local_load_constrainedRecovery_library(cfg, params);
        [u_raw, state_label, ref] = local_constrainedRecoveryb_upup_closed_loop_tracking(t_rel, x, e_target, base, target_state, anchor_state, event, params, cfg, vname, lib);
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
    ctrl.reference_state = ref;
    ctrl.error_state = e_target;
    ctrl.anchor_state = anchor_state;
    ctrl.anchor_time = anchor_time;
    ctrl.saturation_flag = abs(u_cmd - u_raw) > 1e-9;
    ctrl.controller_variant = vname;
    ctrl.controller_source = 'matlab/control/constrainedRecoveryController.m';
end

function [u_raw, state_label, ref] = local_constrainedRecoverya2_microbrake_wrapper(~, x, e, base, target_state, params, cfg, vname)
    xcart = x(1); vcart = x(2);
    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min, u_max]));

    p.horizon = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_horizon_s', 0.62);
    p.guard = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_guard_abs_m', 1.82);
    p.release = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_release_abs_m', 1.68);
    p.kd = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_kd', 9.5);
    p.kp = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_kp', 16.0);
    p.pulse = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_pulse_fraction', 0.34);
    p.max_blend = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_max_brake_blend', 0.58);
    p.lqr_keep = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_lqr_keep', 0.96);
    p.rate_keep = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_rate_keep', 0.04);

    if contains(vname, 'after_upup_l1_r100_right')
        p.horizon = 0.52; p.guard = 1.86; p.release = 1.73;
        p.kd = 7.2; p.kp = 10.0; p.pulse = 0.22; p.max_blend = 0.40; p.lqr_keep = 1.00;
    elseif contains(vname, 'downup_l2_r100')
        p.horizon = 0.72; p.guard = 1.78; p.release = 1.62;
        p.kd = 8.8; p.kp = 13.0; p.pulse = 0.30; p.max_blend = 0.50; p.lqr_keep = 0.98;
    end
    if contains(vname, 'soft')
        p.guard = p.guard + 0.04; p.kd = 0.70*p.kd; p.kp = 0.70*p.kp; p.pulse = 0.70*p.pulse;
    elseif contains(vname, 'mid')
        p.guard = p.guard - 0.02; p.kd = 0.90*p.kd; p.kp = 0.90*p.kp;
    elseif contains(vname, 'late')
        p.guard = p.guard + 0.07; p.horizon = max(0.38, p.horizon - 0.18); p.pulse = 0.60*p.pulse;
    end

    xpred = xcart + p.horizon * vcart;
    sp = sign(xpred); if sp == 0, sp = sign(vcart); end; if sp == 0, sp = sign(xcart); end; if sp == 0, sp = 1; end
    sx = sign(xcart); if sx == 0, sx = sp; end
    outward = sx*vcart > 0;
    risk = abs(xpred) > p.guard || (abs(xcart) > p.release && outward) || (abs(xcart) > rail - 0.055 && outward);

    u_base = double(base.u_raw);
    u_rate = -RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecoverya2_angle_rate_damp', 0.20) * (0.20*e(2) + 0.65*e(4) + 0.65*e(6));
    pred_excess = max(0.0, abs(xpred) - p.guard);
    vel_out = max(0.0, sp*vcart);
    u_brake = -sp * (p.kp*pred_excess + p.kd*vel_out);
    u_brake = min(max(u_brake, -p.pulse*umax), p.pulse*umax);

    if risk
        blend = min(p.max_blend, 0.10 + 0.75*min(1.0, pred_excess / 0.25));
        u_raw = p.lqr_keep*u_base + blend*u_brake + p.rate_keep*u_rate;
        if outward && sign(u_raw)==sx && abs(xpred) > p.guard
            u_raw = (1-blend)*u_base + blend*u_brake + p.rate_keep*u_rate;
        end
        state_label = 'constrainedRecoverya2_micro_brake_active';
    else
        u_raw = u_base + p.rate_keep*u_rate;
        state_label = 'constrainedRecoverya2_base_controller_preserved';
    end
    ref = target_state;
end

function [u_raw, state_label, ref] = local_constrainedRecoveryb_upup_closed_loop_tracking(t_rel, x, e, base, target_state, anchor_state, event, params, cfg, vname, lib)
    family = local_constrainedRecovery_family_name(vname, event);
    p = local_constrainedRecovery_family_params(lib, family, vname, cfg);
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    umax = max(abs([u_min, u_max]));

    angle_norm = norm([e(3), e(5)]);
    vel_norm = norm([e(2), e(4), e(6)]);
    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
    xcart = x(1); vcart = x(2);
    rail_margin_ok = abs(xcart) < min(1.50, rail - 0.30) && abs(vcart) < 1.45;
    in_roa = angle_norm < p.roa_angle && vel_norm < p.roa_vel && rail_margin_ok;

    if in_roa
        ref = target_state;
        u_raw = double(base.u_raw);
        state_label = ['constrainedRecoveryb_', family, '_roa_terminal_lqr'];
        return;
    end

    tau = min(max(t_rel / max(p.duration, eps), 0.0), 1.0);
    s = local_smoothstep(tau);

    ref = target_state;
    ref(1) = (1-s)*anchor_state(1);
    ref(2) = (1-s)*anchor_state(2);
    ref(3) = target_state(3) + (1-s)*RecoveryUtils.wrapToPi(anchor_state(3)-target_state(3));
    ref(4) = (1-s)*anchor_state(4);
    ref(5) = target_state(5) + (1-s)*RecoveryUtils.wrapToPi(anchor_state(5)-target_state(5));
    ref(6) = (1-s)*anchor_state(6);

    e_ref = x - ref;
    e_ref(3) = RecoveryUtils.wrapToPi(x(3)-ref(3));
    e_ref(5) = RecoveryUtils.wrapToPi(x(5)-ref(5));

    K = RecoveryIo.loadPreferredGain(params, cfg, target_state, char(target_name_from_state(target_state)));
    if isempty(K) || ~isnumeric(K) || size(K,2) ~= 6
        K = p.K;
    end
    if size(K,1) > 1
        K = K(1,:);
    end
    u_track = -double(K) * e_ref;
    u_lqr = double(base.u_raw);

    theta_rate = 0.55*e(4) + 0.55*e(6);
    u_absorb = -p.rate_kill * (0.18*e(2) + theta_rate);
    u_cart = -p.cart_kp*xcart - p.cart_kd*vcart;
    timing_dir = sign(0.75*e(3) + 0.55*e(5) + 0.20*e(4) + 0.20*e(6));
    if timing_dir == 0, timing_dir = -sign(vcart); end
    if timing_dir == 0, timing_dir = 1; end
    u_catch = -p.catch_gain * timing_dir;

    horizon = p.rail_horizon;
    guard = p.rail_guard;
    xpred = xcart + horizon*vcart;
    sx = sign(xcart); if sx == 0, sx = sign(vcart); end; if sx == 0, sx = 1; end
    sp = sign(xpred); if sp == 0, sp = sx; end
    outward = sx*vcart > 0;
    rail_risk = abs(xpred) > guard || (abs(xcart) > p.rail_release && outward) || (abs(xcart) > rail-0.06 && outward);
    u_rail = -sp * (p.rail_kp*max(0,abs(xpred)-guard) + p.rail_kd*max(0,sp*vcart));
    u_rail = min(max(u_rail, -p.rail_pulse*umax), p.rail_pulse*umax);

    if tau < p.absorb_frac
        state_label = ['constrainedRecoveryb_', family, '_absorb_energy'];
        u_raw = 0.35*u_track + 0.12*u_lqr + 0.85*u_absorb + 0.22*u_cart;
    elseif tau < p.catch_frac
        state_label = ['constrainedRecoveryb_', family, '_closedloop_angle_catch'];
        u_raw = 0.58*u_track + 0.22*u_lqr + 0.58*u_absorb + 0.32*u_catch + 0.20*u_cart;
    elseif tau < p.kill_frac
        state_label = ['constrainedRecoveryb_', family, '_rate_kill_before_roa'];
        u_raw = 0.65*u_track + 0.42*u_lqr + 0.78*u_absorb + 0.18*u_cart;
    else
        state_label = ['constrainedRecoveryb_', family, '_terminal_roa_approach'];
        u_raw = 0.44*u_track + 0.82*u_lqr + 0.38*u_absorb + 0.12*u_cart;
    end

    if rail_risk
        if outward && sign(u_raw)==sx
            u_raw = 0.35*u_raw + u_rail + 0.12*u_absorb;
            state_label = [state_label, '_rail_outward_blocked'];
        else
            u_raw = u_raw + 0.45*u_rail;
            state_label = [state_label, '_rail_micro_brake'];
        end
    end

    u_raw = local_soft_bound(u_raw, umax, p.soft_fraction);
end

function family = local_constrainedRecovery_family_name(vname, event)
    vname = char(vname);
    if contains(vname, 'after_upup_l2_r100')
        family = 'after_upup_l2_r100';
    elseif contains(vname, 'after_upup_l2_r075')
        family = 'after_upup_l2_r075';
    elseif contains(vname, 'after_upup_l1_r100')
        family = 'after_upup_l1_r100';
    elseif contains(vname, 'upup_l2_r100') || (double(event.link_id)==2 && double(event.contact_ratio)>=1.0-1e-12)
        family = 'up_up_l2_r100';
    elseif contains(vname, 'upup_l2_r075') || (double(event.link_id)==2 && double(event.contact_ratio)>=0.75-1e-12)
        family = 'up_up_l2_r075';
    elseif contains(vname, 'upup_l1_r100') || double(event.link_id)==1
        family = 'up_up_l1_r100';
    else
        family = 'up_up_generic';
    end
end

function p = local_constrainedRecovery_family_params(lib, family, vname, cfg)
    p = lib.default;
    if isfield(lib, family)
        f = lib.(family);
        names = fieldnames(f);
        for i=1:numel(names)
            p.(names{i}) = f.(names{i});
        end
    end
    if contains(vname, 'slow')
        p.duration = p.duration + 1.0; p.catch_gain = 0.82*p.catch_gain; p.soft_fraction = min(0.98, p.soft_fraction+0.03);
    elseif contains(vname, 'rail')
        p.rail_guard = min(p.rail_guard, 1.70); p.rail_pulse = min(0.48, p.rail_pulse+0.08); p.catch_gain = 0.86*p.catch_gain;
    elseif contains(vname, 'aggressive')
        p.catch_gain = 1.18*p.catch_gain; p.rate_kill = 1.16*p.rate_kill; p.soft_fraction = min(0.995, p.soft_fraction+0.05);
    end
    p.roa_angle = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecovery_roa_angle_rad', p.roa_angle);
    p.roa_vel = RecoveryUtils.getDouble(cfg, 'recovery_constrainedRecovery_roa_velocity_norm', p.roa_vel);
end

function lib = local_load_constrainedRecovery_library(cfg, params)
    lib_path = '';
    if isfield(cfg, 'recovery_constrainedRecovery_library_mat')
        lib_path = char(cfg.recovery_constrainedRecovery_library_mat);
    end
    if ~isempty(lib_path) && exist(lib_path, 'file')
        s = load(lib_path);
        if isfield(s, 'constrainedRecovery_library')
            lib = s.constrainedRecovery_library;
            return;
        end
    end
    lib = generateHardStateLibrary(params, cfg);
end

function K = loadPreferredGain(params, cfg, target_state, target_name)
    K = [];
    try
        project_root = projectRoot_from_cfg(cfg);
        gain_path = fullfile(project_root, 'shared', 'lqr_recovery_gains_all_equilibria.mat');
        if ~isfile(gain_path)
            gain_path = fullfile(project_root, 'shared', 'lqr_gains_all_equilibria.mat');
        end
        if isfile(gain_path)
            s = load(gain_path);
            if isfield(s, 'gains') && isfield(s.gains, target_name) && isfield(s.gains.(target_name), 'K')
                K = double(s.gains.(target_name).K);
            elseif isfield(s, 'lqr_gains') && isfield(s.lqr_gains, target_name) && isfield(s.lqr_gains.(target_name), 'K')
                K = double(s.lqr_gains.(target_name).K);
            end
        end
    catch
        K = [];
    end
    if isempty(K)
        K = [3.5 4.0 16.0 5.8 13.0 5.2]; %#ok<NASGU>
    end
end

function name = target_name_from_state(target_state)
    a1 = atan2(sin(target_state(3)), cos(target_state(3)));
    a2 = atan2(sin(target_state(5)), cos(target_state(5)));
    if abs(abs(a1)-pi) < 0.3 && abs(abs(a2)-pi) < 0.3
        name = 'up_up';
    elseif abs(abs(a1)-pi) < 0.3
        name = 'up_down';
    elseif abs(abs(a2)-pi) < 0.3
        name = 'down_up';
    else
        name = 'down_down';
    end
end

function u = local_soft_bound(u_raw, umax, frac)
    lim = max(1e-6, double(frac)*double(umax));
    u = lim * tanh(double(u_raw)/lim);
end

function s = local_smoothstep(tau)
    tau = min(max(double(tau),0),1);
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
end

function y = wrapToPi(a)
    y = atan2(sin(a), cos(a));
end

function val = getDouble(s, name, default_val)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isnumeric(s.(name))
        val = double(s.(name));
    else
        val = double(default_val);
    end
end

function root = projectRoot_from_cfg(cfg)
    if isstruct(cfg) && isfield(cfg, 'project_root') && ~isempty(cfg.project_root)
        root = char(cfg.project_root);
        return;
    end
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
