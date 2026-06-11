function constrainedRecovery_library = generateHardStateLibrary(params, cfg)

    if nargin < 1 || isempty(params), params = loadParams(); end %#ok<NASGU>
    if nargin < 2 || isempty(cfg), cfg = struct(); end

    rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
    cart_gate = RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 1.80);
    angle_gate = RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10);
    vel_gate = RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90);

    default = struct();
    default.duration = 5.6;
    default.absorb_frac = 0.22;
    default.catch_frac = 0.58;
    default.kill_frac = 0.80;
    default.rate_kill = 5.4;
    default.catch_gain = 8.0;
    default.cart_kp = 2.2;
    default.cart_kd = 4.0;
    default.rail_horizon = 0.55;
    default.rail_guard = 1.74;
    default.rail_release = 1.60;
    default.rail_kp = 12.0;
    default.rail_kd = 8.0;
    default.rail_pulse = 0.30;
    default.soft_fraction = 0.88;
    default.roa_angle = 0.32;
    default.roa_vel = 1.65;
    default.K = [3.5 4.0 16.0 5.8 13.0 5.2];

    constrainedRecovery_library = struct();
    constrainedRecovery_library.default = default;
    constrainedRecovery_library.constraints = struct('rail_abs_m', rail, 'cart_gate_abs_m', cart_gate, ...
        'angle_gate_rad', angle_gate, 'velocity_gate', vel_gate, ...
        'note', 'Hard constraints are validated by nonlinear matrix replay; profiles must not be selected unless actual candidate passes gates.');

    constrainedRecovery_library.up_up_l1_r100 = local_merge(default, struct( ...
        'duration', 5.2, 'rate_kill', 4.8, 'catch_gain', 6.4, 'rail_guard', 1.78, 'rail_pulse', 0.24, 'soft_fraction', 0.86));
    constrainedRecovery_library.up_up_l2_r075 = local_merge(default, struct( ...
        'duration', 6.0, 'rate_kill', 6.2, 'catch_gain', 8.8, 'rail_guard', 1.70, 'rail_pulse', 0.34, 'soft_fraction', 0.90));
    constrainedRecovery_library.up_up_l2_r100 = local_merge(default, struct( ...
        'duration', 6.4, 'rate_kill', 6.8, 'catch_gain', 9.4, 'rail_guard', 1.68, 'rail_pulse', 0.38, 'soft_fraction', 0.92));
    constrainedRecovery_library.after_upup_l2_r075 = local_merge(default, struct( ...
        'duration', 6.2, 'rate_kill', 6.0, 'catch_gain', 7.8, 'rail_guard', 1.70, 'rail_pulse', 0.32, 'soft_fraction', 0.88));
    constrainedRecovery_library.after_upup_l2_r100 = local_merge(default, struct( ...
        'duration', 6.8, 'rate_kill', 6.9, 'catch_gain', 8.8, 'rail_guard', 1.66, 'rail_pulse', 0.40, 'soft_fraction', 0.92));
    constrainedRecovery_library.after_upup_l1_r100 = local_merge(default, struct( ...
        'duration', 4.6, 'rate_kill', 4.2, 'catch_gain', 5.0, 'rail_guard', 1.84, 'rail_pulse', 0.22, 'soft_fraction', 0.84));

    if isstruct(cfg) && isfield(cfg, 'recovery_constrainedRecovery_library_mat') && ~isempty(cfg.recovery_constrainedRecovery_library_mat)
        lib_path = char(cfg.recovery_constrainedRecovery_library_mat);
        lib_dir = fileparts(lib_path);
        if ~exist(lib_dir, 'dir'), mkdir(lib_dir); end
        try
            save(lib_path, 'constrainedRecovery_library');
        catch ME
            warning('generateHardStateLibrary:SaveFailed', 'Could not save CONSTRAINED_RECOVERY library: %s', ME.message);
        end
    end
end

function out = local_merge(base, override)
    out = base;
    names = fieldnames(override);
    for i = 1:numel(names)
        out.(names{i}) = override.(names{i});
    end
end

function val = getDouble(s, name, default_val)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && isnumeric(s.(name))
        val = double(s.(name));
    else
        val = double(default_val);
    end
end
