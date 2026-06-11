function metrics = recoveryForceMetrics(sim, target_name, target_state, event, params, cfg)

    if nargin < 6 || isempty(cfg)
        cfg = struct();
    end

    time_s = double(sim.time_s(:));
    states = double(sim.states);
    u_cmd = double(sim.u_cmd(:));
    q_external = double(sim.q_external);
    active_force = logical(sim.active(:));

    n = size(states, 1);
    target_state = double(target_state(:)).';
    if numel(target_state) ~= 6
        error('recoveryForceMetrics:InvalidTargetState', 'target_state must have 6 elements.');
    end

    error_state = states - repmat(target_state, n, 1);
    error_state(:, 3) = atan2(sin(states(:, 3) - target_state(3)), cos(states(:, 3) - target_state(3)));
    error_state(:, 5) = atan2(sin(states(:, 5) - target_state(5)), cos(states(:, 5) - target_state(5)));

    angle_error_norm = sqrt(error_state(:, 3).^2 + error_state(:, 5).^2);
    velocity_norm = sqrt(error_state(:, 2).^2 + error_state(:, 4).^2 + error_state(:, 6).^2);
    cart_abs_m = abs(states(:, 1));
    u_limit = max(abs(double(params.control.min_cart_force_N)), abs(double(params.control.max_cart_force_N)));
    saturation_flag = abs(u_cmd) >= (u_limit - 1e-8);

    gate_angle = RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10);
    gate_velocity = RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.75);
    gate_cart = RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 0.75);
    global_cart_limit = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.00);
    max_saturation_fraction = RecoveryUtils.getDouble(cfg, 'max_saturation_fraction', 0.35);
    settle_hold_s = RecoveryUtils.getDouble(cfg, 'settle_hold_s', 0.30);

    force_end_s = double(event.start_time_s) + double(event.duration_s);
    final_angle_error_rad = angle_error_norm(end);
    final_velocity_norm = velocity_norm(end);
    final_cart_abs_m = cart_abs_m(end);
    max_angle_error_rad = max(angle_error_norm);
    max_velocity_norm = max(velocity_norm);
    max_abs_x_m = max(cart_abs_m);
    max_abs_u_cmd_N = max(abs(u_cmd));
    max_Q_external_norm = max(vecnorm(q_external, 2, 2));
    saturation_fraction = mean(saturation_flag);
    force_sample_count = sum(active_force);

    pass_finite = all(isfinite(states(:))) && all(isfinite(u_cmd)) && all(isfinite(q_external(:)));
    pass_force_applied = force_sample_count > 0 && max_Q_external_norm > 1e-9;
    pass_recovered_final = final_angle_error_rad <= gate_angle && ...
                           final_velocity_norm <= gate_velocity && ...
                           final_cart_abs_m <= gate_cart;
    pass_rail = max_abs_x_m <= global_cart_limit;
    pass_saturation = saturation_fraction <= max_saturation_fraction;

    recovery_time_s = NaN;
    recovered_index = local_find_recovery_index(time_s, angle_error_norm, velocity_norm, cart_abs_m, ...
                                                force_end_s, gate_angle, gate_velocity, gate_cart, settle_hold_s);
    if ~isnan(recovered_index)
        recovery_time_s = max(0.0, time_s(recovered_index) - force_end_s);
    end
    pass_recovery_time = ~isnan(recovery_time_s);

    success = pass_finite && pass_force_applied && pass_recovered_final && pass_rail && pass_saturation && pass_recovery_time;
    fail_reason = local_fail_reason(pass_finite, pass_force_applied, pass_recovered_final, ...
                                    pass_rail, pass_saturation, pass_recovery_time);
    recovery_roa_score = local_recovery_roa_score(final_angle_error_rad, final_velocity_norm, final_cart_abs_m, max_abs_x_m, saturation_fraction, ...
                                                  gate_angle, gate_velocity, gate_cart, global_cart_limit, cfg);
    recovery_margin_score = min([gate_angle - final_angle_error_rad, ...
                                 0.25 * (gate_velocity - final_velocity_norm), ...
                                 gate_cart - final_cart_abs_m, ...
                                 global_cart_limit - max_abs_x_m]);
    primary_fail_reason = RecoveryMetrics.primaryFailReason(fail_reason);

    metrics = struct();
    metrics.target = char(target_name);
    metrics.event_name = char(event.name);
    metrics.link_id = double(event.link_id);
    metrics.contact_ratio = double(event.contact_ratio);
    metrics.direction = char(event.direction_name);
    metrics.force_N = double(event.force_N);
    metrics.duration_s = double(event.duration_s);
    metrics.success = logical(success);
    metrics.recovery_time_s = recovery_time_s;
    metrics.saturation_fraction = saturation_fraction;
    metrics.fail_reason = fail_reason;
    metrics.primary_fail_reason = primary_fail_reason;
    metrics.recovery_roa_score = recovery_roa_score;
    metrics.recovery_margin_score = recovery_margin_score;
    metrics.final_angle_error_rad = final_angle_error_rad;
    metrics.final_velocity_norm = final_velocity_norm;
    metrics.final_cart_abs_m = final_cart_abs_m;
    metrics.max_angle_error_rad = max_angle_error_rad;
    metrics.max_velocity_norm = max_velocity_norm;
    metrics.max_abs_x_m = max_abs_x_m;
    metrics.max_abs_u_cmd_N = max_abs_u_cmd_N;
    metrics.max_Q_external_norm = max_Q_external_norm;
    metrics.pass_finite = logical(pass_finite);
    metrics.pass_force_applied = logical(pass_force_applied);
    metrics.pass_recovered_final = logical(pass_recovered_final);
    metrics.pass_rail = logical(pass_rail);
    metrics.pass_saturation = logical(pass_saturation);
    metrics.pass_recovery_time = logical(pass_recovery_time);
    metrics.force_end_s = force_end_s;
    metrics.gate_angle_rad = gate_angle;
    metrics.gate_velocity_norm = gate_velocity;
    metrics.gate_cart_abs_m = gate_cart;
    metrics.global_cart_limit_m = global_cart_limit;
    metrics.source = 'matlab/evaluation/metrics/recoveryForceMetrics.m';
end


function score = local_recovery_roa_score(final_angle_error_rad, final_velocity_norm, final_cart_abs_m, max_abs_x_m, saturation_fraction, gate_angle, gate_velocity, gate_cart, global_cart_limit, cfg)
    wa = RecoveryUtils.getDouble(cfg, 'recovery_roa_angle_weight', 2.0);
    wv = RecoveryUtils.getDouble(cfg, 'recovery_roa_velocity_weight', 1.2);
    wx = RecoveryUtils.getDouble(cfg, 'recovery_roa_cart_weight', 1.4);
    ws = RecoveryUtils.getDouble(cfg, 'recovery_roa_saturation_weight', 2.5);
    wr = RecoveryUtils.getDouble(cfg, 'recovery_roa_rail_weight', 3.0);
    score = wa * final_angle_error_rad / max(gate_angle, eps) + ...
            wv * final_velocity_norm / max(gate_velocity, eps) + ...
            wx * final_cart_abs_m / max(gate_cart, eps) + ...
            ws * saturation_fraction + ...
            wr * max(0.0, max_abs_x_m / max(global_cart_limit, eps) - 0.75);
end

function reason = primaryFailReason(fail_reason)
    if strcmp(char(fail_reason), 'PASS')
        reason = 'PASS';
        return;
    end
    parts = strsplit(char(fail_reason), '|');
    reason = parts{1};
end

function idx = local_find_recovery_index(time_s, angle_error_norm, velocity_norm, cart_abs_m, force_end_s, gate_angle, gate_velocity, gate_cart, settle_hold_s)
    idx = NaN;
    dt = median(diff(time_s));
    if isempty(dt) || ~isfinite(dt) || dt <= 0
        dt = 0.01;
    end
    hold_samples = max(1, ceil(settle_hold_s / dt));
    candidate_start = find(time_s >= force_end_s, 1, 'first');
    if isempty(candidate_start)
        return;
    end
    inside = angle_error_norm <= gate_angle & velocity_norm <= gate_velocity & cart_abs_m <= gate_cart;
    for k = candidate_start:(numel(time_s) - hold_samples + 1)
        if all(inside(k:(k + hold_samples - 1)))
            idx = k;
            return;
        end
    end
end

function reason = local_fail_reason(pass_finite, pass_force_applied, pass_recovered_final, pass_rail, pass_saturation, pass_recovery_time)
    reasons = {};
    if ~pass_finite, reasons{end + 1} = 'NONFINITE_STATE_OR_CONTROL'; end %#ok<AGROW>
    if ~pass_force_applied, reasons{end + 1} = 'FORCE_EVENT_NOT_APPLIED'; end %#ok<AGROW>
    if ~pass_recovered_final, reasons{end + 1} = 'FINAL_STATE_OUTSIDE_RECOVERY_GATE'; end %#ok<AGROW>
    if ~pass_rail, reasons{end + 1} = 'CART_RAIL_LIMIT_EXCEEDED'; end %#ok<AGROW>
    if ~pass_saturation, reasons{end + 1} = 'SATURATION_FRACTION_TOO_HIGH'; end %#ok<AGROW>
    if ~pass_recovery_time, reasons{end + 1} = 'NO_SETTLED_RECOVERY_WINDOW'; end %#ok<AGROW>
    if isempty(reasons)
        reason = 'PASS';
    else
        reason = strjoin(reasons, '|');
    end
end

function value = getDouble(s, name, default_value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = double(s.(name));
    else
        value = default_value;
    end
end
