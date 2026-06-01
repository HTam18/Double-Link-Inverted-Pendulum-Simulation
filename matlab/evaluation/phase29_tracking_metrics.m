function metrics = phase29_tracking_metrics(result, reference, params, gate_cfg, open_loop_metrics)
% PHASE29_TRACKING_METRICS Compute clean Phase 29 tracking metrics.
%
% Metrics are used for ideal TVLQR, actuator TVLQR and the final noise test.
% The function does not hide failures: it reports finite, rail, force,
% saturation, RMS, tail RMS, final error and handoff gates separately.

    if nargin < 4 || isempty(gate_cfg)
        gate_cfg = struct();
    end
    if nargin < 5
        open_loop_metrics = [];
    end

    cfg = local_default_gate_cfg(gate_cfg);
    X = double(result.state);
    Ucmd = double(result.u_cmd(:));
    Uactual = double(result.u_actual(:));
    Xref = local_reference_on_result_grid(result.t(:), reference);

    E = X - Xref;
    E(:, 3) = arrayfun(@local_wrap_to_pi, X(:, 3) - Xref(:, 3));
    E(:, 5) = arrayfun(@local_wrap_to_pi, X(:, 5) - Xref(:, 5));

    final_angle_error_1 = abs(local_wrap_to_pi(X(end, 3) - pi));
    final_angle_error_2 = abs(local_wrap_to_pi(X(end, 5) - pi));
    final_max_angle_error = max([final_angle_error_1, final_angle_error_2]);
    final_velocity_norm = norm([X(end, 2), X(end, 4), X(end, 6)]);

    angle_error = max(abs(E(:, [3, 5])), [], 2);
    state_rms = sqrt(mean(E(:).^2));
    n_tail = max(1, ceil(cfg.tail_fraction * size(E, 1)));
    tail_idx = (size(E, 1) - n_tail + 1):size(E, 1);
    tail_angle_rms = sqrt(mean(angle_error(tail_idx).^2));

    max_abs_x = max(abs(X(:, 1)));
    max_abs_u_cmd = max(abs(Ucmd));
    max_abs_u_actual = max(abs(Uactual));
    u_lim = max(abs([double(params.control.min_cart_force_N), double(params.control.max_cart_force_N)]));
    sat = abs(Ucmd) >= (u_lim - cfg.saturation_tol_N) | abs(Uactual) >= (u_lim - cfg.saturation_tol_N);
    saturation_fraction = mean(double(sat));

    x_min = double(params.rail_limit.x_min_m);
    x_max = double(params.rail_limit.x_max_m);
    finite_pass = all(isfinite(X(:))) && all(isfinite(Ucmd)) && all(isfinite(Uactual));
    rail_pass = min(X(:, 1)) >= x_min - cfg.rail_tol_m && max(X(:, 1)) <= x_max + cfg.rail_tol_m;
    force_pass = max_abs_u_cmd <= u_lim + cfg.force_tol_N && max_abs_u_actual <= u_lim + cfg.force_tol_N;
    saturation_pass = saturation_fraction <= cfg.max_saturation_fraction;
    handoff_pass = final_max_angle_error <= cfg.handoff_angle_rad && final_velocity_norm <= cfg.handoff_velocity_norm;

    metrics = struct();
    metrics.finite_pass = finite_pass;
    metrics.rail_pass = rail_pass;
    metrics.force_pass = force_pass;
    metrics.saturation_pass = saturation_pass;
    metrics.handoff_pass = handoff_pass;
    metrics.tracking_reaches_handoff = handoff_pass;
    metrics.final_angle_error_1_rad = final_angle_error_1;
    metrics.final_angle_error_2_rad = final_angle_error_2;
    metrics.final_max_angle_error_rad = final_max_angle_error;
    metrics.final_velocity_norm = final_velocity_norm;
    metrics.rms_state_error = state_rms;
    metrics.rms_angle_error_tail = tail_angle_rms;
    metrics.max_abs_x_m = max_abs_x;
    metrics.max_abs_u_cmd_N = max_abs_u_cmd;
    metrics.max_abs_u_actual_N = max_abs_u_actual;
    metrics.saturation_fraction = saturation_fraction;

    if isempty(open_loop_metrics)
        metrics.better_rms = true;
        metrics.better_tail_angle_rms = true;
        metrics.better_final = true;
    else
        metrics.better_rms = metrics.rms_state_error < open_loop_metrics.rms_state_error;
        metrics.better_tail_angle_rms = metrics.rms_angle_error_tail < open_loop_metrics.rms_angle_error_tail;
        metrics.better_final = metrics.final_max_angle_error_rad < open_loop_metrics.final_max_angle_error_rad;
    end
    metrics.constraints_pass = finite_pass && rail_pass && force_pass && saturation_pass;
    metrics.tracking_gate_pass = metrics.constraints_pass && handoff_pass && metrics.better_rms && metrics.better_tail_angle_rms && metrics.better_final;
end

function cfg = local_default_gate_cfg(user_cfg)
    cfg = struct();
    cfg.handoff_angle_rad = 0.70;
    cfg.handoff_velocity_norm = 5.5;
    cfg.max_saturation_fraction = 0.05;
    cfg.tail_fraction = 0.25;
    cfg.rail_tol_m = 1.0e-9;
    cfg.force_tol_N = 1.0e-9;
    cfg.saturation_tol_N = 1.0e-6;
    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
end

function Xref = local_reference_on_result_grid(t, reference)
    if isstruct(reference) && isfield(reference, 't') && isfield(reference, 'x_ref')
        tref = double(reference.t(:));
        Xr = double(reference.x_ref);
        if size(Xr, 1) == 6 && size(Xr, 2) == numel(tref)
            Xr = Xr.';
        end
    elseif isstruct(reference) && isfield(reference, 't') && isfield(reference, 'x')
        tref = double(reference.t(:));
        Xr = double(reference.x);
        if size(Xr, 1) == 6 && size(Xr, 2) == numel(tref)
            Xr = Xr.';
        end
    else
        error('phase29_tracking_metrics:InvalidReference', 'reference must have t and x_ref/x.');
    end

    Xref = zeros(numel(t), 6);
    for j = 1:6
        Xref(:, j) = interp1(tref, Xr(:, j), double(t(:)), 'previous', 'extrap');
    end
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
