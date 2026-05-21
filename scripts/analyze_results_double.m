function metrics = analyze_results_double(t, states, u_raw, u_sat, disturbance_force, options)
% analyze_results_double
% Calculate response metrics for the double link inverted pendulum project.
%
% State vector order:
% states(:,1) = x
% states(:,2) = x_dot
% states(:,3) = theta1
% states(:,4) = theta1_dot
% states(:,5) = theta2
% states(:,6) = theta2_dot
%
% Angles in states must be in radians.
%
% Example:
% metrics = analyze_results_double(t, states, u_raw, u_sat, disturbance_force, options)

    if nargin < 6
        options = struct();
    end

    t = t(:);
    u_raw = u_raw(:);
    u_sat = u_sat(:);
    disturbance_force = disturbance_force(:);

    if size(states, 2) ~= 6
        error('states must be an Nx6 matrix using [x x_dot theta1 theta1_dot theta2 theta2_dot].');
    end

    if length(t) ~= size(states, 1)
        error('Time vector length must match the number of state rows.');
    end

    if length(u_raw) ~= length(t) || length(u_sat) ~= length(t) || length(disturbance_force) ~= length(t)
        error('u_raw, u_sat, and disturbance_force must have the same length as t.');
    end

    u_max = local_get_option(options, 'u_max', max(abs(u_sat)));
    angle_settle_threshold_deg = local_get_option(options, 'angle_settle_threshold_deg', 0.5);
    cart_settle_threshold_m = local_get_option(options, 'cart_settle_threshold_m', 0.05);
    final_angle_threshold_deg = local_get_option(options, 'final_angle_threshold_deg', 0.1);
    final_cart_threshold_m = local_get_option(options, 'final_cart_threshold_m', 0.01);
    disturbance_start_s = local_get_option(options, 'disturbance_start_s', NaN);
    disturbance_end_s = local_get_option(options, 'disturbance_end_s', NaN);
    case_name = local_get_option(options, 'case_name', 'unnamed_case');

    x = states(:, 1);
    x_dot = states(:, 2);
    theta1 = states(:, 3);
    theta1_dot = states(:, 4);
    theta2 = states(:, 5);
    theta2_dot = states(:, 6);

    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);
    theta1_dot_deg_s = rad2deg(theta1_dot);
    theta2_dot_deg_s = rad2deg(theta2_dot);

    metrics = struct();
    metrics.case_name = case_name;
    metrics.u_max_N = u_max;
    metrics.sim_time_s = t(end);

    metrics.max_abs_theta1_deg = max(abs(theta1_deg));
    metrics.max_abs_theta2_deg = max(abs(theta2_deg));
    metrics.max_abs_cart_position_m = max(abs(x));
    metrics.max_abs_cart_velocity_m_s = max(abs(x_dot));
    metrics.max_abs_theta1_dot_deg_s = max(abs(theta1_dot_deg_s));
    metrics.max_abs_theta2_dot_deg_s = max(abs(theta2_dot_deg_s));

    metrics.max_abs_u_raw_N = max(abs(u_raw));
    metrics.max_abs_u_sat_N = max(abs(u_sat));
    metrics.rms_u_sat_N = sqrt(mean(u_sat.^2));
    metrics.max_abs_disturbance_N = max(abs(disturbance_force));
    metrics.max_abs_total_force_N = max(abs(u_sat + disturbance_force));

    metrics.final_theta1_deg = theta1_deg(end);
    metrics.final_theta2_deg = theta2_deg(end);
    metrics.final_cart_position_m = x(end);

    metrics.theta1_settling_time_s = local_settling_time(t, theta1_deg, angle_settle_threshold_deg, 0);
    metrics.theta2_settling_time_s = local_settling_time(t, theta2_deg, angle_settle_threshold_deg, 0);
    metrics.cart_settling_time_s = local_settling_time(t, x, cart_settle_threshold_m, 0);

    metrics.saturation_used = any(abs(u_raw - u_sat) > 1e-8);
    metrics.saturation_limit_respected = max(abs(u_sat)) <= u_max + 1e-8;

    final_state_ok = abs(metrics.final_theta1_deg) < final_angle_threshold_deg && ...
                     abs(metrics.final_theta2_deg) < final_angle_threshold_deg && ...
                     abs(metrics.final_cart_position_m) < final_cart_threshold_m;
    metrics.final_state_ok = final_state_ok;

    if ~isnan(disturbance_end_s)
        metrics.disturbance_start_s = disturbance_start_s;
        metrics.disturbance_end_s = disturbance_end_s;
        metrics.recovery_time_s = local_recovery_time(t, theta1_deg, theta2_deg, x, ...
            angle_settle_threshold_deg, cart_settle_threshold_m, disturbance_end_s);
    else
        metrics.disturbance_start_s = NaN;
        metrics.disturbance_end_s = NaN;
        metrics.recovery_time_s = NaN;
    end
end

function value = local_get_option(options, name, default_value)
    if isfield(options, name)
        value = options.(name);
    else
        value = default_value;
    end
end

function ts = local_settling_time(t, signal, threshold, target)
    if nargin < 4
        target = 0;
    end

    error_signal = abs(signal(:) - target);
    idx = find(error_signal > threshold, 1, 'last');

    if isempty(idx)
        ts = 0;
    elseif idx == length(t)
        ts = NaN;
    else
        ts = t(idx + 1);
    end
end

function recovery_time = local_recovery_time(t, theta1_deg, theta2_deg, x, angle_threshold_deg, cart_threshold_m, disturbance_end_s)
    start_idx = find(t >= disturbance_end_s, 1, 'first');

    if isempty(start_idx)
        recovery_time = NaN;
        return;
    end

    inside_band = abs(theta1_deg) < angle_threshold_deg & ...
                  abs(theta2_deg) < angle_threshold_deg & ...
                  abs(x) < cart_threshold_m;

    recovery_time = NaN;

    for k = start_idx:length(t)
        if all(inside_band(k:end))
            recovery_time = t(k) - disturbance_end_s;
            return;
        end
    end
end
