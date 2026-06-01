function out = energy_swingup(t, state, params, sim_config)
% ENERGY_SWINGUP Phase 27 energy based swing-up controller for up_up.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad downward, theta = pi rad upright.
%
% Senior note:
%   Double-link swing-up is much harder than single-link swing-up. This
%   controller therefore uses an energy term as the main feedback idea and an
%   optional scheduled excitation copied from the old validated Python baseline
%   as an assist. The schedule is not the only mechanism: energy error,
%   angular motion, cart centering and rail protection are all part of the
%   command. The controller hands off to LQR only inside a local capture region.

    if nargin < 4 || isempty(sim_config)
        sim_config = struct();
    end
    if nargin < 3 || isempty(params)
        params = load_params();
    end

    x = double(state(:));
    if numel(x) ~= 6
        error('energy_swingup:InvalidState', 'state must have 6 elements.');
    end

    cfg = local_get_energy_config(params);
    target_mode = local_get_target_mode(cfg, sim_config);
    if ~strcmp(target_mode, 'up_up')
        error('energy_swingup:UnsupportedTarget', ...
              'Phase 27 energy_swingup currently supports target_mode = up_up only.');
    end

    [angle_error_1, angle_error_2] = local_angle_errors_to_up_up(x);
    max_angle_error = max(abs([angle_error_1, angle_error_2]));
    max_angular_velocity = max(abs([x(4), x(6)]));

    in_capture_region = max_angle_error <= cfg.switch_angle_threshold_rad && ...
                        max_angular_velocity <= cfg.switch_velocity_threshold_rad_s && ...
                        abs(x(1)) <= cfg.switch_cart_threshold_m;

    if in_capture_region && cfg.lqr_handoff_enabled
        sim_config.target_mode = 'up_up';
        lqr_out = lqr_stabilize_controller(t, x, params, sim_config);
        lqr_out.mode = 'stabilize_lqr';
        lqr_out.energy_mode = 'lqr_handoff';
        lqr_out.switch_ready = true;
        lqr_out.energy_error = 0.0;
        lqr_out.max_angle_error = max_angle_error;
        lqr_out.max_angular_velocity = max_angular_velocity;
        out = lqr_out;
        return;
    end

    energy_current = local_total_energy(x, params);
    energy_target = local_target_energy_up_up(params);
    energy_error = energy_target - energy_current;

    pump_signal = x(4) * cos(x(3)) + x(6) * cos(x(5));
    if abs(pump_signal) < 1.0e-6
        pump_signal = cos(2.0 * pi * cfg.fallback_frequency_hz * t + 0.25);
    end

    % Energy feedback term. The tanh keeps the command bounded and prevents a
    % huge force when the energy error is large at down-down.
    energy_scale = max(1.0e-9, abs(energy_target));
    normalized_energy_error = energy_error / energy_scale;
    u_energy = cfg.max_force_N * cfg.energy_weight * ...
               tanh(cfg.energy_gain * normalized_energy_error) * sign(pump_signal);

    % Optional reference excitation from the old validated Python baseline.
    % It is used as an assist, not as the only controller.
    u_schedule = cfg.schedule_weight * local_schedule_force(t, cfg, params);

    % Light shaping and cart centering.
    u_align = cfg.angle_align_gain * (sin(x(3)) + sin(x(5)));
    u_center = -cfg.cart_center_gain * x(1) - cfg.cart_velocity_gain * x(2);

    u_raw = u_schedule + u_energy + u_align + u_center;

    % Capture assist: close to upright, use a small LQR blend but do not call it
    % full handoff until strict capture thresholds are met.
    near_capture_region = max_angle_error <= cfg.assist_angle_threshold_rad && ...
                          max_angular_velocity <= cfg.assist_velocity_threshold_rad_s && ...
                          abs(x(1)) <= cfg.assist_cart_threshold_m && ...
                          cfg.lqr_handoff_enabled;
    mode_name = 'swingup_energy';
    if near_capture_region
        sim_config.target_mode = 'up_up';
        lqr_assist = lqr_stabilize_controller(t, x, params, sim_config);
        u_raw = cfg.assist_energy_blend * u_raw + ...
                (1.0 - cfg.assist_energy_blend) * double(lqr_assist.u_cmd);
        mode_name = 'swingup_energy_capture_assist';
    end

    u_raw = local_apply_rail_margin(u_raw, x, params, cfg);
    u_cmd = local_saturate(u_raw, -cfg.max_force_N, cfg.max_force_N);
    u_cmd = local_saturate_force(u_cmd, params);

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = u_raw;
    out.mode = mode_name;
    out.target_mode = target_mode;
    out.energy_current = energy_current;
    out.energy_target = energy_target;
    out.energy_error = energy_error;
    out.energy_pump_signal = pump_signal;
    out.max_angle_error = max_angle_error;
    out.max_angular_velocity = max_angular_velocity;
    out.switch_ready = false;
    out.measurement = x;
    out.x_hat = x;
end

function cfg = local_get_energy_config(params)
    if isfield(params, 'energy_swingup') && ~isempty(params.energy_swingup)
        cfg = params.energy_swingup;
    else
        cfg = struct();
    end
    cfg.target_mode = local_get_field(cfg, 'target_mode', 'up_up');
    cfg.max_force_N = local_get_number(cfg, 'max_force_N', 12.0);
    cfg.energy_gain = local_get_number(cfg, 'energy_gain', 1.8);
    cfg.energy_weight = local_get_number(cfg, 'energy_weight', 0.45);
    cfg.schedule_weight = local_get_number(cfg, 'schedule_weight', 0.55);
    cfg.cart_center_gain = local_get_number(cfg, 'cart_center_gain', 1.0);
    cfg.cart_velocity_gain = local_get_number(cfg, 'cart_velocity_gain', 0.6);
    cfg.angle_align_gain = local_get_number(cfg, 'angle_align_gain', 3.0);
    cfg.switch_angle_threshold_rad = local_get_number(cfg, 'switch_angle_threshold_rad', 0.40);
    cfg.switch_velocity_threshold_rad_s = local_get_number(cfg, 'switch_velocity_threshold_rad_s', 3.0);
    cfg.switch_cart_threshold_m = local_get_number(cfg, 'switch_cart_threshold_m', 1.5);
    cfg.lqr_handoff_enabled = local_get_bool(cfg, 'lqr_handoff_enabled', true);
    cfg.rail_margin_m = local_get_number(cfg, 'rail_margin_m', 0.25);
    cfg.segment_time_s = local_get_number(cfg, 'segment_time_s', 0.25);
    cfg.fallback_frequency_hz = local_get_number(cfg, 'fallback_frequency_hz', 0.45);
    cfg.assist_angle_threshold_rad = local_get_number(cfg, 'assist_angle_threshold_rad', 0.55);
    cfg.assist_velocity_threshold_rad_s = local_get_number(cfg, 'assist_velocity_threshold_rad_s', 3.5);
    cfg.assist_cart_threshold_m = local_get_number(cfg, 'assist_cart_threshold_m', 1.6);
    cfg.assist_energy_blend = local_get_number(cfg, 'assist_energy_blend', 0.65);
    cfg.force_sequence = local_get_force_sequence(cfg);
end

function seq = local_get_force_sequence(cfg)
    if isfield(cfg, 'force_sequence_N') && ~isempty(cfg.force_sequence_N)
        seq = double(cfg.force_sequence_N(:)).';
        return;
    end
    seq = [ ...
        15 15 15 15 15 15, ...
        -15 -15 -15 -15 -15 -15 -15 -15 -15 -15, ...
        15 15 15 15 15 15, ...
        -15 -15 -15 -15 -15 -15 -15 -15 -15 -15, ...
        -15 -15 -15 -15, ...
        15 15 15 15 15 15 15 15 15 15, ...
        -15 -15];
end

function u = local_schedule_force(t, cfg, params)
    if t < 0
        t = 0;
    end
    idx = floor(t / cfg.segment_time_s) + 1;
    if idx <= numel(cfg.force_sequence)
        u = cfg.force_sequence(idx);
    else
        max_force = double(params.control.max_cart_force_N);
        u = max_force * sin(2.0 * pi * cfg.fallback_frequency_hz * t);
    end
    u = local_saturate_force(u, params);
end

function target_mode = local_get_target_mode(cfg, sim_config)
    if isfield(sim_config, 'target_mode') && ~isempty(sim_config.target_mode)
        target_mode = char(sim_config.target_mode);
    else
        target_mode = char(cfg.target_mode);
    end
end

function [e1, e2] = local_angle_errors_to_up_up(x)
    e1 = local_wrap_to_pi(x(3) - pi);
    e2 = local_wrap_to_pi(x(5) - pi);
end

function E = local_total_energy(x, params)
    theta1 = x(3);
    theta2 = x(5);
    q_dot = [x(2); x(4); x(6)];
    M = dip_mass_matrix(theta1, theta2, params);
    kinetic = 0.5 * q_dot.' * M * q_dot;
    potential = local_potential_energy(theta1, theta2, params);
    E = double(kinetic + potential);
end

function E_target = local_target_energy_up_up(params)
    E_target = local_potential_energy(pi, pi, params);
end

function V = local_potential_energy(theta1, theta2, params)
    p = params.physical_parameters;
    m1 = double(p.link1_mass_kg);
    m2 = double(p.link2_mass_kg);
    l1 = double(p.link1_length_m);
    l2 = double(p.link2_length_m);
    g = double(p.gravity_m_s2);
    y1 = -(l1 / 2.0) * cos(theta1);
    y2 = -l1 * cos(theta1) - (l2 / 2.0) * cos(theta2);
    y1_down = -(l1 / 2.0);
    y2_down = -l1 - (l2 / 2.0);
    V = g * (m1 * (y1 - y1_down) + m2 * (y2 - y2_down));
end

function u = local_apply_rail_margin(u, x, params, cfg)
    if ~isfield(params, 'rail_limit')
        return;
    end
    r = params.rail_limit;
    if ~isfield(r, 'x_min_m') || ~isfield(r, 'x_max_m')
        return;
    end
    x_pos = x(1);
    x_dot = x(2);
    x_min = double(r.x_min_m);
    x_max = double(r.x_max_m);
    margin = double(cfg.rail_margin_m);
    if x_pos > (x_max - margin) && u > 0
        scale = max(0.0, (x_max - x_pos) / margin);
        u = u * scale - 1.5 * x_dot;
    elseif x_pos < (x_min + margin) && u < 0
        scale = max(0.0, (x_pos - x_min) / margin);
        u = u * scale - 1.5 * x_dot;
    end
end

function value = local_get_field(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    else
        value = default_value;
    end
end

function value = local_get_number(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = double(s.(field_name));
    else
        value = double(default_value);
    end
end

function value = local_get_bool(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = logical(s.(field_name));
    else
        value = logical(default_value);
    end
end

function y = local_wrap_to_pi(angle_rad)
    y = atan2(sin(angle_rad), cos(angle_rad));
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end

function y = local_saturate(x, lower, upper)
    y = min(max(double(x), double(lower)), double(upper));
end
