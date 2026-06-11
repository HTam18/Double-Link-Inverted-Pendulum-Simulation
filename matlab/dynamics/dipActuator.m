function [F_next, info] = dipActuator(F_actual, F_cmd, dt, params)

    if nargin < 4 || isempty(params)
        params = loadParams();
    end
    if nargin < 3 || isempty(dt)
        error('dipActuator:MissingDt', 'dt is required.');
    end

    F_actual = double(F_actual);
    F_cmd = double(F_cmd);
    dt = double(dt);

    if ~isscalar(F_actual) || ~isfinite(F_actual)
        error('dipActuator:InvalidActualForce', 'F_actual must be a finite scalar.');
    end
    if ~isscalar(F_cmd) || ~isfinite(F_cmd)
        error('dipActuator:InvalidCommandForce', 'F_cmd must be a finite scalar.');
    end
    if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
        error('dipActuator:InvalidDt', 'dt must be a positive finite scalar.');
    end

    actuator = local_get_actuator_params(params);

    F_cmd_after_dead_zone = F_cmd;
    if abs(F_cmd_after_dead_zone) < actuator.dead_zone_N
        F_cmd_after_dead_zone = 0.0;
    end

    F_cmd_limited = min(max(F_cmd_after_dead_zone, actuator.min_force_N), actuator.max_force_N);

    alpha = exp(-dt / actuator.tau_motor_s);
    F_first_order = F_cmd_limited + (F_actual - F_cmd_limited) * alpha;

    delta_unlimited = F_first_order - F_actual;
    delta_limit = actuator.rate_limit_N_per_s * dt;
    delta_limited = min(max(delta_unlimited, -delta_limit), delta_limit);

    F_next_unsat = F_actual + delta_limited;
    F_next = min(max(F_next_unsat, actuator.min_force_N), actuator.max_force_N);

    F_dot_ideal = (F_cmd_limited - F_actual) / actuator.tau_motor_s;
    F_dot_limited = delta_limited / dt;

    info = struct();
    info.F_cmd_raw = F_cmd;
    info.F_cmd_after_dead_zone = F_cmd_after_dead_zone;
    info.F_cmd_limited = F_cmd_limited;
    info.F_actual_prev = F_actual;
    info.F_actual_next = F_next;
    info.F_dot_ideal = F_dot_ideal;
    info.F_dot_limited = F_dot_limited;
    info.tau_motor_s = actuator.tau_motor_s;
    info.rate_limit_N_per_s = actuator.rate_limit_N_per_s;
    info.dead_zone_N = actuator.dead_zone_N;
    info.min_force_N = actuator.min_force_N;
    info.max_force_N = actuator.max_force_N;
end

function actuator = local_get_actuator_params(params)
    if ~isfield(params, 'actuator') || isempty(params.actuator)
        params.actuator = struct();
    end
    a = params.actuator;

    actuator = struct();
    actuator.tau_motor_s = local_get_numeric(a, 'tau_motor_s', 0.03);
    actuator.rate_limit_N_per_s = local_get_numeric(a, 'rate_limit_N_per_s', 250.0);
    actuator.dead_zone_N = local_get_numeric(a, 'dead_zone_N', 0.0);
    actuator.min_force_N = local_get_numeric(a, 'min_force_N', double(params.control.min_cart_force_N));
    actuator.max_force_N = local_get_numeric(a, 'max_force_N', double(params.control.max_cart_force_N));

    if actuator.tau_motor_s <= 0
        error('dipActuator:InvalidParameter', 'tau_motor_s must be positive.');
    end
    if actuator.rate_limit_N_per_s <= 0
        error('dipActuator:InvalidParameter', 'rate_limit_N_per_s must be positive.');
    end
    if actuator.dead_zone_N < 0
        error('dipActuator:InvalidParameter', 'dead_zone_N must be nonnegative.');
    end
    if actuator.min_force_N >= actuator.max_force_N
        error('dipActuator:InvalidParameter', 'min_force_N must be smaller than max_force_N.');
    end
end

function value = local_get_numeric(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = double(s.(field_name));
    else
        value = double(default_value);
    end
end
