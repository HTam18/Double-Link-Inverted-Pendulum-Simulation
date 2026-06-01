function y = phase32_hybrid_step(t, reset_flag)
% PHASE32_HYBRID_STEP One interpreted simulation step for the Phase 32 model.
%
% This function is intentionally called from a Simulink MATLAB Function block
% through coder.extrinsic. It keeps the validated Phase 29/30 logic in MATLAB
% instead of duplicating controller or plant equations inside the .slx file.
%
% Output vector y has 34 elements:
%   01 time
%   02:07 true state [x x_dot theta1 theta1_dot theta2 theta2_dot]
%   08 u_cmd
%   09 u_raw
%   10 u_actual
%   11 mode_id: idle=1, tracking=2, stabilize=3, recovery=4, failsafe=5
%   12 handoff_flag
%   13:18 reference state
%   19:24 tracking error state
%   25:30 target error state
%   31 final angle error
%   32 velocity norm
%   33 saturation fraction so far
%   34 fail_code: none=0, tracking=1, stabilize=2, rail=3, force=4, failsafe=5

    persistent cfg params x_state F_actual hybrid_state last_time last_output initialized

    if nargin < 2
        reset_flag = false;
    end

    if isempty(t)
        t = 0.0;
    end
    t = double(t);
    reset_flag = logical(reset_flag);

    if reset_flag || isempty(initialized) || ~initialized
        cfg = phase32_load_config();
        params = local_phase32_params(cfg.params, cfg);
        x_state = double(cfg.initial_state(:));
        F_actual = local_initial_actuator_force(cfg.tracker);
        hybrid_state = struct();
        last_time = 0.0;
        initialized = true;
        last_output = local_make_output(0.0, x_state, 0.0, 0.0, F_actual, 'idle', false, ...
            cfg.initial_state(:), zeros(6,1), zeros(6,1), NaN, NaN, 0.0, 'none');
        y = last_output;
        return;
    end

    if t < last_time - 1e-12
        % Simulink can call the block during model initialization or rewind.
        y = phase32_hybrid_step([], true);
        return;
    end

    if t > last_time + 1e-12
        dt_total = t - last_time;
        nominal_dt = double(cfg.dt);
        n_steps = max(1, round(dt_total / nominal_dt));
        h = dt_total / n_steps;
        for i = 1:n_steps
            tk = last_time + (i - 1) * h;
            sim_config = cfg;
            sim_config.hybrid_reset = (last_time == 0.0 && i == 1);
            sim_config.current_F_actual = F_actual;
            x_control = local_controller_measurement(x_state, cfg);
            [ctrl, hybrid_state] = hybrid_controller_matlab(tk, x_control, params, sim_config, hybrid_state);
            [x_state, F_actual] = local_step_plant_with_actuator(x_state, F_actual, ctrl.u_cmd, h, cfg.integration_substeps_per_interval, params);
        end
        last_time = t;
    else
        sim_config = cfg;
        sim_config.hybrid_reset = false;
        sim_config.current_F_actual = F_actual;
        x_control = local_controller_measurement(x_state, cfg);
        [ctrl, hybrid_state] = hybrid_controller_matlab(t, x_control, params, sim_config, hybrid_state);
    end

    if ~exist('ctrl', 'var')
        sim_config = cfg;
        sim_config.hybrid_reset = false;
        sim_config.current_F_actual = F_actual;
        [ctrl, hybrid_state] = hybrid_controller_matlab(t, local_controller_measurement(x_state, cfg), params, sim_config, hybrid_state);
    end

    last_output = local_make_output(t, x_state, ctrl.u_cmd, ctrl.u_raw, F_actual, ctrl.hybrid_mode, ctrl.handoff_flag, ...
        ctrl.x_ref, ctrl.tracking_error_state, ctrl.target_error_state, ctrl.final_angle_error_rad, ...
        ctrl.velocity_norm, ctrl.saturation_fraction_so_far, ctrl.fail_reason);
    y = last_output;
end

function params_out = local_phase32_params(params, cfg)
    params_out = params;
    params_out.realism.rail_limit_enabled = false;
    params_out.realism.friction_enabled = false;
    params_out.realism.sensor_noise_enabled = logical(cfg.sensor_noise_enabled);
    params_out.realism.estimator_enabled = false;
    params_out.realism.use_measurement_for_control = logical(cfg.sensor_noise_enabled);
    params_out.realism.use_estimator_for_control = false;
    params_out.realism.actuator_enabled = true;
    params_out.actuator.tau_motor_s = cfg.actuator_tau_motor_s;
    params_out.actuator.rate_limit_N_per_s = cfg.actuator_rate_limit_N_per_s;
    params_out.actuator.dead_zone_N = 0.0;
    params_out.actuator.min_force_N = double(params.control.min_cart_force_N);
    params_out.actuator.max_force_N = double(params.control.max_cart_force_N);
end

function F0 = local_initial_actuator_force(tracker)
    if isfield(tracker, 'F_actual_ref') && ~isempty(tracker.F_actual_ref)
        F0 = double(tracker.F_actual_ref(1));
    elseif isfield(tracker, 'u_ref') && ~isempty(tracker.u_ref)
        F0 = double(tracker.u_ref(1));
    elseif isfield(tracker, 'u_cmd_ref') && ~isempty(tracker.u_cmd_ref)
        F0 = double(tracker.u_cmd_ref(1));
    else
        F0 = 0.0;
    end
end

function x_meas = local_controller_measurement(x_true, cfg)
    x_meas = double(x_true(:));
    if isfield(cfg, 'sensor_noise_enabled') && logical(cfg.sensor_noise_enabled)
        x_meas = x_meas + double(cfg.sensor_noise_std(:)) .* randn(6, 1);
    end
end

function [x_next, F_actual] = local_step_plant_with_actuator(x, F_actual, F_cmd, dt, substeps, params)
    x_next = double(x(:));
    n_sub = max(1, round(double(substeps)));
    h = double(dt) / n_sub;
    for j = 1:n_sub
        [F_actual, ~] = dip_actuator(F_actual, F_cmd, h, params);
        x_next = local_rk4_zoh_step(x_next, F_actual, h, params);
    end
end

function x_next = local_rk4_zoh_step(x, u, dt, params)
    x_next = double(x(:));
    f = @(z) dip_dynamics_nonlinear(z, u, params, zeros(3, 1));
    k1 = f(x_next);
    k2 = f(x_next + 0.5*dt*k1);
    k3 = f(x_next + 0.5*dt*k2);
    k4 = f(x_next + dt*k3);
    x_next = x_next + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
end

function y = local_make_output(t, x, u_cmd, u_raw, u_actual, mode, handoff, x_ref, tracking_error, target_error, final_angle, velocity_norm, saturation_fraction, fail_reason)
    y = zeros(34, 1);
    y(1) = double(t);
    y(2:7) = double(x(:));
    y(8) = double(u_cmd);
    y(9) = double(u_raw);
    y(10) = double(u_actual);
    y(11) = local_mode_to_id(mode);
    y(12) = double(logical(handoff));
    y(13:18) = local_safe_vec(x_ref, 6);
    y(19:24) = local_safe_vec(tracking_error, 6);
    y(25:30) = local_safe_vec(target_error, 6);
    y(31) = double(final_angle);
    y(32) = double(velocity_norm);
    y(33) = double(saturation_fraction);
    y(34) = local_fail_to_code(fail_reason);
end

function v = local_safe_vec(x, n)
    v = zeros(n, 1);
    if nargin < 2
        n = numel(v);
    end
    if ~isempty(x)
        xx = double(x(:));
        m = min(n, numel(xx));
        v(1:m) = xx(1:m);
    end
end

function id = local_mode_to_id(mode)
    mode = char(string(mode));
    switch mode
        case 'idle'
            id = 1;
        case 'trajectory_tracking'
            id = 2;
        case 'stabilize'
            id = 3;
        case 'recovery'
            id = 4;
        case 'failsafe'
            id = 5;
        otherwise
            id = 0;
    end
end

function code = local_fail_to_code(reason)
    reason = char(string(reason));
    if isempty(reason) || strcmp(reason, 'none')
        code = 0;
    elseif contains(reason, 'tracking') || contains(reason, 'handoff')
        code = 1;
    elseif contains(reason, 'stabilize')
        code = 2;
    elseif contains(reason, 'rail')
        code = 3;
    elseif contains(reason, 'force') || contains(reason, 'saturation')
        code = 4;
    elseif contains(reason, 'failsafe') || contains(reason, 'recovery')
        code = 5;
    else
        code = 9;
    end
end
