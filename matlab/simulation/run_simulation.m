function result = run_simulation(sim_config, params)
% RUN_SIMULATION Reusable MATLAB simulation runner for the nonlinear plant.
%
% Phase 21 purpose:
%   - Provide one reusable MATLAB pipeline for open-loop and simple closed-loop tests.
%   - Keep plant, controller, logger, and plotting separated.
%   - Avoid calling Python for basic MATLAB simulation.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.
%
% Controller contract:
%   The controller handle receives (t, state, params, sim_config).
%   It may return either:
%     1) scalar u_cmd
%     2) struct with fields u_cmd and optional mode, measurement, x_hat,
%        external_generalized_force.
%
% Phase 23 addition:
%   sim_config.realism.rail_limit_enabled enables the MATLAB rail limit.
%
% Phase 24 addition:
%   sim_config.realism.actuator_enabled enables a first-order actuator model.
%   The controller output is logged as u_cmd. The nonlinear plant receives
%   u_actual, which is delayed/rate-limited/saturated by dip_actuator.m.
%   The plant state order is unchanged; actuator force is logged separately.
%
% Phase 25 addition:
%   sim_config.realism.sensor_noise_enabled adds noise to measurement.
%   sim_config.realism.use_measurement_for_control sends measurement to the
%   controller instead of the clean state.
%
% Phase 26 addition:
%   sim_config.realism.estimator_enabled enables state_estimator_simple.m.
%   sim_config.realism.use_estimator_for_control sends x_hat to the controller
%   instead of raw measurement or clean state.

    if nargin < 2 || isempty(params)
        params = load_params();
    end
    if nargin < 1 || isempty(sim_config)
        sim_config = struct();
    end

    sim_config = local_fill_defaults(sim_config, params);
    params = local_apply_realism_config(params, sim_config);
    local_initialize_noise_if_needed(params, sim_config);

    dt = double(sim_config.dt);
    t_final = double(sim_config.t_final);
    if dt <= 0 || t_final <= 0
        error('run_simulation:InvalidTimeConfig', 'dt and t_final must be positive.');
    end

    t = (0:dt:t_final).';
    n = numel(t);

    state_hist = zeros(n, 6);
    u_cmd_hist = zeros(n, 1);
    u_actual_hist = zeros(n, 1);
    actuator_rate_hist = zeros(n, 1);
    measurement_hist = nan(n, 6);
    x_hat_hist = nan(n, 6);
    mode_hist = strings(n, 1);

    x0 = double(sim_config.initial_state(:));
    if numel(x0) ~= 6
        error('run_simulation:InvalidInitialState', 'initial_state must have 6 elements.');
    end
    state_hist(1, :) = x0.';

    actuator_enabled = local_actuator_enabled(params);
    estimator_enabled = local_estimator_enabled(params);
    F_actual = local_initial_actual_force(sim_config, params);
    estimator_state = struct();

    for k = 1:(n - 1)
        tk = t(k);
        xk = state_hist(k, :).';

        measurement_k = local_make_measurement(xk, params);
        if estimator_enabled
            [x_hat_k, estimator_state, estimator_info] = state_estimator_simple(measurement_k, estimator_state, dt, params);
        else
            x_hat_k = measurement_k;
            estimator_info = struct();
        end
        x_for_control = local_get_controller_state(xk, measurement_k, x_hat_k, params);

        [u_cmd, step_info] = local_call_controller(sim_config.controller, tk, x_for_control, params, sim_config);
        step_info = local_attach_estimator_info(step_info, x_hat_k, estimator_info, estimator_enabled);

        if actuator_enabled
            [F_next, actuator_info] = dip_actuator(F_actual, u_cmd, dt, params);
            u_actual = F_actual;
            F_actual = F_next;
            actuator_rate_hist(k) = actuator_info.F_dot_limited;
        else
            u_actual = local_saturate_force(u_cmd, params);
            actuator_rate_hist(k) = 0.0;
        end

        q_ext = local_get_external_generalized_force(step_info);

        u_cmd_hist(k) = u_cmd;
        u_actual_hist(k) = u_actual;
        mode_hist(k) = local_get_mode(step_info, sim_config.mode);
        measurement_hist(k, :) = local_get_logged_measurement(step_info, measurement_k, params).';
        x_hat_hist(k, :) = local_get_logged_x_hat(step_info, x_hat_k, x_for_control, params).';

        % Hold u_actual constant over this RK4 step. This keeps the
        % controller/logger contract deterministic. Actuator state is updated
        % once per sample by dip_actuator.m.
        f = @(x) dip_dynamics_nonlinear(x, u_actual, params, q_ext);
        k1 = f(xk);
        k2 = f(xk + 0.5 * dt * k1);
        k3 = f(xk + 0.5 * dt * k2);
        k4 = f(xk + dt * k3);
        x_next = xk + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);

        if any(~isfinite(x_next))
            error('run_simulation:NonFiniteState', ...
                  'State became NaN or Inf at step %d, t = %.6f.', k, tk);
        end

        state_hist(k + 1, :) = x_next.';
    end

    % Log final sample using the final state.
    x_end = state_hist(end, :).';
    measurement_end = local_make_measurement(x_end, params);
    if estimator_enabled
        [x_hat_end, estimator_state, estimator_info_end] = state_estimator_simple(measurement_end, estimator_state, dt, params); %#ok<ASGLU>
    else
        x_hat_end = measurement_end;
        estimator_info_end = struct();
    end
    x_end_for_control = local_get_controller_state(x_end, measurement_end, x_hat_end, params);
    [u_cmd_end, step_info_end] = local_call_controller(sim_config.controller, t(end), x_end_for_control, params, sim_config);
    step_info_end = local_attach_estimator_info(step_info_end, x_hat_end, estimator_info_end, estimator_enabled);
    u_cmd_hist(end) = u_cmd_end;
    if actuator_enabled
        u_actual_hist(end) = F_actual;
        actuator_rate_hist(end) = 0.0;
    else
        u_actual_hist(end) = local_saturate_force(u_cmd_end, params);
        actuator_rate_hist(end) = 0.0;
    end
    mode_hist(end) = local_get_mode(step_info_end, sim_config.mode);
    measurement_hist(end, :) = local_get_logged_measurement(step_info_end, measurement_end, params).';
    x_hat_hist(end, :) = local_get_logged_x_hat(step_info_end, x_hat_end, x_end_for_control, params).';

    result = struct();
    result.t = t;
    result.state = state_hist;
    result.u_cmd = u_cmd_hist;
    result.u_actual = u_actual_hist;
    result.actuator_rate = actuator_rate_hist;
    result.mode = mode_hist;
    result.measurement = measurement_hist;
    result.x_hat = x_hat_hist;
    result.sim_config = sim_config;
    result.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    result.angle_convention = 'theta = 0 downward, theta = pi upright';
    result.params = params;
    result.pass = all(isfinite(state_hist(:))) && all(isfinite(u_actual_hist(:))) && all(isfinite(u_cmd_hist(:))) && all(isfinite(measurement_hist(:)));
end

function params = local_apply_realism_config(params, sim_config)
    if ~isfield(params, 'realism') || isempty(params.realism)
        params.realism = struct();
    end
    if isfield(sim_config, 'realism') && ~isempty(sim_config.realism)
        realism_fields = fieldnames(sim_config.realism);
        for i = 1:numel(realism_fields)
            field_name = realism_fields{i};
            params.realism.(field_name) = sim_config.realism.(field_name);
        end
    end
end


function local_initialize_noise_if_needed(params, sim_config)
    if local_sensor_noise_enabled(params)
        if isfield(sim_config, 'noise_seed') && ~isempty(sim_config.noise_seed)
            seed = double(sim_config.noise_seed);
        elseif isfield(params, 'sensor_noise') && isfield(params.sensor_noise, 'seed')
            seed = double(params.sensor_noise.seed);
        else
            seed = 25;
        end
        rng(seed, 'twister');
    end
end

function measurement = local_make_measurement(x_true, params)
    x_true = double(x_true(:));
    if ~local_sensor_noise_enabled(params)
        measurement = x_true;
        return;
    end

    n = params.sensor_noise;
    sigma = [local_get_number(n, 'x_std_m', 0.001); ...
             local_get_number(n, 'x_dot_std_m_s', 0.005); ...
             local_get_number(n, 'theta1_std_rad', 0.001); ...
             local_get_number(n, 'theta1_dot_std_rad_s', 0.005); ...
             local_get_number(n, 'theta2_std_rad', 0.001); ...
             local_get_number(n, 'theta2_dot_std_rad_s', 0.005)];
    measurement = x_true + sigma .* randn(6, 1);

    if any(~isfinite(measurement))
        error('run_simulation:NonFiniteMeasurement', 'Measurement contains NaN or Inf.');
    end
end

function x_for_control = local_get_controller_state(x_true, measurement, x_hat, params)
    if local_use_estimator_for_control(params)
        x_for_control = x_hat(:);
    elseif local_use_measurement_for_control(params)
        x_for_control = measurement(:);
    else
        x_for_control = x_true(:);
    end
end


function enabled = local_estimator_enabled(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'estimator_enabled')
        enabled = logical(params.realism.estimator_enabled);
    end
end

function enabled = local_use_estimator_for_control(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'use_estimator_for_control')
        enabled = logical(params.realism.use_estimator_for_control);
    end
end

function enabled = local_sensor_noise_enabled(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'sensor_noise_enabled')
        enabled = logical(params.realism.sensor_noise_enabled);
    end
end

function enabled = local_use_measurement_for_control(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'use_measurement_for_control')
        enabled = logical(params.realism.use_measurement_for_control);
    end
end

function value = local_get_number(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = double(s.(field_name));
    else
        value = double(default_value);
    end
end

function sim_config = local_fill_defaults(sim_config, params)
    if ~isfield(sim_config, 'mode') || isempty(sim_config.mode)
        sim_config.mode = 'open_loop';
    end
    if ~isfield(sim_config, 'dt') || isempty(sim_config.dt)
        sim_config.dt = double(params.simulation.time_step_s);
    end
    if ~isfield(sim_config, 't_final') || isempty(sim_config.t_final)
        sim_config.t_final = double(params.simulation.simulation_time_s);
    end
    if ~isfield(sim_config, 'initial_state') || isempty(sim_config.initial_state)
        s0 = params.default_initial_state;
        sim_config.initial_state = [s0.x_m; s0.x_dot_m_s; s0.theta1_rad; ...
                                    s0.theta1_dot_rad_s; s0.theta2_rad; s0.theta2_dot_rad_s];
    end
    if ~isfield(sim_config, 'controller') || isempty(sim_config.controller)
        sim_config.controller = @(t, x, params, cfg) 0.0;
    end
end

function [u_cmd, info] = local_call_controller(controller, t, x, params, sim_config)
    raw = controller(t, x, params, sim_config);
    info = struct();

    if isstruct(raw)
        if isfield(raw, 'u_cmd')
            u_cmd = double(raw.u_cmd);
        elseif isfield(raw, 'u')
            u_cmd = double(raw.u);
        else
            error('run_simulation:ControllerMissingInput', ...
                  'Controller struct output must include u_cmd or u.');
        end
        info = raw;
    else
        u_cmd = double(raw);
    end

    if ~isscalar(u_cmd) || ~isfinite(u_cmd)
        error('run_simulation:InvalidControllerInput', ...
              'Controller must return a finite scalar input.');
    end
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end

function enabled = local_actuator_enabled(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'actuator_enabled')
        enabled = logical(params.realism.actuator_enabled);
    end
end

function F0 = local_initial_actual_force(sim_config, params)
    if isfield(sim_config, 'initial_u_actual') && ~isempty(sim_config.initial_u_actual)
        F0 = double(sim_config.initial_u_actual);
    else
        F0 = 0.0;
    end
    F0 = local_saturate_force(F0, params);
end

function mode = local_get_mode(info, default_mode)
    if isstruct(info) && isfield(info, 'mode') && ~isempty(info.mode)
        mode = string(info.mode);
    else
        mode = string(default_mode);
    end
end



function info = local_attach_estimator_info(info, x_hat, estimator_info, estimator_enabled)
    if ~isstruct(info)
        info = struct();
    end
    if estimator_enabled
        info.x_hat = x_hat(:);
        info.estimator_info = estimator_info;
    elseif ~isfield(info, 'x_hat') || isempty(info.x_hat)
        info.x_hat = x_hat(:);
    end
end

function x_hat = local_get_logged_x_hat(info, generated_x_hat, fallback_value, params)
    if local_estimator_enabled(params)
        x_hat = generated_x_hat(:);
    else
        x_hat = local_get_vector_field(info, 'x_hat', fallback_value);
    end
end

function measurement = local_get_logged_measurement(info, generated_measurement, params)
    % When sensor noise is enabled, the runner-generated measurement is the
    % source of truth for logging. This prevents controllers that echo their
    % input state from accidentally hiding the noisy measurement.
    if local_sensor_noise_enabled(params)
        measurement = generated_measurement(:);
    else
        measurement = local_get_vector_field(info, 'measurement', generated_measurement);
    end
end

function v = local_get_vector_field(info, field_name, default_value)
    if isstruct(info) && isfield(info, field_name) && ~isempty(info.(field_name))
        v = double(info.(field_name)(:));
        if numel(v) ~= 6
            error('run_simulation:InvalidLoggedVector', '%s must have 6 elements.', field_name);
        end
    else
        v = default_value(:);
    end
end

function q_ext = local_get_external_generalized_force(info)
    q_ext = zeros(3, 1);
    if isstruct(info) && isfield(info, 'external_generalized_force') && ~isempty(info.external_generalized_force)
        q_ext = double(info.external_generalized_force(:));
        if numel(q_ext) ~= 3
            error('run_simulation:InvalidExternalForce', ...
                  'external_generalized_force must have 3 elements.');
        end
    end
end
