function [x_hat, estimator_state, info] = state_estimator_simple(measurement, estimator_state, dt, params)
% STATE_ESTIMATOR_SIMPLE Simple Phase 26 state estimator.
%
% Purpose:
%   Build an estimated state x_hat from noisy measurements. The estimator
%   trusts measured positions/angles and estimates velocities using
%   velocity_filter.m. It is intentionally simple and stable; EKF work is
%   left for a later extension.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.
%
% Measurement contract:
%   measurement is a 6x1 vector in the same state order. Velocity channels
%   may be noisy and are not directly trusted when estimator mode is enabled.

    if nargin < 4 || isempty(params)
        params = load_params();
    end
    if nargin < 3 || isempty(dt) || dt <= 0
        error('state_estimator_simple:InvalidDt', 'dt must be positive.');
    end

    measurement = double(measurement(:));
    if numel(measurement) ~= 6
        error('state_estimator_simple:InvalidMeasurement', 'measurement must have 6 elements.');
    end

    if nargin < 2 || isempty(estimator_state) || ~isstruct(estimator_state)
        estimator_state = struct();
    end
    if ~isfield(estimator_state, 'velocity_filter')
        estimator_state.velocity_filter = struct();
    end

    position_meas = [measurement(1); measurement(3); measurement(5)];
    [velocity_est, estimator_state.velocity_filter, filter_info] = ...
        velocity_filter(position_meas, estimator_state.velocity_filter, dt, params);

    x_hat = zeros(6, 1);
    x_hat(1) = position_meas(1);
    x_hat(2) = velocity_est(1);
    x_hat(3) = position_meas(2);
    x_hat(4) = velocity_est(2);
    x_hat(5) = position_meas(3);
    x_hat(6) = velocity_est(3);

    if local_blend_measured_velocity(params)
        beta = local_get_velocity_measurement_blend(params);
        measured_velocity = [measurement(2); measurement(4); measurement(6)];
        blended_velocity = (1.0 - beta) .* velocity_est + beta .* measured_velocity;
        x_hat(2) = blended_velocity(1);
        x_hat(4) = blended_velocity(2);
        x_hat(6) = blended_velocity(3);
    end

    estimator_state.last_measurement = measurement;
    estimator_state.last_x_hat = x_hat;
    estimator_state.initialized = true;

    if any(~isfinite(x_hat))
        error('state_estimator_simple:NonFiniteOutput', 'x_hat contains NaN or Inf.');
    end

    info = struct();
    info.velocity_estimate = [x_hat(2); x_hat(4); x_hat(6)];
    info.filter_info = filter_info;
    info.position_measurement = position_meas;
end

function enabled = local_blend_measured_velocity(params)
    enabled = false;
    if isfield(params, 'estimator') && isfield(params.estimator, 'blend_measured_velocity')
        enabled = logical(params.estimator.blend_measured_velocity);
    end
end

function beta = local_get_velocity_measurement_blend(params)
    beta = 0.0;
    if isfield(params, 'estimator') && isfield(params.estimator, 'velocity_measurement_blend')
        beta = double(params.estimator.velocity_measurement_blend);
    end
    beta = min(max(beta, 0.0), 1.0);
end
