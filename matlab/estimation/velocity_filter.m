function [v_est, filter_state, info] = velocity_filter(position_meas, filter_state, dt, params)
% VELOCITY_FILTER Estimate velocities from noisy position/angle measurements.
%
% Phase 26 purpose:
%   Estimate [x_dot, theta1_dot, theta2_dot] from measured positions
%   [x, theta1, theta2] using a filtered finite difference. This is a
%   simple, stable estimator used before adding a full EKF.
%
% Input:
%   position_meas : 3x1 vector [x; theta1; theta2]
%   filter_state  : struct with prev_position and velocity
%   dt            : sample time in seconds
%   params        : project parameter struct
%
% Output:
%   v_est         : 3x1 estimated velocities
%   filter_state  : updated filter state
%   info          : diagnostic struct

    if nargin < 4 || isempty(params)
        params = load_params();
    end
    if nargin < 3 || isempty(dt) || dt <= 0
        error('velocity_filter:InvalidDt', 'dt must be positive.');
    end

    position_meas = double(position_meas(:));
    if numel(position_meas) ~= 3
        error('velocity_filter:InvalidMeasurement', 'position_meas must have 3 elements.');
    end

    alpha = local_get_alpha(params);

    if nargin < 2 || isempty(filter_state) || ~isstruct(filter_state)
        filter_state = struct();
    end

    if ~isfield(filter_state, 'initialized') || ~filter_state.initialized
        filter_state.initialized = true;
        filter_state.prev_position = position_meas;
        filter_state.velocity = zeros(3, 1);
        v_est = filter_state.velocity;
        info = struct('raw_velocity', zeros(3, 1), 'alpha', alpha, 'initialized_now', true);
        return;
    end

    prev_position = double(filter_state.prev_position(:));
    prev_velocity = double(filter_state.velocity(:));

    if numel(prev_position) ~= 3 || numel(prev_velocity) ~= 3
        error('velocity_filter:InvalidState', 'filter_state is malformed.');
    end

    % For angular channels, use wrapped angle difference to avoid a velocity
    % spike around +/-pi. x is a linear position, so no wrap is applied.
    delta = position_meas - prev_position;
    delta(2) = local_wrap_to_pi(position_meas(2) - prev_position(2));
    delta(3) = local_wrap_to_pi(position_meas(3) - prev_position(3));

    raw_velocity = delta ./ dt;
    v_est = alpha .* prev_velocity + (1.0 - alpha) .* raw_velocity;

    filter_state.prev_position = position_meas;
    filter_state.velocity = v_est;
    filter_state.initialized = true;

    if any(~isfinite(v_est))
        error('velocity_filter:NonFiniteOutput', 'Estimated velocity contains NaN or Inf.');
    end

    info = struct();
    info.raw_velocity = raw_velocity;
    info.alpha = alpha;
    info.initialized_now = false;
end

function alpha = local_get_alpha(params)
    alpha = 0.85;
    if isfield(params, 'estimator') && isfield(params.estimator, 'velocity_filter_alpha')
        alpha = double(params.estimator.velocity_filter_alpha);
    end
    alpha = min(max(alpha, 0.0), 0.999);
end

function y = local_wrap_to_pi(angle_rad)
    y = atan2(sin(angle_rad), cos(angle_rad));
end
