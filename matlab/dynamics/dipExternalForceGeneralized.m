function out = dipExternalForceGeneralized(state, event, params)

    if nargin < 3 || isempty(params)
        params = loadParams();
    end

    state = state(:);
    if numel(state) ~= 6
        error('dipExternalForceGeneralized:InvalidState', 'state must have 6 elements.');
    end
    if nargin < 2 || isempty(event) || ~isstruct(event)
        error('dipExternalForceGeneralized:InvalidEvent', 'event must be a struct.');
    end

    link_id = local_get_field(event, 'link_id', 1);
    contact_ratio = local_get_field(event, 'contact_ratio', 1.0);
    raw_force_N = double(local_get_field(event, 'force_N', 0.0));
    direction_rad = double(local_get_field(event, 'direction_rad', 0.0));

    max_external_force_N = local_max_external_force(params);
    force_N = min(max(raw_force_N, -max_external_force_N), max_external_force_N);
    saturated = abs(force_N - raw_force_N) > 1e-12;

    force_xy_N = [force_N * cos(direction_rad); force_N * sin(direction_rad)];

    contact = dipContactPointKinematics(state, link_id, contact_ratio, params);
    Q_external = contact.jacobian_xy_q.' * force_xy_N;

    if any(~isfinite(Q_external)) || any(~isfinite(force_xy_N))
        error('dipExternalForceGeneralized:NonFiniteOutput', 'External force conversion returned NaN or Inf.');
    end

    out = struct();
    out.Q_external = Q_external;
    out.force_xy_N = force_xy_N;
    out.raw_force_N = raw_force_N;
    out.force_N = force_N;
    out.saturated = saturated;
    out.max_external_force_N = max_external_force_N;
    out.contact = contact;
    out.event = event;
    out.q_order = {'x', 'theta1', 'theta2'};
    out.source = 'matlab/dynamics/dipExternalForceGeneralized.m';
end

function value = local_get_field(s, name, default_value)
    if isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = default_value;
    end
end

function max_force = local_max_external_force(params)
    max_force = 5.0;
    if isfield(params, 'external_force') && isfield(params.external_force, 'max_external_force_N')
        max_force = double(params.external_force.max_external_force_N);
    end
    if ~(isfinite(max_force) && max_force > 0.0)
        error('dipExternalForceGeneralized:InvalidMaxExternalForce', ...
              'params.external_force.max_external_force_N must be positive and finite.');
    end
end
