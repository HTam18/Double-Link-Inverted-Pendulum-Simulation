function state_dot = dipDynamicsNonlinear(state, u, params, external_generalized_force)

    if nargin < 4 || isempty(external_generalized_force)
        external_generalized_force = zeros(3, 1);
    end

    state = state(:);
    if numel(state) ~= 6
        error('dipDynamicsNonlinear:InvalidState', 'state must have 6 elements.');
    end

    external_generalized_force = external_generalized_force(:);
    if numel(external_generalized_force) ~= 3
        error('dipDynamicsNonlinear:InvalidExternalForce', ...
              'external_generalized_force must have 3 elements.');
    end

    x_dot = state(2);
    theta1 = state(3);
    theta1_dot = state(4);
    theta2 = state(5);
    theta2_dot = state(6);

    force = local_saturate_force(u, params);

    q_dot = [x_dot; theta1_dot; theta2_dot];
    generalized_force = [force; 0.0; 0.0] + external_generalized_force;

    if local_rail_limit_enabled(params)
        rail_force = dipRailLimit(state(1), x_dot, params);
        generalized_force(1) = generalized_force(1) + rail_force;
    end

    M = dipMassMatrix(theta1, theta2, params);
    C = dipCoriolis(theta1, theta2, q_dot, params);
    G = dipGravity(theta1, theta2, params);
    D = dipDamping(q_dot, params);
    if local_friction_enabled(params)
        D = D + dipFriction(q_dot, params);
    end

    rhs = generalized_force - C - G - D;
    q_ddot = M \ rhs;

    state_dot = [x_dot; q_ddot(1); theta1_dot; q_ddot(2); theta2_dot; q_ddot(3)];

    if any(~isfinite(state_dot))
        error('dipDynamicsNonlinear:NonFiniteDerivative', ...
              'nonlinear dynamics returned NaN or Inf.');
    end
end

function enabled = local_rail_limit_enabled(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'rail_limit_enabled')
        enabled = logical(params.realism.rail_limit_enabled);
    end
end

function enabled = local_friction_enabled(params)
    enabled = false;
    if isfield(params, 'realism') && isfield(params.realism, 'friction_enabled')
        enabled = logical(params.realism.friction_enabled);
    end
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end
