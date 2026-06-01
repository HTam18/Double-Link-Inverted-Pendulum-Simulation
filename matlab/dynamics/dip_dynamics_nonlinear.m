function state_dot = dip_dynamics_nonlinear(state, u, params, external_generalized_force)
% DIP_DYNAMICS_NONLINEAR Nonlinear double-link pendulum on cart dynamics.
%
% Phase 20 MATLAB port of src/dynamics/nonlinear.py.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means the link points downward.
%   theta = pi rad means the link points upright.
%   theta2 is an absolute angle, not a relative angle.
%
% Generalized coordinate order:
%   q     = [x, theta1, theta2]
%   q_dot = [x_dot, theta1_dot, theta2_dot]
%
% Input:
%   u is the commanded cart force in Newtons. It is saturated using
%   params.control.min_cart_force_N and params.control.max_cart_force_N.
%
% Optional input:
%   external_generalized_force is a 3x1 vector for q = [x, theta1, theta2].
%   It is added after cart force saturation and is intended for later contact
%   force/recovery tests.
%
% Phase 23 rail limit:
%   If params.realism.rail_limit_enabled is true, a finite rail force is
%   computed by dip_rail_limit.m and added to the cart generalized force.
%
% Phase 25 friction:
%   If params.realism.friction_enabled is true, additional viscous and
%   smoothed Coulomb friction are computed by dip_friction.m and added to D.

    if nargin < 4 || isempty(external_generalized_force)
        external_generalized_force = zeros(3, 1);
    end

    state = state(:);
    if numel(state) ~= 6
        error('dip_dynamics_nonlinear:InvalidState', 'state must have 6 elements.');
    end

    external_generalized_force = external_generalized_force(:);
    if numel(external_generalized_force) ~= 3
        error('dip_dynamics_nonlinear:InvalidExternalForce', ...
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
        rail_force = dip_rail_limit(state(1), x_dot, params);
        generalized_force(1) = generalized_force(1) + rail_force;
    end

    M = dip_mass_matrix(theta1, theta2, params);
    C = dip_coriolis(theta1, theta2, q_dot, params);
    G = dip_gravity(theta1, theta2, params);
    D = dip_damping(q_dot, params);
    if local_friction_enabled(params)
        D = D + dip_friction(q_dot, params);
    end

    rhs = generalized_force - C - G - D;
    q_ddot = M \ rhs;

    state_dot = [x_dot; q_ddot(1); theta1_dot; q_ddot(2); theta2_dot; q_ddot(3)];

    if any(~isfinite(state_dot))
        error('dip_dynamics_nonlinear:NonFiniteDerivative', ...
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
