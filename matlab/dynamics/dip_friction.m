function friction_force = dip_friction(q_dot, params)
% DIP_FRICTION Extra Phase 25 friction model for MATLAB realism mode.
%
% State/order convention:
%   q_dot = [x_dot; theta1_dot; theta2_dot]
%
% Output:
%   friction_force is a 3x1 generalized resisting force vector. It is meant
%   to be added to the existing damping vector D and then subtracted in the
%   equation M*q_ddot = Q - C - G - D.
%
% Model:
%   friction = viscous_extra .* q_dot + coulomb .* tanh(q_dot / v_smooth)
%
% The tanh smoothing avoids a discontinuous sign() term that can make ODE
% solvers unstable near zero velocity.

    q_dot = double(q_dot(:));
    if numel(q_dot) ~= 3
        error('dip_friction:InvalidVelocity', 'q_dot must have 3 elements.');
    end

    friction_force = zeros(3, 1);

    if ~isfield(params, 'friction') || isempty(params.friction)
        return;
    end
    f = params.friction;

    viscous_enabled = local_get_bool(f, 'viscous_enabled', true);
    coulomb_enabled = local_get_bool(f, 'coulomb_enabled', true);

    if viscous_enabled
        viscous = [local_get_number(f, 'cart_viscous_extra_N_s_per_m', 0.0); ...
                   local_get_number(f, 'link1_viscous_extra_N_m_s_per_rad', 0.0); ...
                   local_get_number(f, 'link2_viscous_extra_N_m_s_per_rad', 0.0)];
        friction_force = friction_force + viscous .* q_dot;
    end

    if coulomb_enabled
        coulomb = [local_get_number(f, 'cart_coulomb_N', 0.0); ...
                   local_get_number(f, 'link1_coulomb_N_m', 0.0); ...
                   local_get_number(f, 'link2_coulomb_N_m', 0.0)];
        v_smooth = local_get_number(f, 'coulomb_smoothing_velocity', 0.02);
        v_smooth = max(abs(v_smooth), 1.0e-6);
        friction_force = friction_force + coulomb .* tanh(q_dot ./ v_smooth);
    end

    if any(~isfinite(friction_force))
        error('dip_friction:NonFiniteForce', 'friction force contains NaN or Inf.');
    end
end

function value = local_get_bool(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = logical(s.(field_name));
    else
        value = logical(default_value);
    end
end

function value = local_get_number(s, field_name, default_value)
    if isfield(s, field_name) && ~isempty(s.(field_name))
        value = double(s.(field_name));
    else
        value = double(default_value);
    end
end
