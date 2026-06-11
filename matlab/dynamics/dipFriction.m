function friction_force = dipFriction(q_dot, params)

    q_dot = double(q_dot(:));
    if numel(q_dot) ~= 3
        error('dipFriction:InvalidVelocity', 'q_dot must have 3 elements.');
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
        error('dipFriction:NonFiniteForce', 'friction force contains NaN or Inf.');
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
