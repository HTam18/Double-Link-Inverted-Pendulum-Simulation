function D = dip_damping(q_dot, params)
% DIP_DAMPING Return simple viscous damping terms for q = [x, theta1, theta2].
%
% Phase 20 nonlinear MATLAB plant port.
% q_dot order:
%   [x_dot, theta1_dot, theta2_dot]

    q_dot = q_dot(:);
    if numel(q_dot) ~= 3
        error('dip_damping:InvalidQDot', 'q_dot must have 3 elements.');
    end

    p = params.physical_parameters;
    cart_damping = double(p.cart_damping_n_s_per_m);
    link1_damping = double(p.link1_damping_n_m_s_per_rad);
    link2_damping = double(p.link2_damping_n_m_s_per_rad);

    D = [cart_damping * q_dot(1); ...
         link1_damping * q_dot(2); ...
         link2_damping * q_dot(3)];
end
