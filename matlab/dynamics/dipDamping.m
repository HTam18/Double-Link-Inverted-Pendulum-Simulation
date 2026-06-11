function D = dipDamping(q_dot, params)

    q_dot = q_dot(:);
    if numel(q_dot) ~= 3
        error('dipDamping:InvalidQDot', 'q_dot must have 3 elements.');
    end

    p = params.physical_parameters;
    cart_damping = double(p.cart_damping_n_s_per_m);
    link1_damping = double(p.link1_damping_n_m_s_per_rad);
    link2_damping = double(p.link2_damping_n_m_s_per_rad);

    D = [cart_damping * q_dot(1); ...
         link1_damping * q_dot(2); ...
         link2_damping * q_dot(3)];
end
