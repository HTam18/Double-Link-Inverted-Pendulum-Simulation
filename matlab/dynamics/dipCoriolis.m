function C = dipCoriolis(theta1, theta2, q_dot, params)

    q_dot = q_dot(:);
    if numel(q_dot) ~= 3
        error('dipCoriolis:InvalidQDot', 'q_dot must have 3 elements.');
    end

    dM = local_mass_matrix_derivatives(theta1, theta2, params);
    C = zeros(3, 1);

    for i = 1:3
        for j = 1:3
            for k = 1:3
                christoffel = 0.5 * (dM{k}(i, j) + dM{j}(i, k) - dM{i}(j, k));
                C(i) = C(i) + christoffel * q_dot(j) * q_dot(k);
            end
        end
    end
end

function dM = local_mass_matrix_derivatives(theta1, theta2, params)
    p = params.physical_parameters;
    m1 = double(p.link1_mass_kg);
    m2 = double(p.link2_mass_kg);
    l1 = double(p.link1_length_m);
    l2 = double(p.link2_length_m);

    r1 = 0.5 * l1;
    r2 = 0.5 * l2;

    a1 = m1 * r1 + m2 * l1;
    a2 = m2 * r2;
    coupling = m2 * l1 * r2;

    d_dx = zeros(3, 3);

    d_dt1 = zeros(3, 3);
    d_dt1(1, 2) = -a1 * sin(theta1);
    d_dt1(2, 1) = -a1 * sin(theta1);
    d_dt1(2, 3) = -coupling * sin(theta1 - theta2);
    d_dt1(3, 2) = -coupling * sin(theta1 - theta2);

    d_dt2 = zeros(3, 3);
    d_dt2(1, 3) = -a2 * sin(theta2);
    d_dt2(3, 1) = -a2 * sin(theta2);
    d_dt2(2, 3) = coupling * sin(theta1 - theta2);
    d_dt2(3, 2) = coupling * sin(theta1 - theta2);

    dM = {d_dx, d_dt1, d_dt2};
end
