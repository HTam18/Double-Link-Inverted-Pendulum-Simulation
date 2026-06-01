function M = dip_mass_matrix(theta1, theta2, params)
% DIP_MASS_MATRIX Return the 3x3 mass matrix for q = [x, theta1, theta2].
%
% Phase 20 nonlinear MATLAB plant port.
%
% State order used by the full plant:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means the link points downward.
%   theta = pi rad means the link points upright.
%   theta2 is an absolute angle from the downward vertical direction.

    v = local_physical_values(params);

    mc = v.cart_mass;
    m1 = v.m1;
    m2 = v.m2;
    l1 = v.l1;
    l2 = v.l2;

    r1 = 0.5 * l1;
    r2 = 0.5 * l2;

    % Slender rod inertia about each link center of mass.
    i1 = (1.0 / 12.0) * m1 * l1^2;
    i2 = (1.0 / 12.0) * m2 * l2^2;

    a1 = m1 * r1 + m2 * l1;
    a2 = m2 * r2;
    coupling = m2 * l1 * r2;

    M = [mc + m1 + m2,           a1 * cos(theta1),                       a2 * cos(theta2); ...
         a1 * cos(theta1),       m1 * r1^2 + i1 + m2 * l1^2,             coupling * cos(theta1 - theta2); ...
         a2 * cos(theta2),       coupling * cos(theta1 - theta2),        m2 * r2^2 + i2];
end

function v = local_physical_values(params)
    p = params.physical_parameters;
    v = struct();
    v.cart_mass = double(p.cart_mass_kg);
    v.m1 = double(p.link1_mass_kg);
    v.m2 = double(p.link2_mass_kg);
    v.l1 = double(p.link1_length_m);
    v.l2 = double(p.link2_length_m);
end
