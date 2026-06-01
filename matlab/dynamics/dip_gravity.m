function G = dip_gravity(theta1, theta2, params)
% DIP_GRAVITY Return gravity terms for q = [x, theta1, theta2].
%
% Phase 20 nonlinear MATLAB plant port.
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.

    p = params.physical_parameters;

    m1 = double(p.link1_mass_kg);
    m2 = double(p.link2_mass_kg);
    l1 = double(p.link1_length_m);
    l2 = double(p.link2_length_m);
    g = double(p.gravity_m_s2);

    r1 = 0.5 * l1;
    r2 = 0.5 * l2;

    G = [0.0; ...
         (m1 * r1 + m2 * l1) * g * sin(theta1); ...
         m2 * r2 * g * sin(theta2)];
end
