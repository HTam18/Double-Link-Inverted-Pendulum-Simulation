% Phase 6: Numerical linearization around each target balance mode.
%
% This script is the official MATLAB calculation module for Phase 6.
% It loads shared/parameters.json, linearizes the nonlinear model around
% up_up, up_down, and down_up, prints eigenvalues, and exports the data to:
%   shared/linearization/linearized_models.json
%   shared/linearization/linearized_models.mat
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Input:
%   horizontal cart force u in N

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
param_path = fullfile(project_root, 'shared', 'parameters.json');
output_dir = fullfile(project_root, 'shared', 'linearization');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

params = jsondecode(fileread(param_path));
mode_names = fieldnames(params.target_modes);

state_step = 1e-6;
input_step = 1e-6;
u0 = 0.0;

models = struct();
mat_models = struct();

fprintf('Phase 6 linearization around target modes\n');
fprintf('Parameter file: %s\n\n', param_path);

for k = 1:numel(mode_names)
    mode = mode_names{k};
    target = get_target_from_params(params, mode);
    x0 = target.target_state(:);

    [A, B] = numerical_linearization(@(x, u) nonlinear_dynamics_matlab(x, u, params), x0, u0, state_step, input_step);
    eig_A = eig(A);

    fprintf('Mode: %s\n', mode);
    fprintf('  A size: %d x %d\n', size(A, 1), size(A, 2));
    fprintf('  B size: %d x %d\n', size(B, 1), size(B, 2));
    fprintf('  Eigenvalues of A:\n');
    for i = 1:numel(eig_A)
        fprintf('    %+.6f %+.6fj\n', real(eig_A(i)), imag(eig_A(i)));
    end
    fprintf('\n');

    model.mode = mode;
    model.target_state = x0';
    model.target_angles_rad = [target.theta1_target_rad, target.theta2_target_rad];
    model.u_equilibrium_N = u0;
    model.A = A;
    model.B = B;
    model.eigenvalues = complex_to_struct_array(eig_A);
    models.(mode) = model;

    mat_models.(mode).A = A;
    mat_models.(mode).B = B;
    mat_models.(mode).eigenvalues = eig_A;
    mat_models.(mode).target_state = x0;
end

export_data.source = 'MATLAB finite difference linearization';
export_data.state_order = cellstr(params.state_definition.state_order);
export_data.input_order = {'cart_force_N'};
export_data.angle_convention = params.state_definition.angle_convention;
export_data.finite_difference.state_step = state_step;
export_data.finite_difference.input_step = input_step;
export_data.models = models;

json_path = fullfile(output_dir, 'linearized_models.json');
mat_path = fullfile(output_dir, 'linearized_models.mat');
write_text_file(json_path, jsonencode(export_data, PrettyPrint=true));
save(mat_path, 'mat_models', 'params', 'state_step', 'input_step');

fprintf('Saved JSON: %s\n', json_path);
fprintf('Saved MAT:  %s\n', mat_path);

function [A, B] = numerical_linearization(f, x0, u0, state_step, input_step)
    n = numel(x0);
    A = zeros(n, n);
    B = zeros(n, 1);

    for col = 1:n
        dx = zeros(n, 1);
        dx(col) = state_step;
        f_plus = f(x0 + dx, u0);
        f_minus = f(x0 - dx, u0);
        A(:, col) = (f_plus - f_minus) ./ (2.0 * state_step);
    end

    f_plus = f(x0, u0 + input_step);
    f_minus = f(x0, u0 - input_step);
    B(:, 1) = (f_plus - f_minus) ./ (2.0 * input_step);
end

function target = get_target_from_params(params, mode)
    data = params.target_modes.(mode);
    theta1 = data.theta1_target_rad;
    theta2 = data.theta2_target_rad;
    target.mode = mode;
    target.theta1_target_rad = theta1;
    target.theta2_target_rad = theta2;
    target.target_state = [0; 0; theta1; 0; theta2; 0];
end

function dxdt = nonlinear_dynamics_matlab(state, u, params)
    state = state(:);
    if numel(state) ~= 6
        error('state must have 6 values');
    end

    x_dot = state(2);
    theta1 = state(3);
    theta1_dot = state(4);
    theta2 = state(5);
    theta2_dot = state(6);

    min_force = params.control.min_cart_force_N;
    max_force = params.control.max_cart_force_N;
    force = min(max(u, min_force), max_force);

    q_dot = [x_dot; theta1_dot; theta2_dot];
    generalized_force = [force; 0; 0];

    M = mass_matrix_matlab(theta1, theta2, params);
    rhs = generalized_force ...
        - coriolis_vector_matlab(theta1, theta2, q_dot, params) ...
        - gravity_vector_matlab(theta1, theta2, params) ...
        - damping_vector_matlab(q_dot, params);

    q_ddot = M \ rhs;
    dxdt = [x_dot; q_ddot(1); theta1_dot; q_ddot(2); theta2_dot; q_ddot(3)];

    if any(~isfinite(dxdt))
        error('nonlinear dynamics returned NaN or Inf');
    end
end

function M = mass_matrix_matlab(theta1, theta2, params)
    p = params.physical_parameters;
    mc = p.cart_mass_kg;
    m1 = p.link1_mass_kg;
    m2 = p.link2_mass_kg;
    l1 = p.link1_length_m;
    l2 = p.link2_length_m;
    r1 = 0.5 * l1;
    r2 = 0.5 * l2;
    I1 = (1.0 / 12.0) * m1 * l1^2;
    I2 = (1.0 / 12.0) * m2 * l2^2;

    a1 = m1 * r1 + m2 * l1;
    a2 = m2 * r2;
    coupling = m2 * l1 * r2;

    M = [mc + m1 + m2,     a1 * cos(theta1),                  a2 * cos(theta2);
         a1 * cos(theta1), m1 * r1^2 + I1 + m2 * l1^2, coupling * cos(theta1 - theta2);
         a2 * cos(theta2), coupling * cos(theta1 - theta2),    m2 * r2^2 + I2];
end

function dM = mass_matrix_derivatives_matlab(theta1, theta2, params)
    p = params.physical_parameters;
    m1 = p.link1_mass_kg;
    m2 = p.link2_mass_kg;
    l1 = p.link1_length_m;
    l2 = p.link2_length_m;
    r1 = 0.5 * l1;
    r2 = 0.5 * l2;
    a1 = m1 * r1 + m2 * l1;
    a2 = m2 * r2;
    coupling = m2 * l1 * r2;

    d_dx = zeros(3, 3);
    d_dt1 = zeros(3, 3);
    d_dt2 = zeros(3, 3);

    d_dt1(1, 2) = -a1 * sin(theta1);
    d_dt1(2, 1) = d_dt1(1, 2);
    d_dt1(2, 3) = -coupling * sin(theta1 - theta2);
    d_dt1(3, 2) = d_dt1(2, 3);

    d_dt2(1, 3) = -a2 * sin(theta2);
    d_dt2(3, 1) = d_dt2(1, 3);
    d_dt2(2, 3) = coupling * sin(theta1 - theta2);
    d_dt2(3, 2) = d_dt2(2, 3);

    dM = cat(3, d_dx, d_dt1, d_dt2);
end

function C = coriolis_vector_matlab(theta1, theta2, q_dot, params)
    dM = mass_matrix_derivatives_matlab(theta1, theta2, params);
    C = zeros(3, 1);
    for i = 1:3
        for j = 1:3
            for k = 1:3
                christoffel = 0.5 * (dM(i, j, k) + dM(i, k, j) - dM(j, k, i));
                C(i) = C(i) + christoffel * q_dot(j) * q_dot(k);
            end
        end
    end
end

function G = gravity_vector_matlab(theta1, theta2, params)
    p = params.physical_parameters;
    m1 = p.link1_mass_kg;
    m2 = p.link2_mass_kg;
    l1 = p.link1_length_m;
    l2 = p.link2_length_m;
    r1 = 0.5 * l1;
    r2 = 0.5 * l2;
    g = p.gravity_m_s2;
    G = [0; (m1 * r1 + m2 * l1) * g * sin(theta1); m2 * r2 * g * sin(theta2)];
end

function D = damping_vector_matlab(q_dot, params)
    p = params.physical_parameters;
    D = [p.cart_damping_n_s_per_m * q_dot(1);
         p.link1_damping_n_m_s_per_rad * q_dot(2);
         p.link2_damping_n_m_s_per_rad * q_dot(3)];
end

function out = complex_to_struct_array(values)
    values = values(:);
    out = repmat(struct('real', 0, 'imag', 0), numel(values), 1);
    for i = 1:numel(values)
        out(i).real = real(values(i));
        out(i).imag = imag(values(i));
    end
end

function write_text_file(path, text)
    fid = fopen(path, 'w');
    if fid < 0
        error('Could not open file for writing: %s', path);
    end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', text);
end
