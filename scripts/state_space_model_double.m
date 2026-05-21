function [A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g)
% state_space_model_double
% Build the linearized state-space model for a double link inverted pendulum
% around the upright equilibrium.
%
% State vector:
% X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
%
% Input:
% u = horizontal force applied to the cart

%% Mass matrix around upright equilibrium

M = [mc + m1 + m2,        m1*lc1 + m2*L1,              m2*lc2;
     m1*lc1 + m2*L1,     m1*lc1^2 + m2*L1^2 + I1,     m2*L1*lc2;
     m2*lc2,             m2*L1*lc2,                   m2*lc2^2 + I2];

%% Damping matrix

damping_matrix = diag([bc, b1, b2]);

%% Gravity stiffness matrix
% The cart position has no gravity term.
% The angle terms are unstable around the upright equilibrium.

g1 = (m1*lc1 + m2*L1)*g;
g2 = m2*lc2*g;

gravity_stiffness = [0,  0,  0;
                     0, g1,  0;
                     0,  0, g2];

%% Input matrix for generalized coordinates
% Force is applied only to the cart.

input_matrix = [1;
                0;
                0];

%% Build state-space matrices
% State order:
% X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
%
% q columns in X are:       x, theta1, theta2              -> 1, 3, 5
% q_dot columns in X are:   x_dot, theta1_dot, theta2_dot  -> 2, 4, 6
%
% Acceleration order:
% q_ddot = [x_ddot; theta1_ddot; theta2_ddot]

A = zeros(6, 6);
B = zeros(6, 1);

% Kinematic relationships
A(1, 2) = 1;
A(3, 4) = 1;
A(5, 6) = 1;

% Dynamic relationships
Minv = M \ eye(size(M));

q_state_columns = [1, 3, 5];
qdot_state_columns = [2, 4, 6];
acc_state_rows = [2, 4, 6];

A(acc_state_rows, q_state_columns) = Minv * gravity_stiffness;
A(acc_state_rows, qdot_state_columns) = Minv * (-damping_matrix);

B(acc_state_rows, 1) = Minv * input_matrix;

%% Output matrix
% Output all states for easy plotting and debugging.

C = eye(6);
D = zeros(6, 1);

%% Store model information

model_info = struct();

model_info.state_vector = 'X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]';
model_info.input = 'u = horizontal force applied to the cart';
model_info.angle_convention = 'theta1 = 0 and theta2 = 0 mean both links are upright';
model_info.coordinate_type = 'absolute angles around upright equilibrium';

model_info.M = M;
model_info.damping_matrix = damping_matrix;
model_info.gravity_stiffness = gravity_stiffness;
model_info.input_matrix = input_matrix;

model_info.A_size = size(A);
model_info.B_size = size(B);
model_info.C_size = size(C);
model_info.D_size = size(D);

model_info.open_loop_eigenvalues = eig(A);
model_info.controllability_matrix = ctrb(A, B);
model_info.controllability_rank = rank(model_info.controllability_matrix);

%% Final safety checks

if ~isequal(size(A), [6, 6])
    error('A matrix must be 6x6.');
end

if ~isequal(size(B), [6, 1])
    error('B matrix must be 6x1.');
end

if size(C, 2) ~= 6
    error('C matrix must have 6 columns.');
end

if size(D, 1) ~= size(C, 1)
    error('D matrix row size must match C matrix row size.');
end

end
