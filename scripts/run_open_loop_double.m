clear; clc; close all;

%% Make paths robust
% This script can be run from the project root or from the scripts folder.
script_dir = fileparts(mfilename('fullpath'));

if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
addpath(script_dir);

%% Load parameters

run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; therefore rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));

if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
addpath(script_dir);

%% Build state-space model

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g);

%% Display matrix sizes

disp(' ');
disp('===== Matrix Size Check =====');
disp(['A size: ', mat2str(size(A))]);
disp(['B size: ', mat2str(size(B))]);
disp(['C size: ', mat2str(size(C))]);
disp(['D size: ', mat2str(size(D))]);

if ~isequal(size(A), [6, 6])
    error('A matrix size is wrong. Expected 6x6.');
end

if ~isequal(size(B), [6, 1])
    error('B matrix size is wrong. Expected 6x1.');
end

disp('Matrix size check passed.');

%% Open-loop eigenvalue check

open_loop_poles = eig(A);

disp(' ');
disp('===== Open-loop Eigenvalues =====');
disp(open_loop_poles);

if any(real(open_loop_poles) > 0)
    disp('Open-loop result: unstable, as expected for an inverted pendulum.');
else
    warning('Open-loop system does not show positive real eigenvalues. Check model signs and angle convention.');
end

%% Controllability check

Co = ctrb(A, B);
rank_Co = rank(Co);

disp(' ');
disp('===== Controllability Check =====');
disp(['Controllability rank: ', num2str(rank_Co)]);
disp(['Number of states: ', num2str(size(A, 1))]);

if rank_Co == size(A, 1)
    disp('The linearized system is controllable.');
else
    error('The linearized system is not controllable. Check A and B.');
end

%% Open-loop simulation

sys_open = ss(A, B, C, D);

t = 0:dt:sim_time;
u = zeros(size(t));

[y, t_out, x_out] = lsim(sys_open, u, t, x0);

%% Extract states

cart_position = x_out(:, 1);
cart_velocity = x_out(:, 2);

theta1 = x_out(:, 3);
theta1_dot = x_out(:, 4);

theta2 = x_out(:, 5);
theta2_dot = x_out(:, 6);

%% Plot results

figure('Name', 'Open-loop Double Link Response');

subplot(3, 1, 1);
plot(t_out, rad2deg(theta1), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta1 (deg)');
title('Open-loop Link 1 Angle');

subplot(3, 1, 2);
plot(t_out, rad2deg(theta2), 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta2 (deg)');
title('Open-loop Link 2 Angle');

subplot(3, 1, 3);
plot(t_out, cart_position, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Open-loop Cart Position');

%% Save results

results_folder = fullfile(project_root, 'results', 'phase2_open_loop');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

save(fullfile(results_folder, 'phase2_open_loop_data.mat'), ...
    'A', 'B', 'C', 'D', 'model_info', ...
    'open_loop_poles', 'Co', 'rank_Co', ...
    't_out', 'x_out', 'y');

saveas(gcf, fullfile(results_folder, 'open_loop_response.png'));

disp(' ');
disp('===== Phase 2 Result =====');
disp('Open-loop data saved to results/phase2_open_loop/phase2_open_loop_data.mat');
disp('Open-loop plot saved to results/phase2_open_loop/open_loop_response.png');
disp('Phase 2 script finished successfully.');
