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
disp('===== Phase 3: LQR Design =====');
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

if size(C, 2) ~= 6
    error('C matrix must have 6 columns.');
end

if size(D, 1) ~= size(C, 1)
    error('D matrix row size must match C matrix row size.');
end

disp('Matrix size check passed.');

%% Open-loop eigenvalue check

open_loop_poles = eig(A);

disp(' ');
disp('===== Open-loop Poles =====');
disp(open_loop_poles);

if any(real(open_loop_poles) > 0)
    disp('Open-loop result: unstable, as expected for an inverted pendulum.');
else
    warning('Open-loop system does not show positive real poles. Check model signs and angle convention.');
end

%% Controllability check

Co = ctrb(A, B);
rank_Co = rank(Co);
number_of_states = size(A, 1);

disp(' ');
disp('===== Controllability Check =====');
disp(['Controllability rank: ', num2str(rank_Co)]);
disp(['Number of states: ', num2str(number_of_states)]);

if rank_Co ~= number_of_states
    error('The linearized system is not controllable. LQR design cannot continue. Check A and B.');
end

disp('The linearized system is controllable.');

%% Choose LQR weighting matrices
% State vector:
% X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
%
% Q penalizes state error.
% R penalizes control effort.
%
% The angle states theta1 and theta2 are weighted strongly because the main
% goal is to keep both links close to the upright equilibrium.

Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;

if ~isequal(size(Q), [6, 6])
    error('Q matrix size is wrong. Expected 6x6.');
end

if ~isequal(size(R), [1, 1])
    error('R matrix size is wrong. Expected 1x1 for this single-input system.');
end

if any(eig(Q) < 0)
    error('Q must be positive semidefinite.');
end

if any(eig(R) <= 0)
    error('R must be positive definite.');
end

disp(' ');
disp('===== LQR Weighting Matrices =====');
disp('Q = ');
disp(Q);
disp('R = ');
disp(R);

%% Calculate LQR gain

K = lqr(A, B, Q, R);

if ~isequal(size(K), [1, 6])
    error('LQR gain K size is wrong. Expected 1x6.');
end

if any(~isfinite(K), 'all')
    error('LQR gain K contains non-finite values.');
end

disp(' ');
disp('===== LQR Gain =====');
disp('K = ');
disp(K);

disp('Control law: u = -K*x');

%% Closed-loop pole check

closed_loop_A = A - B*K;
closed_loop_poles = eig(closed_loop_A);
closed_loop_stable = all(real(closed_loop_poles) < 0);

disp(' ');
disp('===== Closed-loop Poles =====');
disp(closed_loop_poles);

if closed_loop_stable
    disp('Closed-loop result: stable. All closed-loop poles have negative real parts.');
else
    error('Closed-loop system is not stable. Tune Q/R or check model signs.');
end

%% Save Phase 3 results

results_folder = fullfile(project_root, 'results', 'phase3_lqr_design');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

save(fullfile(results_folder, 'phase3_lqr_design_data.mat'), ...
    'A', 'B', 'C', 'D', 'model_info', ...
    'Q', 'R', 'K', ...
    'open_loop_poles', 'closed_loop_A', 'closed_loop_poles', 'closed_loop_stable', ...
    'Co', 'rank_Co', 'number_of_states');

summary_file = fullfile(results_folder, 'lqr_design_summary.txt');
fid = fopen(summary_file, 'w');

if fid == -1
    warning('Could not create LQR summary text file. MAT file was still saved.');
else
    fprintf(fid, 'Phase 3: LQR Controller Design\n');
    fprintf(fid, '================================\n\n');
    fprintf(fid, 'State vector:\n');
    fprintf(fid, 'X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]\n\n');
    fprintf(fid, 'Control law:\n');
    fprintf(fid, 'u = -K*x\n\n');
    fprintf(fid, 'Controllability rank: %d\n', rank_Co);
    fprintf(fid, 'Number of states: %d\n\n', number_of_states);
    fprintf(fid, 'Q matrix:\n%s\n\n', mat2str(Q, 6));
    fprintf(fid, 'R matrix:\n%s\n\n', mat2str(R, 6));
    fprintf(fid, 'LQR gain K:\n%s\n\n', mat2str(K, 6));
    fprintf(fid, 'Open-loop poles:\n%s\n\n', mat2str(open_loop_poles, 6));
    fprintf(fid, 'Closed-loop poles:\n%s\n\n', mat2str(closed_loop_poles, 6));
    fprintf(fid, 'Closed-loop stable: %d\n', closed_loop_stable);
    fclose(fid);
end

disp(' ');
disp('===== Phase 3 Result =====');
disp('LQR design data saved to results/phase3_lqr_design/phase3_lqr_design_data.mat');
disp('LQR summary saved to results/phase3_lqr_design/lqr_design_summary.txt');
disp('Phase 3 script finished successfully.');
