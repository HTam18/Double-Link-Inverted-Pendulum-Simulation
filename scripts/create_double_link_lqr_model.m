clear; clc;

%% create_double_link_lqr_model
% Phase 5/6 helper script.
% This script creates the Simulink closed-loop model automatically.
% The model includes actuator saturation and an optional external force
% disturbance input for Phase 6 tests.
%
% Output model:
% models/double_link_lqr.slx
%
% Control law:
% u_raw = -K*x
% u_sat = saturation(u_raw, -u_max, +u_max)
% u_total = u_sat + disturbance_force
%
% State vector:
% X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]

%% Make paths robust
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
addpath(script_dir);

if ~exist(models_folder, 'dir')
    mkdir(models_folder);
end

%% Load required workspace variables
run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
models_folder = fullfile(project_root, 'models');
addpath(script_dir);

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g); %#ok<NASGU>

% Selected C13 controller.
Q = diag([8, 1, 40, 2, 40, 2]);
R = 2.0;
K = lqr(A, B, Q, R);

%% Safety checks before model creation
if ~isequal(size(A), [6, 6])
    error('A matrix must be 6x6.');
end
if ~isequal(size(B), [6, 1])
    error('B matrix must be 6x1.');
end
if ~isequal(size(C), [6, 6])
    error('C matrix must be 6x6 because the Simulink model outputs all six states.');
end
if ~isequal(size(D), [6, 1])
    error('D matrix must be 6x1.');
end
if ~isequal(size(K), [1, 6])
    error('K must be 1x6.');
end
if ~isequal(size(x0), [6, 1])
    error('x0 must be 6x1.');
end
if u_max <= 0
    error('u_max must be positive.');
end

%% Create a clean model
model_name = 'double_link_lqr';
model_file = fullfile(models_folder, [model_name, '.slx']);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'StopTime', 'sim_time');
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'ReturnWorkspaceOutputs', 'on');

%% Add main closed-loop blocks
add_block('simulink/Continuous/State-Space', [model_name, '/Double Link Linear Plant'], ...
    'Position', [500 165 620 255], ...
    'A', 'A', ...
    'B', 'B', ...
    'C', 'C', ...
    'D', 'D', ...
    'InitialCondition', 'x0');

add_block('simulink/Math Operations/Gain', [model_name, '/LQR Gain -K'], ...
    'Position', [140 185 240 245], ...
    'Gain', '-K', ...
    'Multiplication', 'Matrix(K*u)');

add_block('simulink/Discontinuities/Saturation', [model_name, '/Actuator Saturation'], ...
    'Position', [280 190 360 240], ...
    'UpperLimit', 'u_max', ...
    'LowerLimit', '-u_max');

add_block('simulink/Sources/From Workspace', [model_name, '/Disturbance Force'], ...
    'Position', [260 300 390 335], ...
    'VariableName', 'disturbance_force_ts');

add_block('simulink/Math Operations/Sum', [model_name, '/Control Plus Disturbance'], ...
    'Position', [420 205 455 250], ...
    'Inputs', '++');

add_block('simulink/Signal Routing/Demux', [model_name, '/Demux States'], ...
    'Position', [680 135 685 305], ...
    'Outputs', '6');

add_block('simulink/Signal Routing/Mux', [model_name, '/Scope Mux'], ...
    'Position', [850 135 855 265], ...
    'Inputs', '5');

add_block('simulink/Sinks/Scope', [model_name, '/Scope x theta1 theta2 u disturbance'], ...
    'Position', [910 155 1000 245]);

%% Add To Workspace blocks for states and control signals
state_names = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
state_y = [90 135 180 225 270 315];

for i = 1:numel(state_names)
    block_name = ['To Workspace ', state_names{i}];
    add_block('simulink/Sinks/To Workspace', [model_name, '/', block_name], ...
        'Position', [750 state_y(i) 830 state_y(i)+25], ...
        'VariableName', ['sim_', state_names{i}], ...
        'SaveFormat', 'Structure With Time');
end

add_block('simulink/Sinks/To Workspace', [model_name, '/To Workspace u_raw'], ...
    'Position', [150 275 265 305], ...
    'VariableName', 'sim_u_raw', ...
    'SaveFormat', 'Structure With Time');

add_block('simulink/Sinks/To Workspace', [model_name, '/To Workspace u_sat'], ...
    'Position', [285 260 400 290], ...
    'VariableName', 'sim_u_sat', ...
    'SaveFormat', 'Structure With Time');

add_block('simulink/Sinks/To Workspace', [model_name, '/To Workspace disturbance'], ...
    'Position', [430 300 560 330], ...
    'VariableName', 'sim_disturbance_force', ...
    'SaveFormat', 'Structure With Time');

add_block('simulink/Sinks/To Workspace', [model_name, '/To Workspace u_total'], ...
    'Position', [455 270 570 300], ...
    'VariableName', 'sim_u_total', ...
    'SaveFormat', 'Structure With Time');

%% Connect closed-loop signal flow
% Plant output X -> LQR Gain -K -> Saturation -> Sum -> Plant input.
add_line(model_name, 'Double Link Linear Plant/1', 'LQR Gain -K/1', 'autorouting', 'on');
add_line(model_name, 'LQR Gain -K/1', 'Actuator Saturation/1', 'autorouting', 'on');
add_line(model_name, 'Actuator Saturation/1', 'Control Plus Disturbance/1', 'autorouting', 'on');
add_line(model_name, 'Disturbance Force/1', 'Control Plus Disturbance/2', 'autorouting', 'on');
add_line(model_name, 'Control Plus Disturbance/1', 'Double Link Linear Plant/1', 'autorouting', 'on');

% Plant output X -> Demux states.
add_line(model_name, 'Double Link Linear Plant/1', 'Demux States/1', 'autorouting', 'on');

% Demux outputs -> To Workspace state logs.
for i = 1:numel(state_names)
    add_line(model_name, ['Demux States/', num2str(i)], ['To Workspace ', state_names{i}, '/1'], 'autorouting', 'on');
end

% Control and disturbance logs.
add_line(model_name, 'LQR Gain -K/1', 'To Workspace u_raw/1', 'autorouting', 'on');
add_line(model_name, 'Actuator Saturation/1', 'To Workspace u_sat/1', 'autorouting', 'on');
add_line(model_name, 'Disturbance Force/1', 'To Workspace disturbance/1', 'autorouting', 'on');
add_line(model_name, 'Control Plus Disturbance/1', 'To Workspace u_total/1', 'autorouting', 'on');

% Scope signals: x, theta1, theta2, u_sat, disturbance.
add_line(model_name, 'Demux States/1', 'Scope Mux/1', 'autorouting', 'on');
add_line(model_name, 'Demux States/3', 'Scope Mux/2', 'autorouting', 'on');
add_line(model_name, 'Demux States/5', 'Scope Mux/3', 'autorouting', 'on');
add_line(model_name, 'Actuator Saturation/1', 'Scope Mux/4', 'autorouting', 'on');
add_line(model_name, 'Disturbance Force/1', 'Scope Mux/5', 'autorouting', 'on');
add_line(model_name, 'Scope Mux/1', 'Scope x theta1 theta2 u disturbance/1', 'autorouting', 'on');

%% Add model notes as annotation
Simulink.Annotation(model_name, ...
    ['Phase 6 ready Simulink closed-loop model', newline, ...
     'Plant: x_dot = A*x + B*u_total', newline, ...
     'Controller: u_raw = -K*x', newline, ...
     'Actuator limit: u_sat in [-u_max, +u_max]', newline, ...
     'Disturbance: u_total = u_sat + disturbance_force', newline, ...
     'Selected C13: Q = diag([8 1 40 2 40 2]), R = 2.0, u_max = 15 N']);

%% Save model
save_system(model_name, model_file);

fprintf('\nSimulink closed-loop model created successfully.\n');
fprintf('Model file: %s\n', model_file);
fprintf('Selected Q: %s\n', mat2str(Q));
fprintf('Selected R: %.4f\n', R);
fprintf('u_max: %.4f N\n', u_max);
fprintf('Control law: u_raw = -K*x\n');
fprintf('Actuator: u_sat = saturation(u_raw, -u_max, +u_max)\n');
fprintf('Plant input: u_total = u_sat + disturbance_force\n');
