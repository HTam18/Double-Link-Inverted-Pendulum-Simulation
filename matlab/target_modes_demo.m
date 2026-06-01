% TARGET_MODES_DEMO
% Phase 5 MATLAB check script.
% This script reads target modes from the same shared JSON parameter file.

clear; clc;

this_file = mfilename('fullpath');
matlab_dir = fileparts(this_file);
project_root = fileparts(matlab_dir);
parameter_file = fullfile(project_root, 'shared', 'parameters.json');

if ~isfile(parameter_file)
    error('Parameter file not found: %s', parameter_file);
end

params = jsondecode(fileread(parameter_file));
target_modes = fieldnames(params.target_modes);

fprintf('Phase 5 check: target modes loaded from shared parameters.\n\n');

for i = 1:numel(target_modes)
    mode = target_modes{i};
    data = params.target_modes.(mode);
    target_state = [0; 0; data.theta1_target_rad; 0; data.theta2_target_rad; 0];

    fprintf('Mode: %s\n', mode);
    fprintf('Description: %s\n', data.description);
    fprintf('theta1 target: %.6f rad\n', data.theta1_target_rad);
    fprintf('theta2 target: %.6f rad\n', data.theta2_target_rad);
    fprintf('target state: [');
    fprintf(' %.6f', target_state);
    fprintf(' ]\n\n');
end

% Local helper example for shortest angle error: current - target.
current_angle = 2*pi + 0.1;
target_angle = 0;
err = atan2(sin(current_angle - target_angle), cos(current_angle - target_angle));
fprintf('Example angle error current=2*pi+0.1, target=0: %.6f rad\n', err);
