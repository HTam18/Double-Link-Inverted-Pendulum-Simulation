% STARTUP_PROJECT
% Phase 19 MATLAB startup script for the Double Link Pendulum project.
%
% Usage from project root:
%   run('matlab/startup_project.m')
%
% This script only configures MATLAB paths. It does not change plant,
% controller, simulation, or evaluation logic.

this_file = mfilename('fullpath');
matlab_root = fileparts(this_file);
project_root = fileparts(matlab_root);

required_dirs = {
    matlab_root
    fullfile(matlab_root, 'params')
    fullfile(matlab_root, 'dynamics')
    fullfile(matlab_root, 'control')
    fullfile(matlab_root, 'simulation')
    fullfile(matlab_root, 'evaluation')
    fullfile(matlab_root, 'visualization')
    fullfile(matlab_root, 'estimation')
    fullfile(matlab_root, 'simulink')
};

for k = 1:numel(required_dirs)
    if ~isfolder(required_dirs{k})
        mkdir(required_dirs{k});
    end
    addpath(required_dirs{k});
end

fprintf('Double Link Pendulum MATLAB path configured.\n');
fprintf('Project root: %s\n', project_root);
fprintf('MATLAB root:  %s\n', matlab_root);
