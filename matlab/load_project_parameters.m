% LOAD_PROJECT_PARAMETERS
% Phase 1 MATLAB check script.
% This script reads the same shared JSON parameter file used by Python.

clear; clc;

this_file = mfilename('fullpath');
matlab_dir = fileparts(this_file);
project_root = fileparts(matlab_dir);
parameter_file = fullfile(project_root, 'shared', 'parameters.json');

if ~isfile(parameter_file)
    error('Parameter file not found: %s', parameter_file);
end

raw_text = fileread(parameter_file);
params = jsondecode(raw_text);

fprintf('Double Link Pendulum on Cart Hybrid Control\n');
fprintf('Phase 1 check: shared parameters loaded successfully.\n\n');

p = params.physical_parameters;
sim = params.simulation;
control = params.control;
state_order = params.state_definition.state_order;
target_modes = fieldnames(params.target_modes);

fprintf('Cart mass: %.3f kg\n', p.cart_mass_kg);
fprintf('Link 1 mass: %.3f kg\n', p.link1_mass_kg);
fprintf('Link 2 mass: %.3f kg\n', p.link2_mass_kg);
fprintf('Link 1 length: %.3f m\n', p.link1_length_m);
fprintf('Link 2 length: %.3f m\n', p.link2_length_m);
fprintf('Gravity: %.3f m/s^2\n', p.gravity_m_s2);
fprintf('Simulation time: %.3f s\n', sim.simulation_time_s);
fprintf('Time step: %.6f s\n', sim.time_step_s);
fprintf('Cart force limit: %.3f N to %.3f N\n', control.min_cart_force_N, control.max_cart_force_N);
fprintf('State order: %s\n', strjoin(cellstr(state_order), ', '));
fprintf('Target modes: %s\n', strjoin(target_modes, ', '));
