% TEST_LOAD_PARAMS
% Phase 19 smoke test for the MATLAB parameter loader.
%
% Usage from project root:
%   run('matlab/simulation/test_load_params.m')
%
% Expected output:
%   Phase 19 test_load_params: PASS

clear; clc;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);

run(fullfile(matlab_root, 'startup_project.m'));

[params, meta] = load_params(fullfile(project_root, 'shared', 'parameters.json'));

assert(isstruct(params), 'params must be a struct');
assert(isstruct(meta), 'meta must be a struct');
assert(isfield(params, 'physical_parameters'), 'Missing physical_parameters');
assert(isfield(params, 'simulation'), 'Missing simulation');
assert(isfield(params, 'control'), 'Missing control');
assert(isfield(params, 'state_definition'), 'Missing state_definition');
assert(isfield(params, 'target_modes'), 'Missing target_modes');

expected_state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
assert(iscell(meta.state_order), 'meta.state_order must be a cell array');
assert(isequal(meta.state_order(:), expected_state_order(:)), 'State order mismatch');

p = params.physical_parameters;
assert(p.cart_mass_kg > 0, 'cart_mass_kg must be positive');
assert(p.link1_mass_kg > 0, 'link1_mass_kg must be positive');
assert(p.link2_mass_kg > 0, 'link2_mass_kg must be positive');
assert(p.link1_length_m > 0, 'link1_length_m must be positive');
assert(p.link2_length_m > 0, 'link2_length_m must be positive');
assert(p.gravity_m_s2 > 0, 'gravity_m_s2 must be positive');

sim = params.simulation;
assert(sim.simulation_time_s > 0, 'simulation_time_s must be positive');
assert(sim.time_step_s > 0, 'time_step_s must be positive');

control = params.control;
assert(control.min_cart_force_N < control.max_cart_force_N, 'Force limits are invalid');

fprintf('\nPhase 19 test_load_params: PASS\n');
fprintf('Parameter file: %s\n', meta.parameter_file);
fprintf('State order: %s\n', strjoin(meta.state_order, ', '));
fprintf('Target modes: %s\n', strjoin(meta.target_names, ', '));
fprintf('Force limit: %.3f N to %.3f N\n', control.min_cart_force_N, control.max_cart_force_N);
