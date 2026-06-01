% INIT_SIMULINK_MODEL Build the Phase 32 main Simulink model.
%
% The generated model is:
%   matlab/simulink/Double_Link_Pendulum_Main.slx
%
% The model uses an interpreted MATLAB Function block to call the validated
% MATLAB Phase 29/30 functions. This keeps the plant, actuator, TVLQR tracking,
% hybrid handoff and terminal hold logic in one source of truth while providing
% a clear Simulink model for signal logging and later block-by-block expansion.

this_file = mfilename('fullpath');
simulink_dir = fileparts(this_file);
matlab_root = fileparts(simulink_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

cfg = phase32_load_config(project_root);
model_name = 'Double_Link_Pendulum_Main';
model_file = fullfile(simulink_dir, [model_name '.slx']);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist(model_file, 'file')
    delete(model_file);
end

new_system(model_name);
open_system(model_name);
set_param(model_name, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(cfg.dt, '%.16g'), ...
    'StopTime', num2str(cfg.t_final, '%.16g'), ...
    'SaveOutput', 'on', ...
    'SignalLogging', 'on', ...
    'ReturnWorkspaceOutputs', 'on');

% Main clock.
add_block('simulink/Sources/Clock', [model_name '/Simulation Clock'], ...
    'Position', [80 100 120 130]);

% MATLAB Function block that calls the existing Phase 29/30 MATLAB pipeline.
block_path = [model_name '/Phase29_30_Hybrid_Controller_Plant_Actuator'];
add_block('simulink/User-Defined Functions/MATLAB Function', block_path, ...
    'Position', [230 65 500 165]);

% Set MATLAB Function block code through Stateflow API.
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
chart.Script = sprintf([ ...
    'function y = fcn(t)\n', ...
    '%%#codegen\n', ...
    'coder.extrinsic(''phase32_hybrid_step'');\n', ...
    'y = zeros(34,1);\n', ...
    'tmp = phase32_hybrid_step(t,false);\n', ...
    'y = tmp;\n', ...
    'end\n']);

% Workspace logger. Format is Structure With Time for easy parsing.
add_block('simulink/Sinks/To Workspace', [model_name '/Log_main_signals_phase32'], ...
    'Position', [620 92 820 138], ...
    'VariableName', 'phase32_yout', ...
    'SaveFormat', 'Structure With Time');

add_line(model_name, 'Simulation Clock/1', 'Phase29_30_Hybrid_Controller_Plant_Actuator/1');
add_line(model_name, 'Phase29_30_Hybrid_Controller_Plant_Actuator/1', 'Log_main_signals_phase32/1');

% Documentation annotations in the model.
Simulink.Annotation(model_name, ...
    sprintf(['Phase 32 main Simulink model\n', ...
             'Pipeline: Phase 29 discrete TVLQR tracking -> Phase 30 hybrid handoff -> terminal hold/stabilize.\n', ...
             'State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot].\n', ...
             'The MATLAB Function block calls phase32_hybrid_step.m, which calls existing MATLAB plant/controller functions.']));

set_param([model_name '/Simulation Clock'], 'Name', 'Clock_t');
set_param(block_path, 'Name', 'Hybrid_Controller_Plant_Actuator_Pipeline');

save_system(model_name, model_file);
fprintf('Phase 32 Simulink model generated: %s\n', model_file);
fprintf('Stop time: %.12g s, fixed step: %.12g s\n', cfg.t_final, cfg.dt);
