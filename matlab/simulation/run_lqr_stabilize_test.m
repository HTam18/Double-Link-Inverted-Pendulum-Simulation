% RUN_LQR_STABILIZE_TEST Phase 22 local LQR stabilize test.
%
% This test uses the Phase 21 MATLAB runner and the Phase 22 MATLAB LQR
% controller. It does not call Python. The initial condition is deliberately
% near the up_up equilibrium because local LQR is not a swing-up controller.
%
% Expected output:
%   Phase 22 run_lqr_stabilize_test: PASS

clear; clc;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();

target_mode = 'up_up';
target = params.target_modes.(target_mode);
target_state = [0; 0; double(target.theta1_target_rad); 0; double(target.theta2_target_rad); 0];

% Near-upright initial condition. Keep this inside the local LQR capture
% region; swing-up is intentionally not part of Phase 22.
initial_state = target_state + [0.030; 0.000; 0.080; 0.000; -0.060; 0.000];

cfg = struct();
cfg.mode = 'lqr_stabilize';
cfg.target_mode = target_mode;
cfg.t_final = 5.0;
cfg.dt = double(params.simulation.time_step_s);
cfg.initial_state = initial_state;
cfg.controller = @lqr_stabilize_controller;

fprintf('Phase 22 MATLAB LQR stabilize test\n');
fprintf('Target mode: %s\n', target_mode);
fprintf('State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('Angle convention: theta = 0 downward, theta = pi upright\n');

result = run_simulation(cfg, params);

angle_error = local_angle_error_history(result.state, target_state);
initial_max_angle_error = max(abs(angle_error(1, :)));
final_max_angle_error = max(abs(angle_error(end, :)));
max_abs_x = max(abs(result.state(:, 1)));
max_abs_u = max(abs(result.u_actual));
max_force_limit = double(params.control.max_cart_force_N);
min_force_limit = double(params.control.min_cart_force_N);
force_limit = max(abs([min_force_limit, max_force_limit]));

final_state = result.state(end, :).';
final_velocity_norm = norm(final_state([2, 4, 6]));

pass = true;
pass = pass && result.pass;
pass = pass && all(isfinite(result.state(:)));
pass = pass && all(isfinite(result.u_actual(:)));
pass = pass && final_max_angle_error < initial_max_angle_error;
pass = pass && final_max_angle_error < 0.08;
pass = pass && final_velocity_norm < 0.70;
pass = pass && max_abs_u <= force_limit + 1e-9;

output_dir = fullfile(project_root, 'results', 'phase22_lqr_stabilize');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

result_table = table(result.t, ...
                     result.state(:, 1), result.state(:, 2), ...
                     result.state(:, 3), result.state(:, 4), ...
                     result.state(:, 5), result.state(:, 6), ...
                     result.u_cmd(:), result.u_actual(:), ...
                     angle_error(:, 1), angle_error(:, 2), ...
                     result.mode(:), ...
                     'VariableNames', {'t_s', 'x_m', 'x_dot_m_s', ...
                                       'theta1_rad', 'theta1_dot_rad_s', ...
                                       'theta2_rad', 'theta2_dot_rad_s', ...
                                       'u_cmd_N', 'u_actual_N', ...
                                       'theta1_error_rad', 'theta2_error_rad', 'mode'});
writetable(result_table, fullfile(output_dir, 'lqr_stabilize_up_up_log.csv'));
save(fullfile(output_dir, 'lqr_stabilize_up_up_result.mat'), 'result', 'cfg', 'target_state', 'angle_error');

plot_results(result, fullfile(output_dir, 'lqr_stabilize_up_up_plot.png'));
local_plot_angle_error(result.t, angle_error, fullfile(output_dir, 'lqr_stabilize_up_up_angle_error.png'));

summary_path = fullfile(output_dir, 'lqr_stabilize_up_up_summary.txt');
fid = fopen(summary_path, 'w');
fprintf(fid, 'Phase 22 MATLAB LQR stabilize test\n');
fprintf(fid, 'target_mode = %s\n', target_mode);
fprintf(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf(fid, 'angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf(fid, 'initial_max_angle_error_rad = %.12g\n', initial_max_angle_error);
fprintf(fid, 'final_max_angle_error_rad = %.12g\n', final_max_angle_error);
fprintf(fid, 'final_velocity_norm = %.12g\n', final_velocity_norm);
fprintf(fid, 'max_abs_x_m = %.12g\n', max_abs_x);
fprintf(fid, 'max_abs_u_N = %.12g\n', max_abs_u);
fprintf(fid, 'force_limit_abs_N = %.12g\n', force_limit);
fprintf(fid, 'final_state = [%s]\n', sprintf(' %.12g', final_state));
if pass
    fprintf(fid, 'compare_result = PASS\n');
else
    fprintf(fid, 'compare_result = FAIL\n');
end
fclose(fid);

fprintf('initial_max_angle_error_rad = %.6g\n', initial_max_angle_error);
fprintf('final_max_angle_error_rad   = %.6g\n', final_max_angle_error);
fprintf('final_velocity_norm          = %.6g\n', final_velocity_norm);
fprintf('max_abs_x_m                  = %.6g\n', max_abs_x);
fprintf('max_abs_u_N                  = %.6g\n', max_abs_u);
fprintf('Saved log:     %s\n', fullfile(output_dir, 'lqr_stabilize_up_up_log.csv'));
fprintf('Saved summary: %s\n', summary_path);

if pass
    fprintf('Phase 22 run_lqr_stabilize_test: PASS\n');
else
    error('Phase 22 run_lqr_stabilize_test: FAIL. See summary: %s', summary_path);
end

function angle_error = local_angle_error_history(state_history, target_state)
    angle_error = zeros(size(state_history, 1), 2);
    angle_error(:, 1) = atan2(sin(state_history(:, 3) - target_state(3)), ...
                              cos(state_history(:, 3) - target_state(3)));
    angle_error(:, 2) = atan2(sin(state_history(:, 5) - target_state(5)), ...
                              cos(state_history(:, 5) - target_state(5)));
end

function local_plot_angle_error(t, angle_error, save_path)
    fig = figure('Name', 'Phase 22 LQR angle error', 'Color', 'w');
    plot(t, angle_error(:, 1), 'LineWidth', 1.2);
    hold on;
    plot(t, angle_error(:, 2), 'LineWidth', 1.2);
    grid on;
    xlabel('time [s]');
    ylabel('wrapped angle error [rad]');
    title('Phase 22 LQR stabilize angle error');
    legend('theta1 error', 'theta2 error', 'Location', 'best');
    save_dir = fileparts(save_path);
    if ~isempty(save_dir) && ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    saveas(fig, save_path);
end
