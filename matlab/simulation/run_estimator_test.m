% RUN_ESTIMATOR_TEST Phase 26 smoke test for simple state estimation.
%
% Purpose:
%   - Verify velocity_filter.m and state_estimator_simple.m run without NaN.
%   - Verify run_simulation logs measurement and x_hat.
%   - Verify controller realism mode can use x_hat instead of true clean state.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.

clear; clc;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
result_dir = fullfile(project_root, 'results', 'phase26_estimator');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

fprintf('Phase 26 MATLAB simple estimator test\n');
fprintf('state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf('velocity_filter_alpha = %.12g\n', double(params.estimator.velocity_filter_alpha));
fprintf('estimator_enabled_default = %d\n', logical(params.realism.estimator_enabled));
fprintf('use_estimator_for_control_default = %d\n', logical(params.realism.use_estimator_for_control));

% Keep the initial condition inside the local LQR capture region. Phase 26
% is about estimator plumbing and state estimation, not swing-up.
initial_state = [0.0; 0.0; pi + 0.004; 0.0; pi - 0.004; 0.0];

cfg = struct();
cfg.mode = 'phase26_estimator_lqr';
cfg.target_mode = 'up_up';
cfg.dt = double(params.simulation.time_step_s);
cfg.t_final = 4.0;
cfg.initial_state = initial_state;
cfg.controller = @lqr_stabilize_controller;
cfg.noise_seed = 26;
cfg.realism = struct();
cfg.realism.rail_limit_enabled = false;
cfg.realism.actuator_enabled = false;
cfg.realism.friction_enabled = true;
cfg.realism.sensor_noise_enabled = true;
cfg.realism.use_measurement_for_control = false;
cfg.realism.estimator_enabled = true;
cfg.realism.use_estimator_for_control = true;

result = run_simulation(cfg, params);

plot_results(result, fullfile(result_dir, 'estimator_lqr_plot.png'));
plot_estimator_comparison(result, fullfile(result_dir, 'true_measurement_estimate_compare.png'));
plot_estimator_errors(result, fullfile(result_dir, 'estimator_error.png'));
save_result_csv(result, fullfile(result_dir, 'estimator_lqr_log.csv'));
save(fullfile(result_dir, 'phase26_estimator_result.mat'), 'result', 'cfg');

warmup_count = min(50, max(1, floor(0.25 / cfg.dt)));
idx = (warmup_count + 1):numel(result.t);
if isempty(idx)
    idx = 1:numel(result.t);
end

position_cols = [1, 3, 5];
velocity_cols = [2, 4, 6];
position_error = result.x_hat(idx, position_cols) - result.state(idx, position_cols);
velocity_error = result.x_hat(idx, velocity_cols) - result.state(idx, velocity_cols);
measurement_velocity_error = result.measurement(idx, velocity_cols) - result.state(idx, velocity_cols);
measurement_position_error = result.measurement(idx, position_cols) - result.state(idx, position_cols);

position_rms = sqrt(mean(position_error.^2, 1));
velocity_rms = sqrt(mean(velocity_error.^2, 1));
measurement_velocity_rms = sqrt(mean(measurement_velocity_error.^2, 1));
measurement_position_rms = sqrt(mean(measurement_position_error.^2, 1));

final_angle_error = final_max_angle_error(result);
final_velocity_norm = norm(result.state(end, velocity_cols));
x_hat_diff_from_true_rms = sqrt(mean((result.x_hat(idx, :) - result.state(idx, :)).^2, 1));
x_hat_diff_from_measurement_rms = sqrt(mean((result.x_hat(idx, :) - result.measurement(idx, :)).^2, 1));

max_position_error_for_pass = double(params.estimator.max_position_error_for_pass);
max_velocity_error_rms_for_pass = double(params.estimator.max_velocity_error_rms_for_pass);

estimator_output_pass = result.pass && all(isfinite(result.x_hat(:))) && all(isfinite(result.measurement(:)));
position_pass = max(position_rms) < max_position_error_for_pass;
velocity_pass = max(velocity_rms) < max_velocity_error_rms_for_pass;
controller_with_xhat_pass = final_angle_error < 0.02 && final_velocity_norm < 0.15;
measurement_noise_visible_pass = any(abs(result.measurement(:) - result.state(:)) > 0);
xhat_not_raw_measurement_pass = any(abs(result.x_hat(:, velocity_cols) - result.measurement(:, velocity_cols)) > 0, 'all');
compare_result = estimator_output_pass && position_pass && velocity_pass && controller_with_xhat_pass && measurement_noise_visible_pass && xhat_not_raw_measurement_pass;

summary_path = fullfile(result_dir, 'phase26_estimator_summary.txt');
fid = fopen(summary_path, 'w');
cleanup = onCleanup(@() fclose(fid));
write_line(fid, 'Phase 26 MATLAB simple estimator test');
write_line(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]');
write_line(fid, 'angle_convention = theta = 0 downward, theta = pi upright');
write_line(fid, sprintf('velocity_filter_alpha = %.12g', double(params.estimator.velocity_filter_alpha)));
write_line(fid, sprintf('sensor_noise_enabled = %d', logical(cfg.realism.sensor_noise_enabled)));
write_line(fid, sprintf('friction_enabled = %d', logical(cfg.realism.friction_enabled)));
write_line(fid, sprintf('estimator_enabled = %d', logical(cfg.realism.estimator_enabled)));
write_line(fid, sprintf('use_estimator_for_control = %d', logical(cfg.realism.use_estimator_for_control)));
write_line(fid, sprintf('estimator_output_pass = %d', estimator_output_pass));
write_line(fid, sprintf('position_pass = %d', position_pass));
write_line(fid, sprintf('velocity_pass = %d', velocity_pass));
write_line(fid, sprintf('controller_with_xhat_pass = %d', controller_with_xhat_pass));
write_line(fid, sprintf('measurement_noise_visible_pass = %d', measurement_noise_visible_pass));
write_line(fid, sprintf('xhat_not_raw_measurement_pass = %d', xhat_not_raw_measurement_pass));
write_line(fid, sprintf('position_rms = [%s]', sprintf(' %.12g', position_rms)));
write_line(fid, sprintf('velocity_rms = [%s]', sprintf(' %.12g', velocity_rms)));
write_line(fid, sprintf('measurement_position_rms = [%s]', sprintf(' %.12g', measurement_position_rms)));
write_line(fid, sprintf('measurement_velocity_rms = [%s]', sprintf(' %.12g', measurement_velocity_rms)));
write_line(fid, sprintf('x_hat_diff_from_true_rms = [%s]', sprintf(' %.12g', x_hat_diff_from_true_rms)));
write_line(fid, sprintf('x_hat_diff_from_measurement_rms = [%s]', sprintf(' %.12g', x_hat_diff_from_measurement_rms)));
write_line(fid, sprintf('final_max_angle_error_rad = %.12g', final_angle_error));
write_line(fid, sprintf('final_velocity_norm = %.12g', final_velocity_norm));
write_line(fid, sprintf('compare_result = %s', pass_fail(compare_result)));
clear cleanup;

fprintf('estimator_output_pass = %d\n', estimator_output_pass);
fprintf('position_pass = %d\n', position_pass);
fprintf('velocity_pass = %d\n', velocity_pass);
fprintf('controller_with_xhat_pass = %d\n', controller_with_xhat_pass);
fprintf('measurement_noise_visible_pass = %d\n', measurement_noise_visible_pass);
fprintf('xhat_not_raw_measurement_pass = %d\n', xhat_not_raw_measurement_pass);
fprintf('position_rms = [%s]\n', sprintf(' %.12g', position_rms));
fprintf('velocity_rms = [%s]\n', sprintf(' %.12g', velocity_rms));
fprintf('measurement_velocity_rms = [%s]\n', sprintf(' %.12g', measurement_velocity_rms));
fprintf('final_max_angle_error_rad = %.12g\n', final_angle_error);
fprintf('final_velocity_norm = %.12g\n', final_velocity_norm);
fprintf('compare_result = %s\n', pass_fail(compare_result));

if ~compare_result
    error('Phase 26 run_estimator_test: FAIL. See %s', summary_path);
end
fprintf('Phase 26 run_estimator_test: PASS\n');

function e = final_max_angle_error(result)
    theta1_err = wrap_to_pi_local(result.state(end, 3) - pi);
    theta2_err = wrap_to_pi_local(result.state(end, 5) - pi);
    e = max(abs([theta1_err, theta2_err]));
end

function y = wrap_to_pi_local(angle_rad)
    y = atan2(sin(angle_rad), cos(angle_rad));
end

function text = pass_fail(value)
    if value
        text = 'PASS';
    else
        text = 'FAIL';
    end
end

function write_line(fid, text)
    fprintf(fid, '%s\n', text);
end

function save_result_csv(result, file_path)
    T = table(result.t(:), ...
              result.state(:, 1), result.state(:, 2), result.state(:, 3), result.state(:, 4), result.state(:, 5), result.state(:, 6), ...
              result.measurement(:, 1), result.measurement(:, 2), result.measurement(:, 3), result.measurement(:, 4), result.measurement(:, 5), result.measurement(:, 6), ...
              result.x_hat(:, 1), result.x_hat(:, 2), result.x_hat(:, 3), result.x_hat(:, 4), result.x_hat(:, 5), result.x_hat(:, 6), ...
              result.u_cmd(:), result.u_actual(:), string(result.mode(:)), ...
              'VariableNames', {'t','x','x_dot','theta1','theta1_dot','theta2','theta2_dot', ...
                                'x_meas','x_dot_meas','theta1_meas','theta1_dot_meas','theta2_meas','theta2_dot_meas', ...
                                'x_hat','x_dot_hat','theta1_hat','theta1_dot_hat','theta2_hat','theta2_dot_hat', ...
                                'u_cmd','u_actual','mode'});
    writetable(T, file_path);
end

function plot_estimator_comparison(result, save_path)
    fig = figure('Name', 'Phase 26 True Measurement Estimate', 'Color', 'w');
    tiledlayout(3, 1);

    nexttile;
    plot(result.t, result.state(:, 1), 'LineWidth', 1.2); hold on;
    plot(result.t, result.measurement(:, 1), ':', 'LineWidth', 1.0);
    plot(result.t, result.x_hat(:, 1), '--', 'LineWidth', 1.2);
    grid on; ylabel('x [m]'); legend('true', 'measurement', 'x hat', 'Location', 'best');
    title('Phase 26 true state, noisy measurement and simple estimate');

    nexttile;
    plot(result.t, result.state(:, 3), 'LineWidth', 1.2); hold on;
    plot(result.t, result.measurement(:, 3), ':', 'LineWidth', 1.0);
    plot(result.t, result.x_hat(:, 3), '--', 'LineWidth', 1.2);
    plot(result.t, result.state(:, 5), 'LineWidth', 1.2);
    plot(result.t, result.measurement(:, 5), ':', 'LineWidth', 1.0);
    plot(result.t, result.x_hat(:, 5), '--', 'LineWidth', 1.2);
    grid on; ylabel('angle [rad]'); legend('theta1 true','theta1 meas','theta1 hat','theta2 true','theta2 meas','theta2 hat','Location','best');

    nexttile;
    plot(result.t, result.state(:, 2), 'LineWidth', 1.2); hold on;
    plot(result.t, result.x_hat(:, 2), '--', 'LineWidth', 1.2);
    plot(result.t, result.state(:, 4), 'LineWidth', 1.2);
    plot(result.t, result.x_hat(:, 4), '--', 'LineWidth', 1.2);
    plot(result.t, result.state(:, 6), 'LineWidth', 1.2);
    plot(result.t, result.x_hat(:, 6), '--', 'LineWidth', 1.2);
    grid on; ylabel('velocity'); xlabel('time [s]'); legend('x dot true','x dot hat','theta1 dot true','theta1 dot hat','theta2 dot true','theta2 dot hat','Location','best');

    saveas(fig, save_path);
end

function plot_estimator_errors(result, save_path)
    err = result.x_hat - result.state;
    fig = figure('Name', 'Phase 26 Estimator Error', 'Color', 'w');
    tiledlayout(3, 1);
    nexttile;
    plot(result.t, err(:, 1), 'LineWidth', 1.0); hold on;
    plot(result.t, err(:, 3), 'LineWidth', 1.0);
    plot(result.t, err(:, 5), 'LineWidth', 1.0);
    grid on; ylabel('position error'); title('Phase 26 x hat minus true state'); legend('x','theta1','theta2','Location','best');
    nexttile;
    plot(result.t, err(:, 2), 'LineWidth', 1.0); hold on;
    plot(result.t, err(:, 4), 'LineWidth', 1.0);
    plot(result.t, err(:, 6), 'LineWidth', 1.0);
    grid on; ylabel('velocity error'); legend('x dot','theta1 dot','theta2 dot','Location','best');
    nexttile;
    plot(result.t, result.u_cmd, 'LineWidth', 1.2); hold on;
    plot(result.t, result.u_actual, '--', 'LineWidth', 1.2);
    grid on; ylabel('force [N]'); xlabel('time [s]'); legend('u cmd','u actual','Location','best');
    saveas(fig, save_path);
end
