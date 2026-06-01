% RUN_FRICTION_NOISE_TEST Phase 25 smoke test for friction and sensor noise.
%
% Purpose:
%   - Verify optional Phase 25 friction can be enabled without solver failure.
%   - Verify sensor noise is added to measurement but not to true plant state.
%   - Compare ideal mode and realism mode using the MATLAB runner.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad means downward, theta = pi rad means upright.

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
result_dir = fullfile(project_root, 'results', 'phase25_friction_noise');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

fprintf('Phase 25 MATLAB friction and sensor noise test\n');
fprintf('state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf('friction_enabled_default = %d\n', logical(params.realism.friction_enabled));
fprintf('sensor_noise_enabled_default = %d\n', logical(params.realism.sensor_noise_enabled));
fprintf('use_measurement_for_control_default = %d\n', logical(params.realism.use_measurement_for_control));

initial_state = [0.0; 0.0; pi + 0.005; 0.0; pi - 0.005; 0.0];

ideal_cfg = struct();
ideal_cfg.mode = 'phase25_ideal_lqr';
ideal_cfg.target_mode = 'up_up';
ideal_cfg.dt = double(params.simulation.time_step_s);
ideal_cfg.t_final = 4.0;
ideal_cfg.initial_state = initial_state;
ideal_cfg.controller = @lqr_stabilize_controller;
ideal_cfg.realism = struct();
ideal_cfg.realism.rail_limit_enabled = false;
ideal_cfg.realism.actuator_enabled = false;
ideal_cfg.realism.friction_enabled = false;
ideal_cfg.realism.sensor_noise_enabled = false;
ideal_cfg.realism.use_measurement_for_control = false;

realism_cfg = ideal_cfg;
realism_cfg.mode = 'phase25_realism_lqr';
realism_cfg.noise_seed = 25;
realism_cfg.realism.friction_enabled = true;
realism_cfg.realism.sensor_noise_enabled = true;
realism_cfg.realism.use_measurement_for_control = true;

ideal_result = run_simulation(ideal_cfg, params);
realism_result = run_simulation(realism_cfg, params);

plot_results(ideal_result, fullfile(result_dir, 'ideal_lqr_plot.png'));
plot_results(realism_result, fullfile(result_dir, 'realism_friction_noise_lqr_plot.png'));
plot_phase25_comparison(ideal_result, realism_result, fullfile(result_dir, 'ideal_vs_realism_compare.png'));
plot_phase25_measurement_noise(realism_result, fullfile(result_dir, 'measurement_noise.png'));

save_result_csv(ideal_result, fullfile(result_dir, 'ideal_lqr_log.csv'));
save_result_csv(realism_result, fullfile(result_dir, 'realism_friction_noise_lqr_log.csv'));
save(fullfile(result_dir, 'phase25_friction_noise_results.mat'), 'ideal_result', 'realism_result');

ideal_final_angle_error = final_max_angle_error(ideal_result);
realism_final_angle_error = final_max_angle_error(realism_result);
ideal_final_velocity_norm = norm(ideal_result.state(end, [2, 4, 6]));
realism_final_velocity_norm = norm(realism_result.state(end, [2, 4, 6]));
noise = realism_result.measurement - realism_result.state;
noise_rms = sqrt(mean(noise.^2, 1));
noise_max_abs = max(abs(noise), [], 1);
friction_sample = dip_friction([0.2; 1.0; -0.8], enable_friction_params(params));

ideal_pass = ideal_result.pass && ideal_final_angle_error < 0.02 && ideal_final_velocity_norm < 0.05;
realism_pass = realism_result.pass && realism_final_angle_error < 0.08 && realism_final_velocity_norm < 0.5;
noise_pass = all(isfinite(noise(:))) && any(noise_rms > 0) && noise_max_abs(1) < 0.02 && noise_max_abs(3) < 0.02 && noise_max_abs(5) < 0.02;
friction_pass = all(isfinite(friction_sample)) && norm(friction_sample) > 0;
compare_result = ideal_pass && realism_pass && noise_pass && friction_pass;

summary_path = fullfile(result_dir, 'phase25_friction_noise_summary.txt');
fid = fopen(summary_path, 'w');
cleanup = onCleanup(@() fclose(fid));
write_line(fid, 'Phase 25 MATLAB friction and sensor noise test');
write_line(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]');
write_line(fid, 'angle_convention = theta = 0 downward, theta = pi upright');
write_line(fid, sprintf('cart_viscous_extra_N_s_per_m = %.12g', double(params.friction.cart_viscous_extra_N_s_per_m)));
write_line(fid, sprintf('cart_coulomb_N = %.12g', double(params.friction.cart_coulomb_N)));
write_line(fid, sprintf('theta_noise_std_rad = %.12g', double(params.sensor_noise.theta1_std_rad)));
write_line(fid, sprintf('velocity_noise_std = %.12g', double(params.sensor_noise.theta1_dot_std_rad_s)));
write_line(fid, sprintf('ideal_response_pass = %d', ideal_pass));
write_line(fid, sprintf('ideal_final_max_angle_error_rad = %.12g', ideal_final_angle_error));
write_line(fid, sprintf('ideal_final_velocity_norm = %.12g', ideal_final_velocity_norm));
write_line(fid, sprintf('realism_response_pass = %d', realism_pass));
write_line(fid, sprintf('realism_final_max_angle_error_rad = %.12g', realism_final_angle_error));
write_line(fid, sprintf('realism_final_velocity_norm = %.12g', realism_final_velocity_norm));
write_line(fid, sprintf('noise_pass = %d', noise_pass));
write_line(fid, sprintf('noise_rms = [%s]', sprintf(' %.12g', noise_rms)));
write_line(fid, sprintf('noise_max_abs = [%s]', sprintf(' %.12g', noise_max_abs)));
write_line(fid, sprintf('friction_pass = %d', friction_pass));
write_line(fid, sprintf('friction_sample = [%s]', sprintf(' %.12g', friction_sample)));
write_line(fid, sprintf('compare_result = %s', pass_fail(compare_result)));
clear cleanup;

fprintf('cart_viscous_extra_N_s_per_m = %.12g\n', double(params.friction.cart_viscous_extra_N_s_per_m));
fprintf('cart_coulomb_N = %.12g\n', double(params.friction.cart_coulomb_N));
fprintf('theta_noise_std_rad = %.12g\n', double(params.sensor_noise.theta1_std_rad));
fprintf('ideal_response_pass = %d\n', ideal_pass);
fprintf('ideal_final_max_angle_error_rad = %.12g\n', ideal_final_angle_error);
fprintf('ideal_final_velocity_norm = %.12g\n', ideal_final_velocity_norm);
fprintf('realism_response_pass = %d\n', realism_pass);
fprintf('realism_final_max_angle_error_rad = %.12g\n', realism_final_angle_error);
fprintf('realism_final_velocity_norm = %.12g\n', realism_final_velocity_norm);
fprintf('noise_pass = %d\n', noise_pass);
fprintf('friction_pass = %d\n', friction_pass);
fprintf('compare_result = %s\n', pass_fail(compare_result));

if ~compare_result
    error('Phase 25 run_friction_noise_test: FAIL. See %s', summary_path);
end
fprintf('Phase 25 run_friction_noise_test: PASS\n');

function p2 = enable_friction_params(p)
    p2 = p;
    p2.realism.friction_enabled = true;
end

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
              result.state(:, 1), result.state(:, 2), ...
              result.state(:, 3), result.state(:, 4), ...
              result.state(:, 5), result.state(:, 6), ...
              result.measurement(:, 1), result.measurement(:, 2), ...
              result.measurement(:, 3), result.measurement(:, 4), ...
              result.measurement(:, 5), result.measurement(:, 6), ...
              result.u_cmd(:), result.u_actual(:), result.actuator_rate(:), string(result.mode(:)), ...
              'VariableNames', {'t','x','x_dot','theta1','theta1_dot','theta2','theta2_dot', ...
                                'x_meas','x_dot_meas','theta1_meas','theta1_dot_meas', ...
                                'theta2_meas','theta2_dot_meas','u_cmd','u_actual','actuator_rate','mode'});
    writetable(T, file_path);
end

function plot_phase25_comparison(ideal_result, realism_result, save_path)
    fig = figure('Name', 'Phase 25 Ideal vs Realism', 'Color', 'w');
    tiledlayout(4, 1);
    nexttile;
    plot(ideal_result.t, ideal_result.state(:, 1), 'LineWidth', 1.2); hold on;
    plot(realism_result.t, realism_result.state(:, 1), '--', 'LineWidth', 1.2);
    grid on; ylabel('x [m]'); legend('ideal', 'realism', 'Location', 'best');
    title('Phase 25 ideal mode vs friction/noise realism mode');

    nexttile;
    plot(ideal_result.t, ideal_result.state(:, 3), 'LineWidth', 1.2); hold on;
    plot(realism_result.t, realism_result.state(:, 3), '--', 'LineWidth', 1.2);
    plot(ideal_result.t, ideal_result.state(:, 5), 'LineWidth', 1.2);
    plot(realism_result.t, realism_result.state(:, 5), '--', 'LineWidth', 1.2);
    grid on; ylabel('angle [rad]'); legend('theta1 ideal', 'theta1 realism', 'theta2 ideal', 'theta2 realism', 'Location', 'best');

    nexttile;
    plot(ideal_result.t, ideal_result.u_cmd, 'LineWidth', 1.2); hold on;
    plot(realism_result.t, realism_result.u_cmd, '--', 'LineWidth', 1.2);
    grid on; ylabel('u cmd [N]'); legend('ideal', 'realism', 'Location', 'best');

    nexttile;
    noise = realism_result.measurement - realism_result.state;
    plot(realism_result.t, noise(:, 1), 'LineWidth', 1.0); hold on;
    plot(realism_result.t, noise(:, 3), 'LineWidth', 1.0);
    plot(realism_result.t, noise(:, 5), 'LineWidth', 1.0);
    grid on; ylabel('noise'); xlabel('time [s]'); legend('x noise', 'theta1 noise', 'theta2 noise', 'Location', 'best');

    saveas(fig, save_path);
end

function plot_phase25_measurement_noise(result, save_path)
    noise = result.measurement - result.state;
    fig = figure('Name', 'Phase 25 Measurement Noise', 'Color', 'w');
    tiledlayout(3, 1);
    nexttile;
    plot(result.t, noise(:, 1), 'LineWidth', 1.0); grid on; ylabel('x noise [m]');
    title('Measurement noise added in Phase 25 realism mode');
    nexttile;
    plot(result.t, noise(:, 3), 'LineWidth', 1.0); hold on;
    plot(result.t, noise(:, 5), 'LineWidth', 1.0); grid on; ylabel('angle noise [rad]'); legend('theta1', 'theta2', 'Location', 'best');
    nexttile;
    plot(result.t, noise(:, 2), 'LineWidth', 1.0); hold on;
    plot(result.t, noise(:, 4), 'LineWidth', 1.0);
    plot(result.t, noise(:, 6), 'LineWidth', 1.0); grid on; ylabel('velocity noise'); xlabel('time [s]'); legend('x dot', 'theta1 dot', 'theta2 dot', 'Location', 'best');
    saveas(fig, save_path);
end
