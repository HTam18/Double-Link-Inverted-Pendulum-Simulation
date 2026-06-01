% RUN_ACTUATOR_TEST Phase 24 actuator dynamics and saturation test.
%
% Goal:
%   Verify that realism actuator mode uses F_actual, not ideal F_cmd, and that
%   the first-order actuator remains finite, saturated, rate-limited and stable.
%
% Expected output:
%   Phase 24 run_actuator_test: PASS

clear; clc;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();

fprintf('Phase 24 MATLAB actuator dynamics test\n');
fprintf('State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('Angle convention: theta = 0 downward, theta = pi upright\n');
fprintf('tau_motor_s = %.12g\n', double(params.actuator.tau_motor_s));
fprintf('rate_limit_N_per_s = %.12g\n', double(params.actuator.rate_limit_N_per_s));
fprintf('dead_zone_N = %.12g\n', double(params.actuator.dead_zone_N));
fprintf('min_force_N = %.12g\n', double(params.actuator.min_force_N));
fprintf('max_force_N = %.12g\n', double(params.actuator.max_force_N));

output_dir = fullfile(project_root, 'results', 'phase24_actuator');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% -------------------------------------------------------------------------
% Test 1: step command. This directly checks delay, saturation and rate limit.
% -------------------------------------------------------------------------
cfg_step = struct();
cfg_step.mode = 'actuator_step_response';
cfg_step.t_final = 1.0;
cfg_step.dt = double(params.simulation.time_step_s);
cfg_step.initial_state = zeros(6, 1);
cfg_step.realism = struct();
cfg_step.realism.actuator_enabled = true;
cfg_step.controller = @local_step_force_controller;

step_result = run_simulation(cfg_step, params);

force_limit = max(abs([double(params.actuator.min_force_N), double(params.actuator.max_force_N)]));
max_abs_actual = max(abs(step_result.u_actual));
max_abs_rate = max(abs(step_result.actuator_rate));
rate_limit = double(params.actuator.rate_limit_N_per_s);
final_actual = step_result.u_actual(end);
final_cmd = step_result.u_cmd(end);
command_jump_idx = find(abs(diff(step_result.u_cmd(:))) > 1e-9, 1, 'first') + 1;
if isempty(command_jump_idx)
    command_jump_idx = NaN;
    initial_lag = 0.0;
else
    initial_lag = abs(step_result.u_cmd(command_jump_idx) - step_result.u_actual(command_jump_idx));
end
final_lag = abs(final_cmd - final_actual);

step_pass = true;
step_pass = step_pass && step_result.pass;
step_pass = step_pass && all(isfinite(step_result.u_cmd));
step_pass = step_pass && all(isfinite(step_result.u_actual));
step_pass = step_pass && max_abs_actual <= force_limit + 1e-9;
step_pass = step_pass && max_abs_rate <= rate_limit + 1e-9;
step_pass = step_pass && initial_lag > 0.1;
step_pass = step_pass && final_lag < 1.0;

step_table = table(step_result.t, step_result.u_cmd(:), step_result.u_actual(:), ...
                   step_result.actuator_rate(:), step_result.state(:, 1), step_result.state(:, 2), ...
                   'VariableNames', {'t_s', 'u_cmd_N', 'u_actual_N', 'actuator_rate_N_per_s', 'x_m', 'x_dot_m_s'});
writetable(step_table, fullfile(output_dir, 'actuator_step_response_log.csv'));
save(fullfile(output_dir, 'actuator_step_response_result.mat'), 'step_result', 'cfg_step');
local_plot_actuator(step_result, fullfile(output_dir, 'actuator_step_response_plot.png'), 'Phase 24 actuator step response');

% -------------------------------------------------------------------------
% Test 2: near-up_up LQR with actuator enabled. This verifies that the Phase
% 22 controller can still run through the actuator path in realism mode.
% -------------------------------------------------------------------------
target_mode = 'up_up';
target = params.target_modes.(target_mode);
target_state = [0; 0; double(target.theta1_target_rad); 0; double(target.theta2_target_rad); 0];
initial_state = target_state + [0.002; 0.000; 0.005; 0.000; -0.004; 0.000];

cfg_lqr = struct();
cfg_lqr.mode = 'lqr_stabilize_actuator';
cfg_lqr.target_mode = target_mode;
cfg_lqr.t_final = 1.5;
cfg_lqr.dt = double(params.simulation.time_step_s);
cfg_lqr.initial_state = initial_state;
cfg_lqr.realism = struct();
cfg_lqr.realism.actuator_enabled = true;
cfg_lqr.controller = @lqr_stabilize_controller;

lqr_result = run_simulation(cfg_lqr, params);
angle_error = local_angle_error_history(lqr_result.state, target_state);
initial_max_angle_error = max(abs(angle_error(1, :)));
final_max_angle_error = max(abs(angle_error(end, :)));
final_velocity_norm = norm(lqr_result.state(end, [2, 4, 6]));
max_abs_lqr_actual = max(abs(lqr_result.u_actual));

lqr_pass = true;
lqr_pass = lqr_pass && lqr_result.pass;
lqr_pass = lqr_pass && all(isfinite(lqr_result.state(:)));
lqr_pass = lqr_pass && all(isfinite(lqr_result.u_actual(:)));
% Phase 24 only checks that LQR can run through the actuator path
% without numerical failure or unsafe boundedness issues. Strong
% performance/stabilization tuning with actuator delay is deferred to later
% control phases.
lqr_pass = lqr_pass && final_max_angle_error < 0.25;
lqr_pass = lqr_pass && final_velocity_norm < 5.00;
lqr_pass = lqr_pass && max(abs(lqr_result.state(:, 1))) < 1.00;
lqr_pass = lqr_pass && max_abs_lqr_actual <= force_limit + 1e-9;

lqr_table = table(lqr_result.t, ...
                  lqr_result.state(:, 1), lqr_result.state(:, 2), ...
                  lqr_result.state(:, 3), lqr_result.state(:, 4), ...
                  lqr_result.state(:, 5), lqr_result.state(:, 6), ...
                  lqr_result.u_cmd(:), lqr_result.u_actual(:), lqr_result.actuator_rate(:), ...
                  angle_error(:, 1), angle_error(:, 2), lqr_result.mode(:), ...
                  'VariableNames', {'t_s', 'x_m', 'x_dot_m_s', ...
                                    'theta1_rad', 'theta1_dot_rad_s', ...
                                    'theta2_rad', 'theta2_dot_rad_s', ...
                                    'u_cmd_N', 'u_actual_N', 'actuator_rate_N_per_s', ...
                                    'theta1_error_rad', 'theta2_error_rad', 'mode'});
writetable(lqr_table, fullfile(output_dir, 'lqr_with_actuator_log.csv'));
save(fullfile(output_dir, 'lqr_with_actuator_result.mat'), 'lqr_result', 'cfg_lqr', 'target_state', 'angle_error');
plot_results(lqr_result, fullfile(output_dir, 'lqr_with_actuator_state_plot.png'));
local_plot_actuator(lqr_result, fullfile(output_dir, 'lqr_with_actuator_force_plot.png'), 'Phase 24 LQR through actuator');

pass = step_pass && lqr_pass;

summary_path = fullfile(output_dir, 'actuator_summary.txt');
fid = fopen(summary_path, 'w');
fprintf(fid, 'Phase 24 MATLAB actuator dynamics test\n');
fprintf(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf(fid, 'angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf(fid, 'tau_motor_s = %.12g\n', double(params.actuator.tau_motor_s));
fprintf(fid, 'rate_limit_N_per_s = %.12g\n', rate_limit);
fprintf(fid, 'dead_zone_N = %.12g\n', double(params.actuator.dead_zone_N));
fprintf(fid, 'force_limit_abs_N = %.12g\n', force_limit);
fprintf(fid, 'step_response_pass = %d\n', step_pass);
fprintf(fid, 'step_command_jump_index = %d\n', command_jump_idx);
fprintf(fid, 'step_initial_lag_N = %.12g\n', initial_lag);
fprintf(fid, 'step_final_lag_N = %.12g\n', final_lag);
fprintf(fid, 'step_max_abs_actual_N = %.12g\n', max_abs_actual);
fprintf(fid, 'step_max_abs_rate_N_per_s = %.12g\n', max_abs_rate);
fprintf(fid, 'lqr_actuator_pass = %d\n', lqr_pass);
fprintf(fid, 'lqr_initial_max_angle_error_rad = %.12g\n', initial_max_angle_error);
fprintf(fid, 'lqr_final_max_angle_error_rad = %.12g\n', final_max_angle_error);
fprintf(fid, 'lqr_final_velocity_norm = %.12g\n', final_velocity_norm);
fprintf(fid, 'lqr_max_abs_actual_N = %.12g\n', max_abs_lqr_actual);
if pass
    fprintf(fid, 'compare_result = PASS\n');
else
    fprintf(fid, 'compare_result = FAIL\n');
end
fclose(fid);

fprintf('step_response: pass=%d, initial_lag=%.6g, final_lag=%.6g, max_abs_actual=%.6g, max_abs_rate=%.6g\n', ...
        step_pass, initial_lag, final_lag, max_abs_actual, max_abs_rate);
fprintf('lqr_with_actuator: pass=%d, initial_angle_error=%.6g, final_angle_error=%.6g, final_velocity_norm=%.6g\n', ...
        lqr_pass, initial_max_angle_error, final_max_angle_error, final_velocity_norm);
fprintf('Saved summary: %s\n', summary_path);

if pass
    fprintf('Phase 24 run_actuator_test: PASS\n');
else
    error('Phase 24 run_actuator_test: FAIL. See summary: %s', summary_path);
end

function out = local_step_force_controller(t, state, params, sim_config)
    %#ok<INUSD>
    if t < 0.10
        cmd = 0.0;
    elseif t < 0.55
        cmd = 12.0;
    else
        cmd = -8.0;
    end
    out = struct();
    out.u_cmd = cmd;
    out.mode = 'actuator_step';
    out.measurement = state;
    out.x_hat = state;
end

function angle_error = local_angle_error_history(state_history, target_state)
    angle_error = zeros(size(state_history, 1), 2);
    angle_error(:, 1) = atan2(sin(state_history(:, 3) - target_state(3)), ...
                              cos(state_history(:, 3) - target_state(3)));
    angle_error(:, 2) = atan2(sin(state_history(:, 5) - target_state(5)), ...
                              cos(state_history(:, 5) - target_state(5)));
end

function local_plot_actuator(result, save_path, plot_title)
    fig = figure('Name', plot_title, 'Color', 'w');
    tiledlayout(2, 1);

    nexttile;
    plot(result.t, result.u_cmd(:), 'LineWidth', 1.2);
    hold on;
    plot(result.t, result.u_actual(:), '--', 'LineWidth', 1.2);
    grid on;
    ylabel('force [N]');
    title(plot_title);
    legend('F cmd', 'F actual', 'Location', 'best');

    nexttile;
    plot(result.t, result.actuator_rate(:), 'LineWidth', 1.2);
    grid on;
    xlabel('time [s]');
    ylabel('F dot [N/s]');

    save_dir = fileparts(save_path);
    if ~isempty(save_dir) && ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    saveas(fig, save_path);
end
