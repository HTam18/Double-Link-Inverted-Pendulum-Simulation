% RUN_RAIL_LIMIT_TEST Phase 23 soft rail limit test.
%
% How to run from project root:
%   run('matlab/startup_project.m')
%   run('matlab/simulation/run_rail_limit_test.m')
%
% Expected output:
%   Phase 23 run_rail_limit_test: PASS

clear; clc;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
rail = params.rail_limit;
x_min = double(rail.x_min_m);
x_max = double(rail.x_max_m);
x_soft = double(rail.x_soft_m);

fprintf('Phase 23 MATLAB rail limit test\n');
fprintf('State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf('Angle convention: theta = 0 downward, theta = pi upright\n');
fprintf('Rail limits: x_min = %.3f m, x_max = %.3f m, x_soft = %.3f m\n', x_min, x_max, x_soft);

output_dir = fullfile(project_root, 'results', 'phase23_rail_limit');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

cases = local_build_cases(params);
summary_lines = strings(numel(cases), 1);
overall_pass = true;

for i = 1:numel(cases)
    c = cases(i);

    cfg = struct();
    cfg.mode = c.name;
    cfg.t_final = c.t_final;
    cfg.dt = double(params.simulation.time_step_s);
    cfg.initial_state = c.initial_state;
    cfg.controller = c.controller;
    cfg.realism = struct();
    cfg.realism.rail_limit_enabled = true;

    result = run_simulation(cfg, params);
    rail_force = local_compute_rail_force_history(result.state, result.params);

    max_x = max(result.state(:, 1));
    min_x = min(result.state(:, 1));
    max_abs_rail_force = max(abs(rail_force));
    finite_ok = result.pass && all(isfinite(result.state(:))) && all(isfinite(result.u_actual(:))) && all(isfinite(rail_force(:)));

    % Normal near-boundary cases should stay inside physical limits.
    % Stress case may touch the soft wall, but should stay bounded and finite.
    if c.strict_inside
        case_pass = finite_ok && max_x <= x_max + 1e-9 && min_x >= x_min - 1e-9;
    else
        stress_margin = x_soft + 0.05;
        case_pass = finite_ok && max_x <= x_max + stress_margin && min_x >= x_min - stress_margin;
    end
    overall_pass = overall_pass && case_pass;

    case_dir = fullfile(output_dir, c.name);
    if ~exist(case_dir, 'dir')
        mkdir(case_dir);
    end

    T = table(result.t, result.state(:, 1), result.state(:, 2), ...
              result.state(:, 3), result.state(:, 4), ...
              result.state(:, 5), result.state(:, 6), ...
              result.u_cmd, result.u_actual, rail_force, string(result.mode), ...
              'VariableNames', {'t_s', 'x_m', 'x_dot_m_s', ...
                                'theta1_rad', 'theta1_dot_rad_s', ...
                                'theta2_rad', 'theta2_dot_rad_s', ...
                                'u_cmd_N', 'u_actual_N', 'rail_force_N', 'mode'});
    writetable(T, fullfile(case_dir, [c.name '_log.csv']));
    save(fullfile(case_dir, [c.name '_result.mat']), 'result', 'cfg', 'rail_force');

    plot_results(result, fullfile(case_dir, [c.name '_standard_plot.png']));
    local_plot_x_with_rail(result.t, result.state(:, 1), rail_force, result.u_actual, rail, ...
                           fullfile(case_dir, [c.name '_rail_limit_plot.png']), c.name);

    summary_lines(i) = sprintf('%s: pass=%d, min_x=%.12g, max_x=%.12g, max_abs_rail_force=%.12g', ...
                               c.name, case_pass, min_x, max_x, max_abs_rail_force);
end

summary_path = fullfile(output_dir, 'phase23_rail_limit_summary.txt');
fid = fopen(summary_path, 'w');
if fid < 0
    error('run_rail_limit_test:FileOpenFailed', 'Cannot open summary file.');
end
cleanup_obj = onCleanup(@() fclose(fid));
fprintf(fid, 'Phase 23 MATLAB rail limit test\n');
fprintf(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf(fid, 'angle_convention = theta = 0 downward, theta = pi upright\n');
fprintf(fid, 'x_min_m = %.12g\n', x_min);
fprintf(fid, 'x_max_m = %.12g\n', x_max);
fprintf(fid, 'x_soft_m = %.12g\n', x_soft);
fprintf(fid, 'k_wall_N_per_m = %.12g\n', double(rail.k_wall_N_per_m));
fprintf(fid, 'c_wall_N_s_per_m = %.12g\n', double(rail.c_wall_N_s_per_m));
fprintf(fid, 'max_wall_force_N = %.12g\n', double(rail.max_wall_force_N));
for i = 1:numel(summary_lines)
    fprintf(fid, '%s\n', summary_lines(i));
end
if overall_pass
    fprintf(fid, 'compare_result = PASS\n');
else
    fprintf(fid, 'compare_result = FAIL\n');
end
clear cleanup_obj;

for i = 1:numel(summary_lines)
    fprintf('%s\n', summary_lines(i));
end
fprintf('Saved summary: %s\n', summary_path);

if overall_pass
    fprintf('Phase 23 run_rail_limit_test: PASS\n');
else
    error('Phase 23 run_rail_limit_test: FAIL. See summary: %s', summary_path);
end

function cases = local_build_cases(params)
    target = params.default_initial_state;
    base_angles = [target.theta1_rad; target.theta1_dot_rad_s; target.theta2_rad; target.theta2_dot_rad_s];

    cases = struct([]);

    cases(1).name = 'near_right_boundary';
    cases(1).initial_state = [1.88; 0.04; base_angles];
    cases(1).t_final = 2.5;
    cases(1).controller = @(t, x, params, cfg) 0.0;
    cases(1).strict_inside = true;

    cases(2).name = 'near_left_boundary';
    cases(2).initial_state = [-1.88; -0.04; base_angles];
    cases(2).t_final = 2.5;
    cases(2).controller = @(t, x, params, cfg) 0.0;
    cases(2).strict_inside = true;

    cases(3).name = 'right_push_stress';
    cases(3).initial_state = [1.75; 0.0; base_angles];
    cases(3).t_final = 3.0;
    cases(3).controller = @(t, x, params, cfg) local_pulse_force(t, 8.0, 0.25);
    cases(3).strict_inside = false;
end

function u = local_pulse_force(t, amplitude, duration)
    if t <= duration
        u = amplitude;
    else
        u = 0.0;
    end
end

function rail_force = local_compute_rail_force_history(state_history, params)
    n = size(state_history, 1);
    rail_force = zeros(n, 1);
    for k = 1:n
        rail_force(k) = dip_rail_limit(state_history(k, 1), state_history(k, 2), params);
    end
end

function local_plot_x_with_rail(t, x, rail_force, u_actual, rail, save_path, case_name)
    fig = figure('Name', ['Phase 23 rail limit - ' case_name], 'Color', 'w');
    tiledlayout(3, 1);

    nexttile;
    plot(t, x, 'LineWidth', 1.2);
    hold on;
    yline(double(rail.x_min_m), '--', 'x min');
    yline(double(rail.x_max_m), '--', 'x max');
    yline(double(rail.x_min_m) + double(rail.x_soft_m), ':', 'left soft');
    yline(double(rail.x_max_m) - double(rail.x_soft_m), ':', 'right soft');
    grid on;
    ylabel('x [m]');
    title(['Phase 23 rail limit: ' strrep(case_name, '_', '\_')]);

    nexttile;
    plot(t, u_actual, 'LineWidth', 1.2);
    grid on;
    ylabel('u actual [N]');

    nexttile;
    plot(t, rail_force, 'LineWidth', 1.2);
    grid on;
    ylabel('rail force [N]');
    xlabel('time [s]');

    save_dir = fileparts(save_path);
    if ~isempty(save_dir) && ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    saveas(fig, save_path);
end
