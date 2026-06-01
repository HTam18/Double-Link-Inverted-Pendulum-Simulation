% RUN_OPEN_LOOP_TEST Phase 21 reusable runner open-loop smoke test.
%
% How to run from project root:
%   run('matlab/startup_project.m')
%   run('matlab/simulation/run_open_loop_test.m')

clearvars -except ans;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
run(fullfile(matlab_root, 'startup_project.m'));

[params, meta] = load_params();

out_dir = fullfile(meta.project_root, 'results', 'phase21_open_loop');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

cfg = struct();
cfg.mode = 'open_loop';
cfg.dt = 0.005;
cfg.t_final = 5.0;
cfg.initial_state = [0.0; 0.0; 0.18; 0.0; -0.12; 0.0];
cfg.controller = @(t, x, params, cfg) local_open_loop_input(t);

result = run_simulation(cfg, params);

if ~result.pass
    error('run_open_loop_test:ResultFailed', 'run_simulation returned pass=false.');
end
if any(~isfinite(result.state(:))) || any(~isfinite(result.u_actual(:)))
    error('run_open_loop_test:NonFiniteOutput', 'Result contains NaN or Inf.');
end

T = table(result.t, result.state(:, 1), result.state(:, 2), result.state(:, 3), ...
          result.state(:, 4), result.state(:, 5), result.state(:, 6), ...
          result.u_cmd, result.u_actual, string(result.mode), ...
          'VariableNames', {'t', 'x', 'x_dot', 'theta1', 'theta1_dot', ...
                            'theta2', 'theta2_dot', 'u_cmd', 'u_actual', 'mode'});
writetable(T, fullfile(out_dir, 'phase21_open_loop_result.csv'));

plot_results(result, fullfile(out_dir, 'phase21_open_loop_plot.png'));

summary_file = fullfile(out_dir, 'phase21_open_loop_summary.txt');
fid = fopen(summary_file, 'w');
if fid < 0
    error('run_open_loop_test:FileOpenFailed', 'Cannot open summary file.');
end
cleanup_obj = onCleanup(@() fclose(fid));

fprintf(fid, 'Phase 21 open-loop runner test\n');
fprintf(fid, 'State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
fprintf(fid, 'Angle convention: theta = 0 downward, theta = pi upright\n');
fprintf(fid, 'dt = %.6f s\n', cfg.dt);
fprintf(fid, 't_final = %.6f s\n', cfg.t_final);
fprintf(fid, 'max_abs_state = %.12g\n', max(abs(result.state(:))));
fprintf(fid, 'max_abs_u_cmd = %.12g\n', max(abs(result.u_cmd)));
fprintf(fid, 'max_abs_u_actual = %.12g\n', max(abs(result.u_actual)));
fprintf(fid, 'final_state = [%s]\n', sprintf(' %.12g', result.state(end, :)));
fprintf(fid, 'result = PASS\n');
clear cleanup_obj;

fprintf('Phase 21 run_open_loop_test: PASS\n');
fprintf('Saved CSV:     %s\n', fullfile(out_dir, 'phase21_open_loop_result.csv'));
fprintf('Saved summary: %s\n', summary_file);
fprintf('Saved plot:    %s\n', fullfile(out_dir, 'phase21_open_loop_plot.png'));

function u = local_open_loop_input(t)
    u = 1.5 * sin(2*pi*0.45*t) + 0.35 * sin(2*pi*1.2*t);
end
