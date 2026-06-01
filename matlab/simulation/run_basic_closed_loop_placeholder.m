% RUN_BASIC_CLOSED_LOOP_PLACEHOLDER Phase 21 controller plumbing smoke test.
%
% This is not the final LQR or hybrid controller. It only verifies that the
% MATLAB runner can call a controller handle, log mode/u_cmd/u_actual, and save
% results for later phases.
%
% How to run from project root:
%   run('matlab/startup_project.m')
%   run('matlab/simulation/run_basic_closed_loop_placeholder.m')

clearvars -except ans;

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
run(fullfile(matlab_root, 'startup_project.m'));

[params, meta] = load_params();

out_dir = fullfile(meta.project_root, 'results', 'phase21_closed_loop_placeholder');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

cfg = struct();
cfg.mode = 'closed_loop_placeholder';
cfg.dt = 0.005;
cfg.t_final = 5.0;
cfg.initial_state = [0.12; 0.0; 0.08; 0.0; -0.06; 0.0];
cfg.controller = @local_placeholder_controller;

result = run_simulation(cfg, params);

if ~result.pass
    error('run_basic_closed_loop_placeholder:ResultFailed', 'run_simulation returned pass=false.');
end
if any(~isfinite(result.state(:))) || any(~isfinite(result.u_actual(:)))
    error('run_basic_closed_loop_placeholder:NonFiniteOutput', 'Result contains NaN or Inf.');
end

T = table(result.t, result.state(:, 1), result.state(:, 2), result.state(:, 3), ...
          result.state(:, 4), result.state(:, 5), result.state(:, 6), ...
          result.u_cmd, result.u_actual, string(result.mode), ...
          'VariableNames', {'t', 'x', 'x_dot', 'theta1', 'theta1_dot', ...
                            'theta2', 'theta2_dot', 'u_cmd', 'u_actual', 'mode'});
writetable(T, fullfile(out_dir, 'phase21_closed_loop_placeholder_result.csv'));

plot_results(result, fullfile(out_dir, 'phase21_closed_loop_placeholder_plot.png'));

summary_file = fullfile(out_dir, 'phase21_closed_loop_placeholder_summary.txt');
fid = fopen(summary_file, 'w');
if fid < 0
    error('run_basic_closed_loop_placeholder:FileOpenFailed', 'Cannot open summary file.');
end
cleanup_obj = onCleanup(@() fclose(fid));

fprintf(fid, 'Phase 21 basic closed-loop placeholder test\n');
fprintf(fid, 'This is not final LQR/hybrid control. It tests runner/controller/logger plumbing only.\n');
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

fprintf('Phase 21 run_basic_closed_loop_placeholder: PASS\n');
fprintf('Saved CSV:     %s\n', fullfile(out_dir, 'phase21_closed_loop_placeholder_result.csv'));
fprintf('Saved summary: %s\n', summary_file);
fprintf('Saved plot:    %s\n', fullfile(out_dir, 'phase21_closed_loop_placeholder_plot.png'));

function out = local_placeholder_controller(t, state, params, cfg) %#ok<INUSD>
    % Simple cart damping/centering only. This deliberately avoids final LQR
    % logic because Phase 22 is responsible for LQR stabilize control.
    x = state(1);
    x_dot = state(2);
    u = -1.2 * x - 1.8 * x_dot;

    out = struct();
    out.u_cmd = u;
    out.mode = 'closed_loop_placeholder';
    out.measurement = state;
    out.x_hat = state;
end
