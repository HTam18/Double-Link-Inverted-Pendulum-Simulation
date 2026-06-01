% RUN_PHASE29_PREPARE_TRACKING_TRAJECTORY Create a validated force-reserve trajectory for Phase 29.
%
% Clean rule:
%   A tracking-ready trajectory is accepted only if its exact nonlinear RK4-ZOH
%   replay reaches the upright target.  The script optimizes to a candidate file
%   first and copies it to up_up_swingup_tracking_ready.mat only after validation.
%
% Resume/checkpoint behavior:
%   - Completed candidate .mat files are loaded and validated instead of rerun.
%   - In-progress fmincon candidates write optimizer_checkpoint.mat every
%     iteration. If MATLAB is interrupted, rerunning this script resumes that
%     candidate from the last checkpoint as the new initial guess.
%   - Previous candidates are skipped automatically, so an interruption during
%     reserve_12N_T12_N101 does not rerun reserve_12N_T10_N91.
%
% Optional variables to set before running:
%   phase29_resume_candidate_name = 'reserve_12N_T12_N101';
%   phase29_force_rerun_completed_candidates = false;
%   phase29_force_continue_after_final = false;
%
% Usage:
%   run('matlab/startup_project.m')
%   run('matlab/simulation/run_phase29_prepare_tracking_trajectory.m')
%   run('matlab/simulation/run_tvlqr_tracking_test.m')

this_file = mfilename('fullpath');
simulation_dir = fileparts(this_file);
matlab_root = fileparts(simulation_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

params = load_params();
trajectory_dir = fullfile(project_root, 'shared', 'trajectories');
result_dir = fullfile(project_root, 'results', 'phase29_tracking_ready_trajectory');
status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE29_TRACKING_READY_TRAJECTORY.md');
final_file = fullfile(trajectory_dir, 'up_up_swingup_tracking_ready.mat');
progress_file = fullfile(result_dir, 'phase29_prepare_progress.mat');
if ~exist(trajectory_dir, 'dir'); mkdir(trajectory_dir); end
if ~exist(result_dir, 'dir'); mkdir(result_dir); end

fprintf('Phase 29 preparation: validated force-reserve optimized trajectory\n');
fprintf('target = up_up\n');
fprintf('purpose = make trajectory feasible for TVLQR + actuator delay\n');
fprintf('acceptance_rule = exact_replay_pass_required_before_saving_final_tracking_ready_file\n');
fprintf('resume_policy = skip_completed_candidates_and_resume_optimizer_checkpoint\n');
fprintf('progress_file = %s\n', progress_file);

resume_candidate_name = local_get_workspace_string('phase29_resume_candidate_name', '');
force_rerun_completed = local_get_workspace_bool('phase29_force_rerun_completed_candidates', false);
force_continue_after_final = local_get_workspace_bool('phase29_force_continue_after_final', false);

fprintf('phase29_resume_candidate_name = %s\n', local_print_empty_as_auto(resume_candidate_name));
fprintf('phase29_force_rerun_completed_candidates = %d\n', force_rerun_completed);
fprintf('phase29_force_continue_after_final = %d\n', force_continue_after_final);

candidates = local_candidate_list();
all_results = repmat(local_result_template(), 0, 1);
accepted = false;
accepted_result = local_result_template();

% If the final accepted file already exists and the user did not ask to keep
% searching, validate it and stop. This prevents accidental recomputation.
if ~force_continue_after_final && exist(final_file, 'file') == 2
    [loaded, final_one] = local_load_validate_candidate_file(final_file, params, local_final_file_candidate());
    fprintf('existing_final_tracking_ready_file_found = %d\n', loaded);
    if loaded && final_one.exact_replay_pass && final_one.force_reserve_N >= 2.0
        accepted = true;
        accepted_result = final_one;
        accepted_result.name = 'existing_final_up_up_swingup_tracking_ready';
        all_results(end + 1, 1) = accepted_result; %#ok<SAGROW>
        local_write_status(status_file, accepted, accepted_result, all_results, final_file);
        local_save_progress(progress_file, 'final_file_already_valid', 0, accepted_result, all_results, final_file);
        local_print_final_summary(accepted, accepted_result, final_file);
        return;
    else
        fprintf('existing_final_tracking_ready_file_valid = 0\n');
        fprintf('existing_final_tracking_ready_file_will_not_be_used = %s\n', final_file);
    end
end

% Manual resume starts at the requested candidate. Auto mode still skips any
% completed candidate files it sees, so this variable is optional.
if strlength(string(resume_candidate_name)) > 0
    names = arrayfun(@(c) string(c.name), candidates);
    resume_idx = find(names == string(resume_candidate_name), 1);
    if isempty(resume_idx)
        error('Phase29Prepare:ResumeCandidateNotFound', 'Resume candidate not found: %s', resume_candidate_name);
    end
    fprintf('resume_candidate_start_index = %d\n', resume_idx);
    candidates = candidates(resume_idx:end);
else
    fprintf('resume_candidate_start_index = auto\n');
end

for i = 1:numel(candidates)
    cand = candidates(i);
    cfg = local_candidate_to_optimizer_cfg(cand, project_root, result_dir, trajectory_dir);
    fprintf('\ntracking_ready_candidate_index = %d\n', i);
    fprintf('tracking_ready_candidate_name = %s\n', cand.name);
    fprintf('tracking_ready_candidate_file = %s\n', cfg.trajectory_file);
    fprintf('tracking_ready_candidate_checkpoint_file = %s\n', cfg.checkpoint_file);

    local_save_progress(progress_file, 'candidate_start', i, local_result_template_with_name(cand.name, cfg.trajectory_file), all_results, final_file);

    % Completed candidate cache: do not rerun T10_N91 if it already produced a
    % .mat file. Validate the file and move on, or accept it if it passes.
    if ~force_rerun_completed && exist(cfg.trajectory_file, 'file') == 2
        fprintf('tracking_ready_candidate_cached_file_found = 1\n');
        [loaded, one] = local_load_validate_candidate_file(cfg.trajectory_file, params, cand);
        if loaded
            local_print_candidate_result(one);
            all_results(end + 1, 1) = one; %#ok<SAGROW>
            local_save_progress(progress_file, 'candidate_cached_validated', i, one, all_results, final_file);
            if local_candidate_is_acceptable(one)
                local_save_final_tracking_ready(final_file, cfg.trajectory_file, one);
                accepted = true;
                accepted_result = one;
                fprintf('tracking_ready_candidate_accepted = 1\n');
                break;
            else
                fprintf('tracking_ready_candidate_accepted = 0\n');
                continue;
            end
        else
            fprintf('tracking_ready_candidate_cached_file_loadable = 0\n');
            fprintf('tracking_ready_candidate_cached_file_will_be_rerun = 1\n');
        end
    else
        fprintf('tracking_ready_candidate_cached_file_found = 0\n');
    end

    try
        opt_result = optimize_swingup_direct_collocation(cfg);
    catch ME
        interrupted_one = local_result_template_with_name(cand.name, cfg.trajectory_file);
        interrupted_one.status = 'optimizer_interrupted_or_failed';
        interrupted_one.message = ME.message;
        local_save_progress(progress_file, 'candidate_interrupted_or_failed', i, interrupted_one, all_results, final_file);
        fprintf('tracking_ready_candidate_interrupted_or_failed = 1\n');
        fprintf('tracking_ready_candidate_failure_message = %s\n', ME.message);
        fprintf('resume_hint = rerun this script; it will skip completed candidates and use optimizer_checkpoint.mat if available\n');
        rethrow(ME);
    end

    trajectory = opt_result.trajectory;
    validation = local_validate_tracking_ready_trajectory(trajectory, params, cand);

    one = local_result_template();
    one.name = cand.name;
    one.optimization_usable = opt_result.metrics.optimization_usable;
    one.exact_replay_pass = validation.exact_replay_pass;
    one.final_max_angle_error_rad = validation.final_max_angle_error_rad;
    one.final_velocity_norm = validation.final_velocity_norm;
    one.max_abs_x_m = validation.max_abs_x_m;
    one.max_abs_u_N = validation.max_abs_u_N;
    one.force_reserve_N = validation.force_reserve_N;
    one.candidate_file = cfg.trajectory_file;
    one.checkpoint_file = cfg.checkpoint_file;
    one.status = 'optimized_and_validated';
    one.message = '';
    all_results(end + 1, 1) = one; %#ok<SAGROW>

    local_print_candidate_result(one);
    local_save_progress(progress_file, 'candidate_completed', i, one, all_results, final_file);

    if local_candidate_is_acceptable(one)
        local_save_final_tracking_ready(final_file, cfg.trajectory_file, one);
        accepted = true;
        accepted_result = one;
        fprintf('tracking_ready_candidate_accepted = 1\n');
        break;
    else
        fprintf('tracking_ready_candidate_accepted = 0\n');
    end
end

local_write_status(status_file, accepted, accepted_result, all_results, final_file);
local_save_progress(progress_file, 'finished', numel(candidates), accepted_result, all_results, final_file);
local_print_final_summary(accepted, accepted_result, final_file);

function candidates = local_candidate_list()
    candidates = local_candidate('reserve_12N_T10_N91', 91, 10.0, 12.0, 0.10, 2.0e4, 1.5e3, 5.0e-2, 5.0e-2, 5.0);
    candidates(end+1) = local_candidate('reserve_12N_T12_N101', 101, 12.0, 12.0, 0.10, 2.5e4, 2.0e3, 6.0e-2, 8.0e-2, 6.0);
    candidates(end+1) = local_candidate('reserve_11N_T12_N121', 121, 12.0, 11.0, 0.12, 3.0e4, 2.5e3, 7.0e-2, 1.0e-1, 7.0);
    candidates(end+1) = local_candidate('reserve_11p5N_T14_N121', 121, 14.0, 11.5, 0.12, 3.0e4, 2.5e3, 7.0e-2, 1.2e-1, 7.0);
end

function r = local_result_template()
    r = struct();
    r.name = '';
    r.optimization_usable = false;
    r.exact_replay_pass = false;
    r.final_max_angle_error_rad = NaN;
    r.final_velocity_norm = NaN;
    r.max_abs_x_m = NaN;
    r.max_abs_u_N = NaN;
    r.force_reserve_N = NaN;
    r.candidate_file = '';
    r.checkpoint_file = '';
    r.status = '';
    r.message = '';
end

function r = local_result_template_with_name(name, candidate_file)
    r = local_result_template();
    r.name = name;
    r.candidate_file = candidate_file;
end

function cand = local_candidate(name, N, T, umax, x_margin, w_angle, w_vel, w_force, w_smooth, w_center)
    cand = struct();
    cand.name = name;
    cand.N = N;
    cand.T = T;
    cand.umax = umax;
    cand.x_margin = x_margin;
    cand.w_terminal_angle = w_angle;
    cand.w_terminal_velocity = w_vel;
    cand.w_force = w_force;
    cand.w_force_smooth = w_smooth;
    cand.w_cart_center = w_center;
end

function cand = local_final_file_candidate()
    cand = local_candidate('existing_final_up_up_swingup_tracking_ready', NaN, NaN, 15.0, NaN, NaN, NaN, NaN, NaN, NaN);
end

function cfg = local_candidate_to_optimizer_cfg(cand, project_root, result_dir, trajectory_dir)
    cfg = struct();
    cfg.N = cand.N;
    cfg.T = cand.T;
    cfg.u_min = -cand.umax;
    cfg.u_max = cand.umax;
    cfg.x_margin = cand.x_margin;
    cfg.terminal_angle_tolerance_rad = 0.18;
    cfg.terminal_velocity_norm_limit = 1.5;
    cfg.terminal_cart_abs_limit_m = 0.60;
    cfg.usable_angle_tolerance_rad = 0.25;
    cfg.usable_velocity_norm_limit = 2.5;
    cfg.w_terminal_angle = cand.w_terminal_angle;
    cfg.w_terminal_velocity = cand.w_terminal_velocity;
    cfg.w_terminal_cart = 8.0e2;
    cfg.w_force = cand.w_force;
    cfg.w_force_smooth = cand.w_force_smooth;
    cfg.w_cart_center = cand.w_cart_center;
    cfg.w_velocity = 2.0e-2;
    cfg.max_iterations = 650;
    cfg.max_function_evaluations = 550000;
    cfg.display = 'iter';
    cfg.result_dir = fullfile(result_dir, cand.name);
    cfg.trajectory_dir = trajectory_dir;
    cfg.trajectory_file = fullfile(trajectory_dir, ['up_up_swingup_tracking_ready_candidate_' cand.name '.mat']);
    cfg.summary_file = fullfile(cfg.result_dir, 'direct_collocation_summary.txt');
    cfg.convergence_file = fullfile(cfg.result_dir, 'direct_collocation_convergence.mat');
    cfg.trajectory_plot_file = fullfile(cfg.result_dir, 'trajectory.png');
    cfg.status_file = fullfile(project_root, 'docs', ['PROJECT_STATUS_PHASE29_TRACKING_READY_' cand.name '.md']);
    cfg.checkpoint_file = fullfile(cfg.result_dir, 'optimizer_checkpoint.mat');
    cfg.resume_from_checkpoint = true;
    cfg.candidate_name = cand.name;
end

function [loaded, one] = local_load_validate_candidate_file(candidate_file, params, cand)
    one = local_result_template_with_name(cand.name, candidate_file);
    loaded = false;
    if exist(candidate_file, 'file') ~= 2
        return;
    end
    try
        data = load(candidate_file);
        if isfield(data, 'trajectory')
            trajectory = data.trajectory;
        elseif isfield(data, 'opt_result') && isfield(data.opt_result, 'trajectory')
            trajectory = data.opt_result.trajectory;
        else
            one.message = 'file_does_not_contain_trajectory';
            return;
        end
        validation = local_validate_tracking_ready_trajectory(trajectory, params, cand);
        one.exact_replay_pass = validation.exact_replay_pass;
        one.final_max_angle_error_rad = validation.final_max_angle_error_rad;
        one.final_velocity_norm = validation.final_velocity_norm;
        one.max_abs_x_m = validation.max_abs_x_m;
        one.max_abs_u_N = validation.max_abs_u_N;
        one.force_reserve_N = validation.force_reserve_N;
        one.optimization_usable = local_get_metric_bool(trajectory, data, 'optimization_usable', true);
        one.status = 'loaded_from_existing_candidate_file';
        one.message = '';
        loaded = true;
    catch ME
        one.status = 'candidate_file_load_or_validation_failed';
        one.message = ME.message;
    end
end

function value = local_get_metric_bool(trajectory, data, field_name, default_value)
    value = default_value;
    try
        if isfield(trajectory, 'metrics') && isfield(trajectory.metrics, field_name)
            value = logical(trajectory.metrics.(field_name));
        elseif isfield(data, 'opt_result') && isfield(data.opt_result, 'metrics') && isfield(data.opt_result.metrics, field_name)
            value = logical(data.opt_result.metrics.(field_name));
        end
    catch
        value = default_value;
    end
end

function validation = local_validate_tracking_ready_trajectory(trajectory, params, cand)
    replay_result = replay_optimized_trajectory_exact(trajectory, params);
    X = double(replay_result.state);
    U = double(replay_result.u_actual(:));
    validation = struct();
    validation.final_max_angle_error_rad = max(abs([local_wrap_to_pi(X(end,3)-pi), local_wrap_to_pi(X(end,5)-pi)]));
    validation.final_velocity_norm = norm([X(end,2), X(end,4), X(end,6)]);
    validation.max_abs_x_m = max(abs(X(:,1)));
    validation.max_abs_u_N = max(abs(U));
    validation.force_reserve_N = 15.0 - validation.max_abs_u_N;
    validation.finite_pass = all(isfinite(X(:))) && all(isfinite(U(:)));
    validation.rail_pass = validation.max_abs_x_m <= 2.0 + 1e-9;
    if isfinite(cand.umax)
        validation.force_pass = validation.max_abs_u_N <= cand.umax + 1e-6;
    else
        validation.force_pass = validation.max_abs_u_N <= 15.0 + 1e-6;
    end
    validation.handoff_pass = validation.final_max_angle_error_rad <= 0.70 && validation.final_velocity_norm <= 5.5;
    validation.exact_replay_pass = validation.finite_pass && validation.rail_pass && validation.force_pass && validation.handoff_pass;
end

function ok = local_candidate_is_acceptable(one)
    ok = one.optimization_usable && one.exact_replay_pass && one.force_reserve_N >= 2.0;
end

function local_save_final_tracking_ready(final_file, candidate_file, one)
    data = load(candidate_file);
    trajectory = data.trajectory;
    if isfield(data, 'opt_result')
        opt_result = data.opt_result;
    else
        opt_result = struct();
    end
    validation = one; %#ok<NASGU>
    save(final_file, 'trajectory', 'opt_result', 'validation');
end

function local_print_candidate_result(one)
    fprintf('tracking_ready_candidate_optimization_usable = %d\n', one.optimization_usable);
    fprintf('tracking_ready_candidate_exact_replay_pass = %d\n', one.exact_replay_pass);
    fprintf('tracking_ready_candidate_final_max_angle_error_rad = %.12g\n', one.final_max_angle_error_rad);
    fprintf('tracking_ready_candidate_final_velocity_norm = %.12g\n', one.final_velocity_norm);
    fprintf('tracking_ready_candidate_max_abs_x_m = %.12g\n', one.max_abs_x_m);
    fprintf('tracking_ready_candidate_max_abs_u_N = %.12g\n', one.max_abs_u_N);
    fprintf('tracking_ready_candidate_force_reserve_N = %.12g\n', one.force_reserve_N);
end

function local_print_final_summary(accepted, accepted_result, final_file)
    fprintf('\ntracking_ready_accepted = %d\n', accepted);
    if accepted
        fprintf('tracking_ready_selected_candidate = %s\n', accepted_result.name);
        fprintf('tracking_ready_final_max_angle_error_rad = %.12g\n', accepted_result.final_max_angle_error_rad);
        fprintf('tracking_ready_final_velocity_norm = %.12g\n', accepted_result.final_velocity_norm);
        fprintf('tracking_ready_max_abs_x_m = %.12g\n', accepted_result.max_abs_x_m);
        fprintf('tracking_ready_max_abs_u_N = %.12g\n', accepted_result.max_abs_u_N);
        fprintf('tracking_ready_force_reserve_N = %.12g\n', accepted_result.force_reserve_N);
        fprintf('tracking_ready_trajectory_file = %s\n', final_file);
    else
        fprintf('tracking_ready_selected_candidate = NONE\n');
        fprintf('tracking_ready_failure_reason = no_candidate_met_exact_replay_and_force_reserve_gates\n');
        fprintf('tracking_ready_trajectory_file_not_updated = %s\n', final_file);
    end
end

function local_save_progress(path, stage, candidate_index, current_result, all_results, final_file)
    try
        progress = struct(); %#ok<NASGU>
        progress.stage = stage;
        progress.candidate_index = candidate_index;
        progress.current_result = current_result;
        progress.all_results = all_results;
        progress.final_file = final_file;
        progress.timestamp = char(datetime('now'));
        save(path, 'progress');
    catch ME
        warning('Phase29Prepare:ProgressSaveFailed', 'Could not save progress file %s: %s', path, ME.message);
    end
end

function local_write_status(path, accepted, accepted_result, all_results, final_file)
    fid = fopen(path, 'w');
    if fid < 0
        warning('Phase29Prepare:StatusOpenFailed', 'Could not write %s', path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# PROJECT STATUS - PHASE 29 TRACKING READY TRAJECTORY\n\n');
    fprintf(fid, 'accepted: `%d`  \n', accepted);
    fprintf(fid, 'final tracking ready file: `%s`  \n\n', final_file);
    fprintf(fid, 'Resume behavior: completed candidate files are reused, and in-progress `fmincon` runs write `optimizer_checkpoint.mat` for restart.  \n\n');
    if accepted
        fprintf(fid, 'selected candidate: `%s`  \n', accepted_result.name);
        fprintf(fid, 'final angle error: `%.12g`  \n', accepted_result.final_max_angle_error_rad);
        fprintf(fid, 'final velocity norm: `%.12g`  \n', accepted_result.final_velocity_norm);
        fprintf(fid, 'max abs x: `%.12g`  \n', accepted_result.max_abs_x_m);
        fprintf(fid, 'max abs u: `%.12g`  \n', accepted_result.max_abs_u_N);
        fprintf(fid, 'force reserve: `%.12g`  \n\n', accepted_result.force_reserve_N);
    else
        fprintf(fid, 'No candidate met exact replay and force reserve gates. Do not use stale `up_up_swingup_tracking_ready.mat` for Phase 29.  \n\n');
    end
    fprintf(fid, '## Candidate table\n\n');
    fprintf(fid, '| name | opt usable | exact replay | final angle | final vel | max x | max u | reserve | status |\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---|\n');
    for i = 1:numel(all_results)
        r = all_results(i);
        fprintf(fid, '| %s | %d | %d | %.6g | %.6g | %.6g | %.6g | %.6g | %s |\n', ...
            r.name, r.optimization_usable, r.exact_replay_pass, r.final_max_angle_error_rad, ...
            r.final_velocity_norm, r.max_abs_x_m, r.max_abs_u_N, r.force_reserve_N, r.status);
    end
end

function value = local_get_workspace_string(name, default_value)
    value = default_value;
    try
        exists_in_caller = evalin('caller', sprintf('exist(''%s'', ''var'')', name));
        exists_in_base = evalin('base', sprintf('exist(''%s'', ''var'')', name));
        if exists_in_caller
            value = char(evalin('caller', name));
        elseif exists_in_base
            value = char(evalin('base', name));
        end
    catch
        value = default_value;
    end
end

function value = local_get_workspace_bool(name, default_value)
    value = default_value;
    try
        exists_in_caller = evalin('caller', sprintf('exist(''%s'', ''var'')', name));
        exists_in_base = evalin('base', sprintf('exist(''%s'', ''var'')', name));
        if exists_in_caller
            value = logical(evalin('caller', name));
        elseif exists_in_base
            value = logical(evalin('base', name));
        end
    catch
        value = default_value;
    end
end

function s = local_print_empty_as_auto(value)
    if strlength(string(value)) == 0
        s = 'auto';
    else
        s = char(value);
    end
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2.0 * pi) - pi;
end
