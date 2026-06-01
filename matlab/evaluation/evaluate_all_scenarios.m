% EVALUATE_ALL_SCENARIOS Phase 33 MATLAB/Simulink evaluation runner.
%
% This script converts the technical results from Phase 29 to Phase 32 into
% report-ready metrics, figures and a short engineering evaluation report.
%
% Main scenarios:
%   S1 - Phase 29 open-loop feedforward baseline.
%   S2 - Phase 29 ideal discrete TVLQR tracking.
%   S3 - Phase 29 TVLQR with actuator delay.
%   S4 - Phase 29 TVLQR with actuator delay and sensor noise.
%   S5 - Phase 30 full hybrid tracking -> handoff -> balance.
%   S6 - Phase 31 standard Monte Carlo robustness summary.
%   S7 - Phase 32 Simulink model compared with MATLAB Phase 30.
%
% Usage:
%   run('matlab/startup_project.m')
%   run('matlab/evaluation/evaluate_all_scenarios.m')
%
% Optional variables before running:
%   phase33_run_missing_prerequisites = true;   % default true
%   phase33_force_rerun = false;                % default false
%   phase33_num_monte_carlo_runs = 30;          % default 30
%
% Outputs:
%   results/phase33_evaluation/phase33_evaluation_data.mat
%   results/phase33_evaluation/phase33_summary_metrics.csv
%   results/phase33_evaluation/phase33_result_summary.txt
%   results/phase33_evaluation/figures/*.png
%   docs/MATLAB_SIMULINK_EVALUATION_REPORT.md
%   docs/PROJECT_STATUS_PHASE33.md

this_file = mfilename('fullpath');
eval_dir = fileparts(this_file);
matlab_root = fileparts(eval_dir);
project_root = fileparts(matlab_root);
run(fullfile(matlab_root, 'startup_project.m'));

if ~exist('phase33_run_missing_prerequisites', 'var') || isempty(phase33_run_missing_prerequisites)
    phase33_run_missing_prerequisites = true;
end
if ~exist('phase33_force_rerun', 'var') || isempty(phase33_force_rerun)
    phase33_force_rerun = false;
end
if ~exist('phase33_num_monte_carlo_runs', 'var') || isempty(phase33_num_monte_carlo_runs)
    phase33_num_monte_carlo_runs = 30;
end

result_dir = fullfile(project_root, 'results', 'phase33_evaluation');
figure_dir = fullfile(result_dir, 'figures');
log_dir = fullfile(result_dir, 'logs');
if ~exist(result_dir, 'dir'), mkdir(result_dir); end
if ~exist(figure_dir, 'dir'), mkdir(figure_dir); end
if ~exist(log_dir, 'dir'), mkdir(log_dir); end

phase29_file = fullfile(project_root, 'results', 'phase29_tvlqr_tracking', 'tvlqr_tracking_result.mat');
phase30_file = fullfile(project_root, 'results', 'phase30_full_hybrid', 'full_hybrid_result.mat');
phase31_file = fullfile(project_root, 'results', 'phase31_monte_carlo', 'phase31_monte_carlo_results.mat');
phase32_file = fullfile(project_root, 'results', 'phase32_simulink', 'phase32_simulink_result.mat');

fprintf('Phase 33 MATLAB/Simulink evaluation\n');
fprintf('project_root = %s\n', project_root);
fprintf('phase33_result_dir = %s\n', result_dir);

if logical(phase33_run_missing_prerequisites)
    if logical(phase33_force_rerun) || ~exist(phase29_file, 'file')
        fprintf('Running prerequisite Phase 29 TVLQR test...\n');
        run(fullfile(matlab_root, 'simulation', 'run_tvlqr_tracking_test.m'));
    end
    if logical(phase33_force_rerun) || ~exist(phase30_file, 'file')
        fprintf('Running prerequisite Phase 30 hybrid test...\n');
        run(fullfile(matlab_root, 'simulation', 'run_full_hybrid_matlab.m'));
    end
    if logical(phase33_force_rerun) || ~exist(phase31_file, 'file')
        fprintf('Running prerequisite Phase 31 standard Monte Carlo test...\n');
        phase31_profile = 'standard'; %#ok<NASGU>
        phase31_num_runs = phase33_num_monte_carlo_runs; %#ok<NASGU>
        run(fullfile(matlab_root, 'simulation', 'run_monte_carlo_robustness.m'));
    end
    if logical(phase33_force_rerun) || ~exist(phase32_file, 'file')
        fprintf('Running prerequisite Phase 32 Simulink comparison test...\n');
        run(fullfile(matlab_root, 'simulink', 'init_simulink_model.m'));
        run(fullfile(matlab_root, 'simulink', 'run_simulink_test.m'));
    end
end

missing = {};
if ~exist(phase29_file, 'file'), missing{end+1} = phase29_file; end %#ok<AGROW>
if ~exist(phase30_file, 'file'), missing{end+1} = phase30_file; end %#ok<AGROW>
if ~exist(phase31_file, 'file'), missing{end+1} = phase31_file; end %#ok<AGROW>
if ~exist(phase32_file, 'file'), missing{end+1} = phase32_file; end %#ok<AGROW>
if ~isempty(missing)
    error('evaluate_all_scenarios:MissingResults', 'Missing required result files. First missing: %s', missing{1});
end

S29 = load(phase29_file);
S30 = load(phase30_file);
S31 = load(phase31_file);
S32 = load(phase32_file);

phase33 = struct();
phase33.project_root = project_root;
phase33.result_dir = result_dir;
phase33.figure_dir = figure_dir;
phase33.phase29_file = phase29_file;
phase33.phase30_file = phase30_file;
phase33.phase31_file = phase31_file;
phase33.phase32_file = phase32_file;
phase33.created_at = datestr(now, 31);
phase33.scenarios = local_collect_scenarios(S29, S30, S31, S32);

[summary_table, summary_struct] = generate_result_summary(phase33, result_dir, project_root);
phase33.summary_table = summary_table;
phase33.summary = summary_struct;

local_make_phase33_figures(figure_dir, S29, S30, S31, S32, summary_table);

mat_file = fullfile(result_dir, 'phase33_evaluation_data.mat');
save(mat_file, 'phase33', 'summary_table', 'summary_struct', 'phase29_file', 'phase30_file', 'phase31_file', 'phase32_file');

status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE33.md');
local_write_phase33_status(status_file, phase33, summary_struct, mat_file);

fprintf('phase33_scenario_count = %d\n', height(summary_table));
fprintf('phase33_required_scenarios_present = %d\n', summary_struct.required_scenarios_present);
fprintf('phase33_phase29_T4_pass = %d\n', summary_struct.phase29_T4_pass);
fprintf('phase33_phase30_pass = %d\n', summary_struct.phase30_pass);
fprintf('phase33_phase31_pass = %d\n', summary_struct.phase31_pass);
fprintf('phase33_phase32_pass = %d\n', summary_struct.phase32_pass);
fprintf('phase33_pass = %d\n', summary_struct.phase33_pass);
fprintf('phase33_result_file = %s\n', mat_file);
if summary_struct.phase33_pass
    fprintf('Phase 33 evaluate_all_scenarios: PASS\n');
else
    fprintf('Phase 33 evaluate_all_scenarios: REVIEW_REQUIRED\n');
    fprintf('phase33_fail_reason = %s\n', summary_struct.fail_reason);
end

function scenarios = local_collect_scenarios(S29, S30, S31, S32)
    % Create a typed empty struct first. In some MATLAB releases, assigning
    % a populated struct into plain struct([]) can throw "Subscripted
    % assignment between dissimilar structures". Starting from an empty
    % template guarantees that every appended scenario has the same field set.
    scenarios = local_scenario_template('', '', '', '');
    scenarios = scenarios([]);

    scenarios(1) = local_scenario_from_phase29('S1', 'Phase 29 T1 open-loop feedforward', 'Open-loop feedforward baseline without feedback. Used as baseline and expected to be weaker than TVLQR.', S29, {'T1_open_loop'});
    scenarios(end+1) = local_scenario_from_phase29('S2', 'Phase 29 T2 ideal TVLQR', 'Discrete TVLQR tracking with ideal force input.', S29, {'T2_ideal_tvlqr'});
    scenarios(end+1) = local_scenario_from_phase29('S3', 'Phase 29 T3 actuator delay TVLQR', 'Discrete TVLQR command through stable first-order actuator delay.', S29, {'T3_actuator_tvlqr'});
    scenarios(end+1) = local_scenario_from_phase29('S4', 'Phase 29 T4 actuator delay + sensor noise TVLQR', 'Mandatory Phase 33 scenario: TVLQR tracking with actuator delay and sensor noise.', S29, {'T4_noise_tvlqr'});
    scenarios(end+1) = local_scenario_from_phase30(S30);
    scenarios(end+1) = local_scenario_from_phase31(S31);
    scenarios(end+1) = local_scenario_from_phase32(S32);
end

function s = local_scenario_template(id, name, category, notes)
    s = struct();
    s.id = string(id);
    s.name = string(name);
    s.category = string(category);
    s.notes = string(notes);
    s.pass = false;
    s.success_rate = NaN;
    s.handoff_success = NaN;
    s.handoff_success_rate = NaN;
    s.stabilize_success = NaN;
    s.final_angle_error_rad = NaN;
    s.final_velocity_norm = NaN;
    s.settling_time_s = NaN;
    s.max_abs_x_m = NaN;
    s.max_abs_u_cmd_N = NaN;
    s.max_abs_u_actual_N = NaN;
    s.saturation_fraction = NaN;
    s.failure_reason = "none";
end

function s = local_scenario_from_phase29(id, name, notes, S29, metric_path)
    s = local_scenario_template(id, name, 'Phase 29 tracking', notes);
    m = local_get_nested(S29.metrics, metric_path, struct());
    if isempty(fieldnames(m))
        s.failure_reason = "missing_metric";
        return;
    end
    s.pass = local_get_any_bool(m, {'tracking_gate_pass','phase29_pass','success'}, false);
    s.handoff_success = double(local_get_any_bool(m, {'handoff_success','tracking_gate_pass'}, s.pass));
    s.stabilize_success = NaN;
    s.final_angle_error_rad = local_get_any(m, {'final_max_angle_error_rad','final_angle_error_rad','max_final_angle_error_rad'}, NaN);
    s.final_velocity_norm = local_get_any(m, {'final_velocity_norm','velocity_norm'}, NaN);
    s.settling_time_s = local_get_any(m, {'settling_time_s'}, NaN);
    s.max_abs_x_m = local_get_any(m, {'max_abs_x_m','max_abs_x'}, NaN);
    s.max_abs_u_cmd_N = local_get_any(m, {'max_abs_u_cmd_N','max_abs_u_N','max_abs_u'}, NaN);
    s.max_abs_u_actual_N = local_get_any(m, {'max_abs_u_actual_N','max_abs_u_N','max_abs_u'}, NaN);
    s.saturation_fraction = local_get_any(m, {'saturation_fraction','force_saturation_fraction'}, NaN);
    if ~s.pass
        s.failure_reason = "not_pass_or_baseline";
    end
end

function s = local_scenario_from_phase30(S30)
    s = local_scenario_template('S5', 'Phase 30 full hybrid MATLAB', 'Phase 30 hybrid', 'Full MATLAB pipeline: Phase 29 TVLQR tracking, handoff, terminal hold and balance.');
    m = S30.metrics;
    s.pass = logical(local_get_any(m, {'phase30_pass'}, false));
    s.handoff_success = double(local_get_any(m, {'handoff_success'}, false));
    s.stabilize_success = double(local_get_any(m, {'stabilize_success'}, false));
    s.final_angle_error_rad = local_get_any(m, {'final_angle_error_rad'}, NaN);
    s.final_velocity_norm = local_get_any(m, {'final_velocity_norm'}, NaN);
    s.max_abs_x_m = local_get_any(m, {'max_abs_x_m'}, NaN);
    s.max_abs_u_cmd_N = local_get_any(m, {'max_abs_u_cmd_N'}, NaN);
    s.max_abs_u_actual_N = local_get_any(m, {'max_abs_u_actual_N'}, NaN);
    s.saturation_fraction = local_get_any(m, {'saturation_fraction'}, NaN);
    s.failure_reason = string(local_get_any(m, {'fail_reason'}, 'none'));
end

function s = local_scenario_from_phase31(S31)
    s = local_scenario_template('S6', 'Phase 31 standard Monte Carlo robustness', 'Phase 31 robustness', 'Thirty-run standard Monte Carlo robustness test with randomized plant, noise, actuator delay, initial error and disturbance.');
    m = S31.summary;
    s.pass = logical(local_get_any(m, {'phase31_pass'}, false));
    s.success_rate = local_get_any(m, {'success_rate'}, NaN);
    s.handoff_success_rate = local_get_any(m, {'handoff_success_rate'}, NaN);
    s.stabilize_success = local_get_any(m, {'stabilize_success_rate'}, NaN);
    s.final_angle_error_rad = local_get_any(m, {'worst_final_angle_error_rad'}, NaN);
    s.final_velocity_norm = local_get_any(m, {'worst_final_velocity_norm'}, NaN);
    s.saturation_fraction = local_get_any(m, {'mean_saturation_fraction'}, NaN);
    if ~s.pass
        s.failure_reason = "robustness_review_required";
    end
end

function s = local_scenario_from_phase32(S32)
    s = local_scenario_template('S7', 'Phase 32 Simulink equivalence', 'Phase 32 Simulink', 'Main Simulink model compared against MATLAB Phase 30 state trajectory and final metrics.');
    m = S32.metrics;
    s.pass = logical(local_get_any(m, {'phase32_pass'}, false));
    s.handoff_success = double(local_get_any(m, {'handoff_success'}, false));
    s.stabilize_success = double(local_get_any(m, {'stabilize_success'}, false));
    s.final_angle_error_rad = local_get_any(m, {'final_angle_error_rad'}, NaN);
    s.final_velocity_norm = local_get_any(m, {'final_velocity_norm'}, NaN);
    s.max_abs_x_m = local_get_any(m, {'max_abs_x_m'}, NaN);
    s.max_abs_u_cmd_N = local_get_any(m, {'max_abs_u_cmd_N'}, NaN);
    s.max_abs_u_actual_N = local_get_any(m, {'max_abs_u_actual_N'}, NaN);
    s.saturation_fraction = local_get_any(m, {'saturation_fraction'}, NaN);
    s.failure_reason = string(local_get_any(m, {'fail_reason'}, 'none'));
end

function local_make_phase33_figures(figure_dir, S29, S30, S31, S32, summary_table)
    if ~exist(figure_dir, 'dir'), mkdir(figure_dir); end
    local_plot_summary_table(figure_dir, summary_table);
    local_plot_phase29_overview(figure_dir, S29);
    local_plot_phase30_hybrid(figure_dir, S30);
    local_plot_phase31_summary(figure_dir, S31);
    local_plot_phase32_compare(figure_dir, S32);
end

function local_plot_summary_table(figure_dir, T)
    fig = figure('Name', 'Phase 33 summary metrics', 'Visible', 'off');
    bar(categorical(cellstr(T.id)), double(T.pass));
    ylim([0 1.2]); grid on;
    ylabel('Pass flag');
    title('Phase 33 scenario pass overview');
    saveas(fig, fullfile(figure_dir, 'phase33_scenario_pass_overview.png'));
    close(fig);

    fig = figure('Name', 'Phase 33 final angle metrics', 'Visible', 'off');
    values = double(T.final_angle_error_rad);
    values(~isfinite(values)) = 0;
    bar(categorical(cellstr(T.id)), values);
    grid on; ylabel('Final/worst angle error (rad)');
    title('Final angle error by scenario');
    saveas(fig, fullfile(figure_dir, 'phase33_final_angle_error_by_scenario.png'));
    close(fig);
end

function local_plot_phase29_overview(figure_dir, S29)
    names = {'T1 open loop','T2 ideal','T3 actuator','T4 noise'};
    paths = {{'T1_open_loop'}, {'T2_ideal_tvlqr'}, {'T3_actuator_tvlqr'}, {'T4_noise_tvlqr'}};
    angle = zeros(1, 4); vel = zeros(1, 4); sat = zeros(1, 4);
    for i = 1:4
        m = local_get_nested(S29.metrics, paths{i}, struct());
        angle(i) = local_get_any(m, {'final_max_angle_error_rad','final_angle_error_rad'}, NaN);
        vel(i) = local_get_any(m, {'final_velocity_norm'}, NaN);
        sat(i) = local_get_any(m, {'saturation_fraction'}, NaN);
    end
    fig = figure('Name', 'Phase 29 tracking evaluation', 'Visible', 'off');
    subplot(3,1,1); bar(categorical(names), angle); grid on; ylabel('angle err (rad)'); title('Phase 29 tracking scenarios');
    subplot(3,1,2); bar(categorical(names), vel); grid on; ylabel('velocity norm');
    subplot(3,1,3); bar(categorical(names), sat); grid on; ylabel('sat fraction');
    saveas(fig, fullfile(figure_dir, 'phase33_phase29_tracking_metrics.png'));
    close(fig);
end

function local_plot_phase30_hybrid(figure_dir, S30)
    if ~isfield(S30, 'result'), return; end
    r = S30.result;
    fig = figure('Name', 'Phase 30 hybrid result', 'Visible', 'off');
    subplot(3,1,1); plot(r.t, r.state(:,1), 'LineWidth', 1.2); grid on; ylabel('x (m)'); title('Phase 30 full hybrid MATLAB');
    subplot(3,1,2); plot(r.t, arrayfun(@local_wrap_to_pi, r.state(:,3) - pi), 'LineWidth', 1.2); hold on; plot(r.t, arrayfun(@local_wrap_to_pi, r.state(:,5) - pi), 'LineWidth', 1.2); grid on; ylabel('angle err'); legend('theta1','theta2');
    subplot(3,1,3); plot(r.t, r.u_cmd, 'LineWidth', 1.2); hold on; plot(r.t, r.u_actual, 'LineWidth', 1.2); grid on; ylabel('force (N)'); xlabel('time (s)'); legend('u cmd','u actual');
    saveas(fig, fullfile(figure_dir, 'phase33_phase30_full_hybrid.png'));
    close(fig);
end

function local_plot_phase31_summary(figure_dir, S31)
    if ~isfield(S31, 'run_metrics'), return; end
    rm = S31.run_metrics;
    success = arrayfun(@(x) double(x.success), rm);
    angle = arrayfun(@(x) double(x.final_angle_error_rad), rm);
    vel = arrayfun(@(x) double(x.final_velocity_norm), rm);
    fig = figure('Name', 'Phase 31 Monte Carlo summary', 'Visible', 'off');
    subplot(3,1,1); stem(success, 'filled'); grid on; ylim([-0.1 1.1]); ylabel('success'); title('Phase 31 Monte Carlo by run');
    subplot(3,1,2); plot(angle, 'o-'); grid on; ylabel('angle err');
    subplot(3,1,3); plot(vel, 'o-'); grid on; ylabel('vel norm'); xlabel('run index');
    saveas(fig, fullfile(figure_dir, 'phase33_phase31_monte_carlo.png'));
    close(fig);
end

function local_plot_phase32_compare(figure_dir, S32)
    if ~isfield(S32, 'phase32_log'), return; end
    log = S32.phase32_log;
    fig = figure('Name', 'Phase 32 Simulink result', 'Visible', 'off');
    subplot(3,1,1); plot(log.t, log.state(:,1), 'LineWidth', 1.2); grid on; ylabel('x (m)'); title('Phase 32 Simulink logged result');
    subplot(3,1,2); plot(log.t, arrayfun(@local_wrap_to_pi, log.state(:,3) - pi), 'LineWidth', 1.2); hold on; plot(log.t, arrayfun(@local_wrap_to_pi, log.state(:,5) - pi), 'LineWidth', 1.2); grid on; ylabel('angle err'); legend('theta1','theta2');
    subplot(3,1,3); plot(log.t, log.u_cmd, 'LineWidth', 1.2); hold on; plot(log.t, log.u_actual, 'LineWidth', 1.2); grid on; ylabel('force (N)'); xlabel('time (s)'); legend('u cmd','u actual');
    saveas(fig, fullfile(figure_dir, 'phase33_phase32_simulink_result.png'));
    close(fig);
end

function local_write_phase33_status(path, phase33, summary, mat_file)
    fid = fopen(path, 'w');
    if fid < 0, error('evaluate_all_scenarios:CannotWriteStatus', 'Cannot write %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# PROJECT STATUS - PHASE 33\n\n');
    fprintf(fid, '## Phase name\n');
    fprintf(fid, 'Phase 33 - MATLAB/Simulink system evaluation and report-ready results.\n\n');
    fprintf(fid, '## Goal\n');
    fprintf(fid, 'Create a report-ready evaluation package from the passed MATLAB/Simulink pipeline. The evaluation includes Phase 29 TVLQR tracking with actuator delay and sensor noise, Phase 30 full hybrid handoff and balance, Phase 31 Monte Carlo robustness and Phase 32 Simulink equivalence.\n\n');
    fprintf(fid, '## Files added\n');
    fprintf(fid, '- `matlab/evaluation/evaluate_all_scenarios.m`\n');
    fprintf(fid, '- `matlab/evaluation/generate_result_summary.m`\n');
    fprintf(fid, '- `docs/MATLAB_SIMULINK_EVALUATION_REPORT.md`\n');
    fprintf(fid, '- `docs/PROJECT_STATUS_PHASE33.md`\n\n');
    fprintf(fid, '## Test command\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'run(''matlab/startup_project.m'')\n');
    fprintf(fid, 'run(''matlab/evaluation/evaluate_all_scenarios.m'')\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, '## Latest result\n');
    fprintf(fid, '- result file: `%s`\n', mat_file);
    fprintf(fid, '- scenario count: `%d`\n', summary.scenario_count);
    fprintf(fid, '- required_scenarios_present: `%d`\n', summary.required_scenarios_present);
    fprintf(fid, '- phase29_T4_pass: `%d`\n', summary.phase29_T4_pass);
    fprintf(fid, '- phase30_pass: `%d`\n', summary.phase30_pass);
    fprintf(fid, '- phase31_pass: `%d`\n', summary.phase31_pass);
    fprintf(fid, '- phase32_pass: `%d`\n', summary.phase32_pass);
    fprintf(fid, '- phase33_pass: `%d`\n', summary.phase33_pass);
    fprintf(fid, '- fail_reason: `%s`\n\n', summary.fail_reason);
    fprintf(fid, '## Output folders\n');
    fprintf(fid, '- figures: `%s`\n', fullfile(phase33.result_dir, 'figures'));
    fprintf(fid, '- logs: `%s`\n', fullfile(phase33.result_dir, 'logs'));
    fprintf(fid, '- report: `docs/MATLAB_SIMULINK_EVALUATION_REPORT.md`\n\n');
    fprintf(fid, '## Pass condition\n');
    fprintf(fid, 'Phase 33 passes when the evaluation contains at least five scenarios, includes Phase 29 T4 TVLQR with actuator delay and sensor noise, and preserves passed results from Phase 30, Phase 31 standard profile and Phase 32 Simulink equivalence.\n');
end

function v = local_get_nested(s, path, default_value)
    v = default_value;
    cur = s;
    for i = 1:numel(path)
        key = path{i};
        if isstruct(cur) && isfield(cur, key)
            cur = cur.(key);
        else
            return;
        end
    end
    v = cur;
end

function value = local_get_any(s, names, default_value)
    value = default_value;
    if ~isstruct(s), return; end
    for i = 1:numel(names)
        if isfield(s, names{i})
            value = s.(names{i});
            return;
        end
    end
end

function value = local_get_any_bool(s, names, default_value)
    value = logical(default_value);
    raw = local_get_any(s, names, default_value);
    if islogical(raw)
        value = raw;
    elseif isnumeric(raw)
        value = raw ~= 0;
    else
        value = logical(default_value);
    end
end

function y = local_wrap_to_pi(a)
    y = atan2(sin(a), cos(a));
end
