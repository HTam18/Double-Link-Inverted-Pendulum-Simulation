function [summary_table, summary] = generate_result_summary(phase33, result_dir, project_root)
% GENERATE_RESULT_SUMMARY Build Phase 33 tables, text summary and report.
%
% [summary_table, summary] = generate_result_summary(phase33, result_dir, project_root)
%
% The input phase33.scenarios must be a struct array created by
% evaluate_all_scenarios.m. This function writes:
%   results/phase33_evaluation/phase33_summary_metrics.csv
%   results/phase33_evaluation/phase33_result_summary.txt
%   docs/MATLAB_SIMULINK_EVALUATION_REPORT.md

if nargin < 2 || isempty(result_dir)
    this_file = mfilename('fullpath');
    eval_dir = fileparts(this_file);
    matlab_root = fileparts(eval_dir);
    project_root = fileparts(matlab_root);
    result_dir = fullfile(project_root, 'results', 'phase33_evaluation');
end
if nargin < 3 || isempty(project_root)
    this_file = mfilename('fullpath');
    eval_dir = fileparts(this_file);
    matlab_root = fileparts(eval_dir);
    project_root = fileparts(matlab_root);
end
if ~exist(result_dir, 'dir'), mkdir(result_dir); end

scenarios = phase33.scenarios;
summary_table = local_scenarios_to_table(scenarios);

csv_file = fullfile(result_dir, 'phase33_summary_metrics.csv');
summary_file = fullfile(result_dir, 'phase33_result_summary.txt');
report_file = fullfile(project_root, 'docs', 'MATLAB_SIMULINK_EVALUATION_REPORT.md');
writetable(summary_table, csv_file);

summary = local_build_summary(summary_table);
local_write_text_summary(summary_file, summary_table, summary, phase33);
local_write_report(report_file, summary_table, summary, phase33);

fprintf('phase33_summary_csv = %s\n', csv_file);
fprintf('phase33_summary_text = %s\n', summary_file);
fprintf('phase33_report_file = %s\n', report_file);
end

function T = local_scenarios_to_table(scenarios)
    n = numel(scenarios);
    id = strings(n,1);
    name = strings(n,1);
    category = strings(n,1);
    pass = false(n,1);
    success_rate = nan(n,1);
    handoff_success = nan(n,1);
    handoff_success_rate = nan(n,1);
    stabilize_success = nan(n,1);
    final_angle_error_rad = nan(n,1);
    final_velocity_norm = nan(n,1);
    settling_time_s = nan(n,1);
    max_abs_x_m = nan(n,1);
    max_abs_u_cmd_N = nan(n,1);
    max_abs_u_actual_N = nan(n,1);
    saturation_fraction = nan(n,1);
    failure_reason = strings(n,1);
    notes = strings(n,1);
    for i = 1:n
        id(i) = scenarios(i).id;
        name(i) = scenarios(i).name;
        category(i) = scenarios(i).category;
        pass(i) = logical(scenarios(i).pass);
        success_rate(i) = double(scenarios(i).success_rate);
        handoff_success(i) = double(scenarios(i).handoff_success);
        handoff_success_rate(i) = double(scenarios(i).handoff_success_rate);
        stabilize_success(i) = double(scenarios(i).stabilize_success);
        final_angle_error_rad(i) = double(scenarios(i).final_angle_error_rad);
        final_velocity_norm(i) = double(scenarios(i).final_velocity_norm);
        settling_time_s(i) = double(scenarios(i).settling_time_s);
        max_abs_x_m(i) = double(scenarios(i).max_abs_x_m);
        max_abs_u_cmd_N(i) = double(scenarios(i).max_abs_u_cmd_N);
        max_abs_u_actual_N(i) = double(scenarios(i).max_abs_u_actual_N);
        saturation_fraction(i) = double(scenarios(i).saturation_fraction);
        failure_reason(i) = scenarios(i).failure_reason;
        notes(i) = scenarios(i).notes;
    end
    T = table(id, name, category, pass, success_rate, handoff_success, handoff_success_rate, ...
        stabilize_success, final_angle_error_rad, final_velocity_norm, settling_time_s, ...
        max_abs_x_m, max_abs_u_cmd_N, max_abs_u_actual_N, saturation_fraction, failure_reason, notes);
end

function summary = local_build_summary(T)
    summary = struct();
    summary.scenario_count = height(T);
    summary.pass_count = sum(T.pass);
    summary.required_scenarios_present = height(T) >= 5 && any(T.id == "S4") && any(T.id == "S5") && any(T.id == "S6") && any(T.id == "S7");
    idx_s4 = find(T.id == "S4", 1);
    idx_s5 = find(T.id == "S5", 1);
    idx_s6 = find(T.id == "S6", 1);
    idx_s7 = find(T.id == "S7", 1);
    summary.phase29_T4_pass = ~isempty(idx_s4) && logical(T.pass(idx_s4));
    summary.phase30_pass = ~isempty(idx_s5) && logical(T.pass(idx_s5));
    summary.phase31_pass = ~isempty(idx_s6) && logical(T.pass(idx_s6));
    summary.phase32_pass = ~isempty(idx_s7) && logical(T.pass(idx_s7));
    summary.phase33_pass = summary.required_scenarios_present && summary.phase29_T4_pass && summary.phase30_pass && summary.phase31_pass && summary.phase32_pass;
    summary.fail_reason = local_fail_reason(summary);
end

function reason = local_fail_reason(s)
    parts = {};
    if ~s.required_scenarios_present, parts{end+1} = 'missing_required_scenarios'; end %#ok<AGROW>
    if ~s.phase29_T4_pass, parts{end+1} = 'phase29_T4_not_pass'; end %#ok<AGROW>
    if ~s.phase30_pass, parts{end+1} = 'phase30_not_pass'; end %#ok<AGROW>
    if ~s.phase31_pass, parts{end+1} = 'phase31_not_pass'; end %#ok<AGROW>
    if ~s.phase32_pass, parts{end+1} = 'phase32_not_pass'; end %#ok<AGROW>
    if isempty(parts)
        reason = 'none';
    else
        reason = strjoin(parts, ',');
    end
end

function local_write_text_summary(path, T, summary, phase33)
    fid = fopen(path, 'w');
    if fid < 0, error('generate_result_summary:CannotWriteSummary', 'Cannot write %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'Phase 33 MATLAB/Simulink evaluation summary\n');
    fprintf(fid, 'created_at = %s\n', phase33.created_at);
    fprintf(fid, 'scenario_count = %d\n', summary.scenario_count);
    fprintf(fid, 'pass_count = %d\n', summary.pass_count);
    fprintf(fid, 'phase29_T4_pass = %d\n', summary.phase29_T4_pass);
    fprintf(fid, 'phase30_pass = %d\n', summary.phase30_pass);
    fprintf(fid, 'phase31_pass = %d\n', summary.phase31_pass);
    fprintf(fid, 'phase32_pass = %d\n', summary.phase32_pass);
    fprintf(fid, 'phase33_pass = %d\n', summary.phase33_pass);
    fprintf(fid, 'fail_reason = %s\n\n', summary.fail_reason);
    for i = 1:height(T)
        fprintf(fid, '[%s] %s | pass=%d | angle=%.6g | vel=%.6g | max_x=%.6g | sat=%.6g | reason=%s\n', ...
            char(T.id(i)), char(T.name(i)), T.pass(i), T.final_angle_error_rad(i), T.final_velocity_norm(i), ...
            T.max_abs_x_m(i), T.saturation_fraction(i), char(T.failure_reason(i)));
    end
end

function local_write_report(path, T, summary, phase33)
    fid = fopen(path, 'w');
    if fid < 0, error('generate_result_summary:CannotWriteReport', 'Cannot write %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# MATLAB/Simulink Evaluation Report - Phase 33\n\n');
    fprintf(fid, '## 1. Purpose\n\n');
    fprintf(fid, 'This report summarizes the evaluation of the Double Link Pendulum on Cart MATLAB/Simulink pipeline. The current validated chain is Phase 29 discrete TVLQR tracking, Phase 30 hybrid handoff and balance, Phase 31 Monte Carlo robustness and Phase 32 Simulink equivalence.\n\n');
    fprintf(fid, '## 2. State convention\n\n');
    fprintf(fid, '- State order: `[x, x_dot, theta1, theta1_dot, theta2, theta2_dot]`.\n');
    fprintf(fid, '- Angle convention: `theta = 0` is downward and `theta = pi` is upright.\n');
    fprintf(fid, '- Target used in the main evaluation: `up_up`.\n\n');
    fprintf(fid, '## 3. Scenario summary\n\n');
    fprintf(fid, '| ID | Scenario | Pass | Final angle error rad | Final velocity norm | Max abs x m | Max abs u cmd N | Max abs u actual N | Saturation fraction | Notes |\n');
    fprintf(fid, '|---|---|---:|---:|---:|---:|---:|---:|---:|---|\n');
    for i = 1:height(T)
        fprintf(fid, '| %s | %s | %d | %.6g | %.6g | %.6g | %.6g | %.6g | %.6g | %s |\n', ...
            char(T.id(i)), char(T.name(i)), T.pass(i), T.final_angle_error_rad(i), T.final_velocity_norm(i), ...
            T.max_abs_x_m(i), T.max_abs_u_cmd_N(i), T.max_abs_u_actual_N(i), T.saturation_fraction(i), char(T.notes(i)));
    end
    fprintf(fid, '\n## 4. Key observations\n\n');
    fprintf(fid, '1. The open-loop feedforward case is kept as a baseline. It is not the final validation gate because Phase 29 specifically adds feedback tracking to improve over open-loop replay.\n');
    fprintf(fid, '2. Phase 29 T4 is the most important tracking result for this report because it includes both actuator delay and sensor noise. This directly addresses non-ideal tracking conditions before the hybrid handoff stage.\n');
    fprintf(fid, '3. Phase 30 validates the complete MATLAB pipeline from trajectory tracking to handoff and short upright balance.\n');
    fprintf(fid, '4. Phase 31 evaluates robustness. The standard Monte Carlo profile is used as the pass profile, while stronger stress tests may be used to describe known limitations.\n');
    fprintf(fid, '5. Phase 32 validates the main Simulink model by comparing it with the MATLAB Phase 30 reference. The state trajectory match is the primary equivalence metric.\n\n');
    fprintf(fid, '## 5. Overall result\n\n');
    fprintf(fid, '- Required scenarios present: `%d`.\n', summary.required_scenarios_present);
    fprintf(fid, '- Phase 29 T4 pass: `%d`.\n', summary.phase29_T4_pass);
    fprintf(fid, '- Phase 30 pass: `%d`.\n', summary.phase30_pass);
    fprintf(fid, '- Phase 31 pass: `%d`.\n', summary.phase31_pass);
    fprintf(fid, '- Phase 32 pass: `%d`.\n', summary.phase32_pass);
    fprintf(fid, '- Phase 33 pass: `%d`.\n', summary.phase33_pass);
    fprintf(fid, '- Fail reason: `%s`.\n\n', summary.fail_reason);
    fprintf(fid, '## 6. Result files\n\n');
    fprintf(fid, '- Phase 29 result: `%s`\n', phase33.phase29_file);
    fprintf(fid, '- Phase 30 result: `%s`\n', phase33.phase30_file);
    fprintf(fid, '- Phase 31 result: `%s`\n', phase33.phase31_file);
    fprintf(fid, '- Phase 32 result: `%s`\n', phase33.phase32_file);
    fprintf(fid, '- Phase 33 figures: `%s`\n\n', fullfile(phase33.result_dir, 'figures'));
    fprintf(fid, '## 7. Known limitations\n\n');
    fprintf(fid, 'The controller has passed the standard robustness profile, but earlier stress tests showed sensitivity to large parameter mismatch and strong disturbance. Future improvement should focus on a wider terminal stabilizer region, online trajectory correction or robust/MPC-style recovery after handoff.\n');
end
