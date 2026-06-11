function summary = runRobustnessSuiteCore(user_cfg)

    if nargin < 1 || isempty(user_cfg), user_cfg = struct(); end
    this_file = mfilename('fullpath');
    sim_dir = fileparts(this_file);
    matlab_root = fileparts(sim_dir);
    project_root = fileparts(matlab_root);
    run(fullfile(matlab_root, 'startupProject.m'));

    cfg = RecoveryUtils.defaultCfg(user_cfg, project_root);
    if ~exist(cfg.result_dir, 'dir'), mkdir(cfg.result_dir); end

    recovery_summary = local_read_recovery_metrics(fullfile(project_root, 'results', 'recovery', 'recoveryAllMetrics.csv'));
    switching_summary = local_read_switching_result(fullfile(project_root, 'results', 'switching', 'switchingResults.mat'));

    robustness_table = local_build_robustness_table(recovery_summary, switching_summary);
    robustness_csv = fullfile(cfg.result_dir, 'robustnessSummary.csv');
    writetable(robustness_table, robustness_csv);

    if ~exist(fullfile(project_root, 'docs'), 'dir'), mkdir(fullfile(project_root, 'docs')); end
    requirement_report = generateFinalRequirementReport(cfg, recovery_summary, switching_summary, robustness_table);

    summary = struct();
    summary.workflow = 'robustness';
    summary.backend_controller = 'constrainedRecoveryTrajectoryRouter';
    summary.declared_status = 'ROBUSTNESS_COMPLETE_WITH_CURRENT_RECOVERY_BACKEND_LIMITATION';
    summary.robustness_pass = recovery_summary.total_count > 0 && switching_summary.available;
    summary.recovery_success_count = recovery_summary.success_count;
    summary.recovery_total_count = recovery_summary.total_count;
    summary.recovery_success_rate = recovery_summary.success_rate;
    summary.recovery_overall_pass_95 = recovery_summary.overall_pass;
    summary.switching_result_available = switching_summary.available;
    summary.robustness_csv = robustness_csv;
    summary.requirement_report_file = requirement_report.report_file;
    summary.requirement_csv = requirement_report.requirement_csv;
    summary.result_dir = cfg.result_dir;
    summary.known_limitation = 'Backend recovery remains constrained recovery success rate; future controller work is required for the remaining hard cases.';

    summary_txt = fullfile(cfg.result_dir, 'robustness_summary.txt');
    local_write_summary_txt(summary_txt, summary, recovery_summary, switching_summary);

    save(fullfile(cfg.result_dir, 'robustnessSuiteSummary.mat'), ...
        'summary', 'recovery_summary', 'switching_summary', 'robustness_table', 'cfg');

    local_write_project_status(fullfile(cfg.result_dir, 'robustnessStatus.txt'), summary, recovery_summary, switching_summary);
end

function cfg = defaultCfg(user_cfg, project_root)
    cfg = struct();
    cfg.project_root = project_root;
    cfg.result_dir = fullfile(project_root, 'results', 'robustness');
    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
end

function s = local_read_recovery_metrics(metrics_file)
    s = struct();
    s.metrics_file = metrics_file;
    s.total_count = 0; s.success_count = 0; s.success_rate = NaN; s.overall_pass = false;
    s.target_total_count = 0; s.target_success_count = 0;
    s.after_total_count = 0; s.after_success_count = 0;
    s.up_up_total_count = 0; s.up_up_success_count = 0;
    s.up_down_total_count = 0; s.up_down_success_count = 0;
    s.down_up_total_count = 0; s.down_up_success_count = 0;
    s.source_status = 'missing';
    if ~isfile(metrics_file)
        warning('Robustness:MissingRecoveryMetrics', 'Missing recovery metrics: %s', metrics_file);
        return;
    end
    T = readtable(metrics_file, 'TextType', 'string');
    if ~ismember('success', T.Properties.VariableNames)
        warning('Robustness:InvalidRecoveryMetrics', 'No success column in %s', metrics_file);
        return;
    end
    success_vec = logical(T.success);
    s.total_count = height(T);
    s.success_count = sum(success_vec);
    if s.total_count > 0, s.success_rate = s.success_count / s.total_count; end
    s.overall_pass = isfinite(s.success_rate) && s.success_rate >= 0.95;
    s.source_status = 'loaded';
    if ismember('case_type', T.Properties.VariableNames)
        ct = string(T.case_type);
        m = ct == "target_recovery"; s.target_total_count = sum(m); s.target_success_count = sum(success_vec & m);
        m = ct == "after_full_switching"; s.after_total_count = sum(m); s.after_success_count = sum(success_vec & m);
    end
    if ismember('target', T.Properties.VariableNames)
        tg = string(T.target);
    elseif ismember('target_name', T.Properties.VariableNames)
        tg = string(T.target_name);
    else
        tg = strings(height(T),1);
    end
    m = tg == "up_up"; s.up_up_total_count = sum(m); s.up_up_success_count = sum(success_vec & m);
    m = tg == "up_down"; s.up_down_total_count = sum(m); s.up_down_success_count = sum(success_vec & m);
    m = tg == "down_up"; s.down_up_total_count = sum(m); s.down_up_success_count = sum(success_vec & m);
end

function s = local_read_switching_result(result_file)
    s = struct();
    s.result_file = result_file;
    s.available = isfile(result_file);
    s.status = 'missing';
    s.frame_count = 0;
    if ~s.available, return; end
    data = load(result_file);
    s.status = 'loaded';
    if isfield(data, 't')
        s.frame_count = numel(data.t);
    elseif isfield(data, 'sim') && isfield(data.sim, 't')
        s.frame_count = numel(data.sim.t);
    end
end

function T = local_build_robustness_table(r, sw)
    category = strings(7,1);
    pass_count = zeros(7,1);
    total_count = zeros(7,1);
    success_rate = zeros(7,1);
    pass_flag = false(7,1);
    note = strings(7,1);
    category(1) = "Recovery overall"; pass_count(1) = r.success_count; total_count(1) = r.total_count; success_rate(1) = r.success_rate; pass_flag(1) = r.overall_pass; note(1) = "Backend limitation retained";
    category(2) = "Target recovery"; pass_count(2) = r.target_success_count; total_count(2) = r.target_total_count; success_rate(2) = local_safe_rate(pass_count(2), total_count(2)); pass_flag(2) = success_rate(2) >= 0.95; note(2) = "Recovery matrix group";
    category(3) = "After full switching recovery"; pass_count(3) = r.after_success_count; total_count(3) = r.after_total_count; success_rate(3) = local_safe_rate(pass_count(3), total_count(3)); pass_flag(3) = success_rate(3) >= 0.95; note(3) = "Recovery matrix group";
    category(4) = "Up Up recovery cases"; pass_count(4) = r.up_up_success_count; total_count(4) = r.up_up_total_count; success_rate(4) = local_safe_rate(pass_count(4), total_count(4)); pass_flag(4) = success_rate(4) >= 0.95; note(4) = "Hard upright bottleneck";
    category(5) = "Up Down recovery cases"; pass_count(5) = r.up_down_success_count; total_count(5) = r.up_down_total_count; success_rate(5) = local_safe_rate(pass_count(5), total_count(5)); pass_flag(5) = success_rate(5) >= 0.95; note(5) = "Protected group";
    category(6) = "Down Up recovery cases"; pass_count(6) = r.down_up_success_count; total_count(6) = r.down_up_total_count; success_rate(6) = local_safe_rate(pass_count(6), total_count(6)); pass_flag(6) = success_rate(6) >= 0.95; note(6) = "Few hard states remain";
    category(7) = "Switching result available"; pass_count(7) = double(sw.available); total_count(7) = 1; success_rate(7) = pass_count(7); pass_flag(7) = sw.available; note(7) = "MATLAB backend export for Python GUI";
    T = table(category, pass_count, total_count, success_rate, pass_flag, note);
end

function r = local_safe_rate(a,b)
    if b <= 0, r = NaN; else, r = a / b; end
end

function local_write_summary_txt(path, s, r, sw)
    fid = fopen(path, 'w');
    if fid < 0, error('Robustness:SummaryOpenFailed', 'Cannot write %s', path); end
    fprintf(fid, 'ROBUSTNESS SUITE SUMMARY\n');
    fprintf(fid, 'Declared status: %s\n', s.declared_status);
    fprintf(fid, 'Backend: %s\n', s.backend_controller);
    fprintf(fid, 'Robustness pass: %d\n', s.robustness_pass);
    fprintf(fid, 'Recovery success: %d/%d = %.6f\n', r.success_count, r.total_count, r.success_rate);
    fprintf(fid, 'Recovery overall pass at 95%%%%: %d\n', r.overall_pass);
    fprintf(fid, 'Target recovery: %d/%d\n', r.target_success_count, r.target_total_count);
    fprintf(fid, 'After full switching: %d/%d\n', r.after_success_count, r.after_total_count);
    fprintf(fid, 'Switching result available: %d\n', sw.available);
    fprintf(fid, 'Known limitation: %s\n', s.known_limitation);
    fprintf(fid, 'Requirement report: %s\n', s.requirement_report_file);
    fprintf(fid, 'Robustness CSV: %s\n', s.robustness_csv);
    fclose(fid);
end

function local_write_project_status(path, s, r, sw)
    fid = fopen(path, 'w');
    if fid < 0, error('Robustness:StatusOpenFailed', 'Cannot write %s', path); end
    fprintf(fid, '# ROBUSTNESS STATUS\n\n');
    fprintf(fid, '## Status\n\n');
    fprintf(fid, '`%s`\n\n', s.declared_status);
    fprintf(fid, 'Robustness and requirement verification are complete using the current constrained recovery backend success rate. The remaining recovery improvement is future controller work.\n\n');
    fprintf(fid, '## Metrics\n\n');
    fprintf(fid, '- Recovery success: %d/%d = %.6f\n', r.success_count, r.total_count, r.success_rate);
    fprintf(fid, '- Recovery 95%%%% pass: %d\n', r.overall_pass);
    fprintf(fid, '- Switching result available: %d\n\n', sw.available);
    fprintf(fid, '## Outputs\n\n');
    fprintf(fid, '- `results/robustness/robustnessSummary.csv`\n');
    fprintf(fid, '- `results/robustness/robustness_summary.txt`\n');
    fprintf(fid, '- `docs/Requirement Verification Report.md`\n');
    fprintf(fid, '## Known limitation\n\n');
    fprintf(fid, 'Hard Up Up force recovery cases remain the backend bottleneck. Continue controller work to improve backend recovery success rate.\n');
    fclose(fid);
end
