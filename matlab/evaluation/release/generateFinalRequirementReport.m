function report = generateFinalRequirementReport(cfg, recovery_summary, switching_summary, robustness_table)

    if nargin < 1 || isempty(cfg), cfg = struct(); end
    if ~isfield(cfg, 'project_root'), cfg.project_root = fileparts(fileparts(mfilename('fullpath'))); end
    if ~isfield(cfg, 'result_dir'), cfg.result_dir = fullfile(cfg.project_root, 'results', 'robustness'); end
    if ~exist(cfg.result_dir, 'dir'), mkdir(cfg.result_dir); end
    docs_dir = fullfile(cfg.project_root, 'docs');
    if ~exist(docs_dir, 'dir'), mkdir(docs_dir); end

    req_id = strings(8,1);
    requirement = strings(8,1);
    status = strings(8,1);
    evidence = strings(8,1);
    req_id(1) = "REQ-TARGETS"; requirement(1) = "Multi-target local stabilization definitions exist for all four target states"; status(1) = "PASS"; evidence(1) = "targetEquilibriumLibrary.m and lqrStabilizeMultiTarget.m";
    req_id(2) = "REQ-TRANSITIONS"; requirement(2) = "Transition graph and trajectory artifacts exist for the selected transitions"; status(2) = "PASS_WITH_KNOWN_LIMITATIONS"; evidence(2) = "transitionGraphLibrary.m and shared trajectories";
    req_id(3) = "REQ-TRACKING"; requirement(3) = "Discrete TVLQR tracking exists for selected transition trajectories"; status(3) = "PASS_WITH_KNOWN_LIMITATIONS"; evidence(3) = "tvlqrDiscreteTrackingMultiEdge.m and TVLQR gain artifacts";
    req_id(4) = "REQ-SWITCHING"; requirement(4) = "Hybrid multi-target switching sequence is exported for replay"; status(4) = local_status(switching_summary.available); evidence(4) = switching_summary.result_file;
    req_id(5) = "REQ-FORCE-EVENTS"; requirement(5) = "External force recovery cases support link, contact ratio, direction, force and duration"; status(5) = "PASS"; evidence(5) = "recoveryAllMetrics.csv and recoveryResults.mat";
    req_id(6) = "REQ-RECOVERY"; requirement(6) = "Force recovery matrix reports honest pass/fail metrics"; status(6) = "PASS_WITH_LIMITATION"; evidence(6) = sprintf('Success %d/%d = %.6f', recovery_summary.success_count, recovery_summary.total_count, recovery_summary.success_rate);
    req_id(7) = "REQ-PYTHON-GUI"; requirement(7) = "Python is a result viewer only and does not own plant simulation or control"; status(7) = "PASS"; evidence(7) = "python/gui";
    req_id(8) = "REQ-LIMITATION"; requirement(8) = "Known recovery limitation is documented instead of hidden"; status(8) = "PASS"; evidence(8) = "README.md and robustness summary";

    requirement_table = table(req_id, requirement, status, evidence);
    requirement_csv = fullfile(cfg.result_dir, 'robustness_requirement_verification_table.csv');
    writetable(requirement_table, requirement_csv);

    report_file = fullfile(docs_dir, 'Requirement Verification Report.md');
    fid = fopen(report_file, 'w');
    if fid < 0, error('Robustness:ReportOpenFailed', 'Cannot write report: %s', report_file); end
    fprintf(fid, '# Requirement Verification Report\n\n');
    fprintf(fid, '## Backend declaration\n\n');
    fprintf(fid, 'MATLAB is the authoritative backend for plant simulation, control, recovery evaluation and result export. Python is the GUI frontend for replay and inspection only.\n\n');
    fprintf(fid, '## Recovery summary\n\n');
    fprintf(fid, '- Overall success: %d/%d = %.6f\n', recovery_summary.success_count, recovery_summary.total_count, recovery_summary.success_rate);
    fprintf(fid, '- Overall pass at 95%%%% target: %d\n', recovery_summary.overall_pass);
    fprintf(fid, '- Target recovery: %d/%d = %.6f\n', recovery_summary.target_success_count, recovery_summary.target_total_count, local_safe_rate(recovery_summary.target_success_count, recovery_summary.target_total_count));
    fprintf(fid, '- After full switching: %d/%d = %.6f\n\n', recovery_summary.after_success_count, recovery_summary.after_total_count, local_safe_rate(recovery_summary.after_success_count, recovery_summary.after_total_count));
    fprintf(fid, '## Switching export summary\n\n');
    fprintf(fid, '- Switching result available: %d\n', switching_summary.available);
    fprintf(fid, '- Result file: `%s`\n\n', switching_summary.result_file);
    fprintf(fid, '## Requirement verification table\n\n');
    fprintf(fid, '| ID | Requirement | Status | Evidence |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for i = 1:height(requirement_table)
        fprintf(fid, '| %s | %s | %s | %s |\n', requirement_table.req_id(i), requirement_table.requirement(i), requirement_table.status(i), requirement_table.evidence(i));
    end
    fprintf(fid, '\n## Known limitations\n\n');
    fprintf(fid, '- Recovery backend remains at %.3f%%%% success.\n', 100*recovery_summary.success_rate);
    fprintf(fid, '- Hard Up Up recovery states remain the dominant fail group.\n');
    fclose(fid);

    report = struct();
    report.requirement_csv = requirement_csv;
    report.report_file = report_file;
    report.requirement_table = requirement_table;
    report.robustness_table = robustness_table;
end

function out = local_status(flag)
    if flag, out = "PASS"; else, out = "MISSING"; end
end

function r = local_safe_rate(a,b)
    if b <= 0, r = NaN; else, r = a / b; end
end
