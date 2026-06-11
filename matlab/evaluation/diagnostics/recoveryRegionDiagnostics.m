function report = recoveryRegionDiagnostics(result_dir)

    if nargin < 1 || isempty(result_dir)
        result_dir = fullfile(RecoveryUtils.projectRoot(), 'results', 'recovery');
    end
    all_csv = fullfile(result_dir, 'recoveryAllMetrics.csv');
    if ~isfile(all_csv)
        error('recoveryRegionDiagnostics:MissingCSV', 'Missing %s. Run Recovery matrix first.', all_csv);
    end

    T = readtable(all_csv, 'TextType', 'string');
    if ~ismember('primary_fail_reason', T.Properties.VariableNames)
        T.primary_fail_reason = strings(height(T),1);
        for i = 1:height(T)
            T.primary_fail_reason(i) = local_primary_reason(T.fail_reason(i));
        end
    end
    if ~ismember('planner_try_opt', T.Properties.VariableNames)
        T.planner_try_opt = false(height(T),1);
    end
    if ~ismember('planner_opt_used_trajectory', T.Properties.VariableNames)
        T.planner_opt_used_trajectory = false(height(T),1);
    end
    if ~ismember('recovery_roa_score', T.Properties.VariableNames)
        T.recovery_roa_score = NaN(height(T),1);
    end
    if ~ismember('recovery_margin_score', T.Properties.VariableNames)
        T.recovery_margin_score = NaN(height(T),1);
    end

    by_variant = local_group_router(T, {'case_type','planner_selected_variant'});
    by_target_force_variant = local_group_router(T, {'case_type','target','force_N','planner_selected_variant'});
    by_decision = local_group_router(T, {'case_type','planner_decision_reason'});
    by_fail = local_group_router(T, {'case_type','planner_selected_variant','primary_fail_reason'});

    writetable(by_variant, fullfile(result_dir, 'recovery_roa_router_by_variant.csv'));
    writetable(by_target_force_variant, fullfile(result_dir, 'recovery_roa_router_by_target_force_variant.csv'));
    writetable(by_decision, fullfile(result_dir, 'recovery_roa_router_by_decision.csv'));
    writetable(by_fail, fullfile(result_dir, 'recovery_roa_router_by_fail_reason.csv'));

    lines = {};
    lines{end+1} = 'RECOVERY REGION ROUTER DIAGNOSTIC SUMMARY'; %#ok<AGROW>
    lines{end+1} = 'Purpose: verify that opt_library is no longer blocked at 0/320 and that actual simulation decides candidate selection.'; %#ok<AGROW>
    lines{end+1} = ' '; %#ok<AGROW>
    lines{end+1} = 'By selected variant:'; %#ok<AGROW>
    lines = local_append_table(lines, by_variant);
    lines{end+1} = ' '; %#ok<AGROW>
    lines{end+1} = 'By planner decision:'; %#ok<AGROW>
    lines = local_append_table(lines, by_decision);
    RecoveryIo.writeText(fullfile(result_dir, 'recovery_roa_router_diagnostic_summary.txt'), strjoin(lines, newline));

    report = struct();
    report.result_dir = result_dir;
    report.by_variant = by_variant;
    report.by_target_force_variant = by_target_force_variant;
    report.by_decision = by_decision;
    report.by_fail = by_fail;
    report.source = 'matlab/evaluation/diagnostics/recoveryRegionDiagnostics.m';
end

function R = local_group_router(T, keys)
    keys = keys(ismember(keys, T.Properties.VariableNames));
    if isempty(keys)
        R = table();
        return;
    end
    [G, keytbl] = findgroups(T(:, keys));
    R = keytbl;
    R.case_count = splitapply(@numel, T.success, G);
    R.success_rate = splitapply(@mean, double(T.success), G);
    R.opt_try_rate = splitapply(@mean, double(T.planner_try_opt), G);
    R.opt_used_rate = splitapply(@mean, double(T.planner_opt_used_trajectory), G);
    R.mean_recovery_roa_score = splitapply(@nanmean, T.recovery_roa_score, G);
    R.mean_recovery_margin_score = splitapply(@nanmean, T.recovery_margin_score, G);
    R.mean_saturation_fraction = splitapply(@nanmean, T.saturation_fraction, G);
    R.mean_recovery_time_s = splitapply(@nanmean, T.recovery_time_s, G);
end

function r = local_primary_reason(reason)
    reason = string(reason);
    if reason == "PASS"
        r = "PASS";
        return;
    end
    parts = split(reason, '|');
    r = parts(1);
end

function v = nanmean(x)
    x = x(isfinite(x));
    if isempty(x), v = NaN; else, v = mean(x); end
end

function lines = local_append_table(lines, T)
    names = T.Properties.VariableNames;
    for i = 1:height(T)
        parts = cell(1, numel(names));
        for j = 1:numel(names)
            v = T.(names{j})(i);
            if isnumeric(v) || islogical(v)
                parts{j} = sprintf('%s=%g', names{j}, v);
            else
                parts{j} = sprintf('%s=%s', names{j}, char(v));
            end
        end
        lines{end+1} = strjoin(parts, ', '); %#ok<AGROW>
    end
end

function writeText(path, txt)
    fid = fopen(path, 'w');
    if fid < 0, error('Cannot write %s', path); end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s\n', txt);
end

function root = projectRoot()
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
