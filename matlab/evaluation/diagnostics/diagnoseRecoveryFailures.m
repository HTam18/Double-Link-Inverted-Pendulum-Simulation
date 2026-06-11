function report = diagnoseRecoveryFailures(result_dir)

    if nargin < 1 || isempty(result_dir)
        result_dir = fullfile(RecoveryUtils.projectRoot(), 'results', 'recovery');
    end
    all_csv = fullfile(result_dir, 'recoveryAllMetrics.csv');
    if ~isfile(all_csv)
        error('diagnoseRecoveryFailures:MissingCSV', 'Missing %s. Run Recovery matrix first.', all_csv);
    end

    T = readtable(all_csv, 'TextType', 'string');
    T.primary_fail = strings(height(T),1);
    for i = 1:height(T)
        T.primary_fail(i) = local_primary_reason(T.fail_reason(i));
    end

    by_force_reason = local_group_reason(T, {'case_type','force_N','primary_fail'});
    by_target_reason = local_group_reason(T, {'case_type','target','primary_fail'});
    by_link_reason = local_group_reason(T, {'case_type','link_id','primary_fail'});

    writetable(by_force_reason, fullfile(result_dir, 'failReasonByForce.csv'));
    writetable(by_target_reason, fullfile(result_dir, 'failReasonByTarget.csv'));
    writetable(by_link_reason, fullfile(result_dir, 'failReasonByLink.csv'));

    lines = {};
    lines{end+1} = 'RECOVERY FAILURE DIAGNOSTIC SUMMARY'; %#ok<AGROW>
    lines{end+1} = 'Primary fail reason is the first reason in fail_reason for each failed case.'; %#ok<AGROW>
    lines{end+1} = ' '; %#ok<AGROW>
    lines{end+1} = 'By force and reason:'; %#ok<AGROW>
    lines = local_append_table(lines, by_force_reason);
    lines{end+1} = ' '; %#ok<AGROW>
    lines{end+1} = 'By target and reason:'; %#ok<AGROW>
    lines = local_append_table(lines, by_target_reason);
    RecoveryIo.writeText(fullfile(result_dir, 'failureDiagnosticSummary.txt'), strjoin(lines, newline));

    report = struct();
    report.result_dir = result_dir;
    report.by_force_reason = by_force_reason;
    report.by_target_reason = by_target_reason;
    report.by_link_reason = by_link_reason;
    report.source = 'matlab/evaluation/diagnostics/diagnoseRecoveryFailures.m';
end

function R = local_group_reason(T, keys)
    [G, keytbl] = findgroups(T(:, keys));
    count = splitapply(@numel, T.success, G);
    success_rate = splitapply(@mean, double(T.success), G);
    sat = splitapply(@nanmean, T.saturation_fraction, G);
    R = keytbl;
    R.case_count = count;
    R.success_rate = success_rate;
    R.mean_saturation_fraction = sat;
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
