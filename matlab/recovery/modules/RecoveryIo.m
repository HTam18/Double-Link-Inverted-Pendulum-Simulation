classdef RecoveryIo
    methods (Static)
        function lines = appendMarkdownTable(lines, tbl)
            names = tbl.Properties.VariableNames;
            lines{end + 1} = ['| ', strjoin(names, ' | '), ' |']; %#ok<AGROW>
            lines{end + 1} = ['| ', strjoin(repmat({'---'}, 1, numel(names)), ' | '), ' |']; %#ok<AGROW>
            for i = 1:height(tbl)
                vals = cell(1, numel(names));
                for j = 1:numel(names)
                    v = tbl.(names{j})(i);
                    if iscell(v), v = v{1}; end
                    if isnumeric(v) || islogical(v)
                        vals{j} = sprintf('%.4g', v);
                    else
                        vals{j} = char(v);
                    end
                end
                lines{end + 1} = ['| ', strjoin(vals, ' | '), ' |']; %#ok<AGROW>
            end
            lines{end + 1} = ''; %#ok<AGROW>
        end

        function lines = appendTableLines(lines, tbl)
            names = tbl.Properties.VariableNames;
            for i = 1:height(tbl)
                parts = cell(1, numel(names));
                for j = 1:numel(names)
                    v = tbl.(names{j})(i);
                    if iscell(v), v = v{1}; end
                    if isnumeric(v) || islogical(v)
                        parts{j} = sprintf('%s=%g', names{j}, v);
                    else
                        parts{j} = sprintf('%s=%s', names{j}, char(v));
                    end
                end
                lines{end + 1} = strjoin(parts, ', '); %#ok<AGROW>
            end
        end

        function names = caseTableNames()
            names = {'case_index', 'case_type', 'target', 'event_name', 'link_id', 'contact_ratio', 'direction', 'force_N', 'duration_s', ...
                     'success', 'recovery_time_s', 'saturation_fraction', 'fail_reason', ...
                     'final_angle_error_rad', 'final_velocity_norm', 'final_cart_abs_m', ...
                     'max_angle_error_rad', 'max_velocity_norm', 'max_abs_x_m', 'max_abs_u_cmd_N', 'max_Q_external_norm', ...
                     'pass_finite', 'pass_force_applied', 'pass_recovered_final', 'pass_rail', 'pass_saturation', 'pass_recovery_time', ...
                     'x_after_force', 'xdot_after_force', 'theta1_after_force', 'theta1dot_after_force', 'theta2_after_force', 'theta2dot_after_force', ...
                     'planner_selected_variant', 'planner_selected_cost', 'recovery_roa_score', 'recovery_margin_score', 'primary_fail_reason', ...
                     'planner_try_opt', 'planner_opt_used_trajectory', 'planner_local_cost', 'planner_opt_cost', 'planner_decision_reason', ...
                     'planner_force_takeover', 'planner_tried_variants', 'planner_candidate_validation_vector', ...
                     'trajectory_file', 'animation_available'};
        end

        function out = forceCol(value)
            out = double(value(:));
        end

        function rows = loadCheckpointRows(path)
            rows = {};
            if ~isfile(path), return; end
            try
                T = readtable(path);
                expected = RecoveryIo.caseTableNames();
                missing = setdiff(expected, T.Properties.VariableNames);
                for i_missing = 1:numel(missing)
                    name = missing{i_missing};
                    if strcmp(name, 'trajectory_file')
                        T.(name) = repmat({''}, height(T), 1);
                    elseif strcmp(name, 'animation_available')
                        T.(name) = zeros(height(T), 1);
                    else
                        T.(name) = repmat({''}, height(T), 1);
                    end
                end
                if all(ismember(expected, T.Properties.VariableNames))
                    T = T(:, expected);
                    rows = table2cell(T);
                    fprintf('Resume checkpoint loaded: %s (%d rows)\n', path, height(T));
                end
            catch ME
                warning('runRecoveryMatrixCore:CheckpointLoadFailed', 'Could not load checkpoint %s: %s', path, ME.message);
                rows = {};
            end
        end

        function K = loadPreferredGain(params, cfg, target_name)
            root = RecoveryUtils.projectRoot();
            rec_path = fullfile(root, 'shared', 'lqr_recovery_gains_all_equilibria.mat');
            K = [];
            if isfile(rec_path)
                try
                    loaded = load(rec_path);
                    if isfield(loaded, 'gains') && isfield(loaded.gains, target_name)
                        K = double(loaded.gains.(target_name).K);
                    elseif isfield(loaded, 'report') && isfield(loaded.report, 'gains') && isfield(loaded.report.gains, target_name)
                        K = double(loaded.report.gains.(target_name).K);
                    end
                catch
                    K = [];
                end
            end
            if isempty(K)
                base = lqrStabilizeMultiTarget(0, targetEquilibriumLibrary(params).targets.(target_name).target_state(:), params, struct('target_mode', target_name));
                if isfield(base, 'K')
                    K = base.K;
                else
                    gpath = fullfile(root, 'shared', 'lqr_gains_all_equilibria.mat');
                    loaded = load(gpath);
                    if isfield(loaded, 'gains')
                        K = double(loaded.gains.(target_name).K);
                    else
                        K = double(loaded.report.gains.(target_name).K);
                    end
                end
            end
        end

        function token = safeFileToken(value)
            token = char(string(value));
            token = regexprep(token, '[^A-Za-z0-9_\-]+', '_');
        end

        function writeCheckpointRows(path, rows)
            if isempty(rows), return; end
            try
                T = cell2table(rows, 'VariableNames', RecoveryIo.caseTableNames());
                writetable(T, path);
            catch ME
                warning('runRecoveryMatrixCore:CheckpointWriteFailed', 'Could not write checkpoint %s: %s', path, ME.message);
            end
        end

        function writeFailedDisturbedStatesCsv(all_table, result_dir, cfg)
            out_csv = fullfile(result_dir, 'recovery_failed_disturbed_states.csv');
            priority_csv = fullfile(result_dir, 'recovery_failed_cases_for_trajectory_recovery.csv');
            state_cols = {'x_after_force','xdot_after_force','theta1_after_force','theta1dot_after_force','theta2_after_force','theta2dot_after_force'};
            if isempty(all_table) || height(all_table) == 0
                warning('runRecoveryMatrixCore:NoRowsForFailedStateExport', 'No Recovery rows available for failed-state export.');
                return;
            end
            if ~all(ismember(state_cols, all_table.Properties.VariableNames))
                error('runRecoveryMatrixCore:MissingAfterForceStateColumns', ...
                      'real-state export requires real after-force state columns before optimizing recovery trajectories.');
            end
            failed = all_table(~logical(all_table.success), :);
            if height(failed) == 0
                writetable(failed, out_csv);
                writetable(failed, priority_csv);
                fprintf('real-state export failed disturbed state export: no failed cases. Wrote empty CSV: %s\n', out_csv);
                return;
            end
            finite_state = true(height(failed), 1);
            nontrivial_state = false(height(failed), 1);
            for i = 1:numel(state_cols)
                v = double(failed.(state_cols{i}));
                finite_state = finite_state & isfinite(v);
                nontrivial_state = nontrivial_state | abs(v) > 1e-10;
            end
            failed = failed(finite_state & nontrivial_state, :);
            if height(failed) == 0
                error('runRecoveryMatrixCore:NoValidRealAfterForceStates', ...
                      'real-state export could not export valid real failed disturbed states. Check sim.states/time logging.');
            end

            failed.x0_source = repmat("REAL_AFTER_FORCE_STATE", height(failed), 1);
            failed.x0_extraction_time_s = failed.duration_s + RecoveryUtils.getDouble(cfg, 'recovery_after_force_state_offset_s', 0.02);
            target_str = string(failed.target);
            weak_target = double(target_str == "up_up" | target_str == "down_up");
            nominal_force = double(abs(double(failed.force_N) - 2) < 1e-9 | abs(double(failed.force_N) - 3) < 1e-9);
            gate_fail = double(contains(string(failed.fail_reason), 'FINAL_STATE_OUTSIDE_RECOVERY_GATE'));
            rail_fail = double(contains(string(failed.fail_reason), 'CART_RAIL_LIMIT_EXCEEDED'));
            sat = double(failed.saturation_fraction);
            sat(~isfinite(sat)) = 0;
            failed.priority_score = 100.0 * nominal_force + 20.0 * weak_target + 5.0 * gate_fail + 8.0 * rail_fail + sat;
            failed = sortrows(failed, {'priority_score','force_N','target'}, {'descend','ascend','ascend'});

            writetable(failed, out_csv);
            writetable(failed, priority_csv);
            fprintf('real-state export exported %d real failed disturbed states: %s\n', height(failed), out_csv);
        end

        function writeRecoveryGuiCaseCsv(result_dir, all_table)
            out_csv = fullfile(result_dir, 'recoveryGuiCases.csv');
            if isempty(all_table) || height(all_table) == 0
                writetable(all_table, out_csv);
                return;
            end
            required = {'case_index','case_type','target','event_name','link_id','contact_ratio','direction','force_N', ...
                        'success','recovery_time_s','saturation_fraction','fail_reason','final_angle_error_rad', ...
                        'final_velocity_norm','max_abs_x_m','max_abs_u_cmd_N','planner_selected_variant'};
            keep = required(ismember(required, all_table.Properties.VariableNames));
            gui_table = all_table(:, keep);
            if ismember('trajectory_file', all_table.Properties.VariableNames)
                gui_table.trajectory_file = all_table.trajectory_file;
            else
                gui_table.trajectory_file = repmat({''}, height(all_table), 1);
            end
            if ismember('animation_available', all_table.Properties.VariableNames)
                gui_table.animation_available = all_table.animation_available;
            else
                gui_table.animation_available = zeros(height(all_table), 1);
            end
            writetable(gui_table, out_csv);
        end

        function writeStatus(status_file, summary, cfg, by_target, by_force, by_link, by_post, by_planner)
            lines = {};
            lines{end + 1} = '# RECOVERY MATRIX STATUS'; %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Scope'; %#ok<AGROW>
            lines{end + 1} = sprintf('Recovery evaluates recovery after external horizontal force events from force event for force levels %s N. It does not implement a user interface.', mat2str(cfg.force_levels_N)); %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Controller under test'; %#ok<AGROW>
            lines{end + 1} = '- disturbanceRecoveryController.m now protects the LQR baseline LQR baseline and adds explicit recovery modes.';
            lines{end + 1} = sprintf('- Active force levels in this run: %s N.', mat2str(cfg.force_levels_N)); %#ok<AGROW>
            lines{end + 1} = '- External force remains separated from u_cmd/u_actual and is passed as external_generalized_force.'; %#ok<AGROW>
            lines{end + 1} = '- Failures are reported with fail_reason and are not hidden.'; %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Test matrix'; %#ok<AGROW>
            lines{end + 1} = sprintf('- Targets: %s', strjoin(cfg.target_names, ', ')); %#ok<AGROW>
            lines{end + 1} = sprintf('- Force levels: %s N', mat2str(cfg.force_levels_N)); %#ok<AGROW>
            lines{end + 1} = sprintf('- Contact ratios: %s', mat2str(cfg.contact_ratios)); %#ok<AGROW>
            lines{end + 1} = '- Link IDs: 1 and 2'; %#ok<AGROW>
            lines{end + 1} = '- Directions: right and left'; %#ok<AGROW>
            lines{end + 1} = '- Additional post-handoff test: after switching demo full switching final state.'; %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Summary'; %#ok<AGROW>
            lines{end + 1} = sprintf('- target_case_count: %d', summary.target_case_count); %#ok<AGROW>
            lines{end + 1} = sprintf('- post_switching_case_count: %d', summary.post_switching_case_count); %#ok<AGROW>
            lines{end + 1} = sprintf('- target_success_rate: %.3f', summary.target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- post_switching_success_rate: %.3f', summary.post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- overall_success_rate_1_2_3_4_5N: %.3f', summary.overall_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- nominal_target_success_rate_1_2_3N: %.3f', summary.nominal_target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- nominal_post_switching_success_rate_1_2_3N: %.3f', summary.nominal_post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- stress_target_success_rate_5N: %.3f', summary.stress_target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- stress_post_switching_success_rate_5N: %.3f', summary.stress_post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('- overall_pass_nominal_1_2_3N: %d', summary.overall_pass); %#ok<AGROW>
            lines{end + 1} = ['- stress_limit_note: ', summary.stress_limit_note]; %#ok<AGROW>
            lines{end + 1} = sprintf('- post_switching_initial_state_source: %s', summary.post_switching_initial_state_source); %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Grouped results'; %#ok<AGROW>
            lines{end + 1} = '### By target'; %#ok<AGROW>
            lines = RecoveryIo.appendMarkdownTable(lines, by_target);
            lines{end + 1} = '### By force'; %#ok<AGROW>
            lines = RecoveryIo.appendMarkdownTable(lines, by_force);
            lines{end + 1} = '### By link'; %#ok<AGROW>
            lines = RecoveryIo.appendMarkdownTable(lines, by_link);
            lines{end + 1} = '### After full switching by force'; %#ok<AGROW>
            lines = RecoveryIo.appendMarkdownTable(lines, by_post);
            lines{end + 1} = '### Planner selected variant usage'; %#ok<AGROW>
            lines = RecoveryIo.appendMarkdownTable(lines, by_planner);
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Interpretation rule'; %#ok<AGROW>
            lines{end + 1} = '- 1/2/3 N are treated as nominal recovery levels; 4 N is extended; 5 N is stress limit.'; %#ok<AGROW>
            lines{end + 1} = '- 5 N remains the model Nmax stress limit and is reported separately from nominal 1/2/3 N success rate.'; %#ok<AGROW>
            lines{end + 1} = ''; %#ok<AGROW>
            lines{end + 1} = '## Output files'; %#ok<AGROW>
            lines{end + 1} = ['- ', summary.case_csv]; %#ok<AGROW>
            lines{end + 1} = ['- ', summary.post_csv]; %#ok<AGROW>
            lines{end + 1} = ['- ', summary.all_csv]; %#ok<AGROW>
            RecoveryIo.writeText(status_file, strjoin(lines, newline));
        end

        function writeSummaryText(result_dir, summary, cfg, by_target, by_force, by_link, by_post, by_planner)
            lines = {};
            lines{end + 1} = 'RECOVERY MATRIX SUMMARY'; %#ok<AGROW>
            lines{end + 1} = 'Scope: recovery after external force events. User interaction is handled by the Python GUI.'; %#ok<AGROW>
            lines{end + 1} = 'Controller: constrained terminal recovery with rail-aware safety gates. Local LQR is kept only for easy states; the 16 hard 1N fail-state families try rail braking, angular damping, and terminal settle candidates before logging NO_TRAJ_CANDIDATE_PASSED.'; %#ok<AGROW>
            lines{end + 1} = 'State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]'; %#ok<AGROW>
            lines{end + 1} = 'Force mapping: Q_external = J_contact(q)^T * F_external_xy'; %#ok<AGROW>
            lines{end + 1} = sprintf('Nmax = %.3f N', summary.max_external_force_N); %#ok<AGROW>
            lines{end + 1} = sprintf('target_case_count = %d', summary.target_case_count); %#ok<AGROW>
            lines{end + 1} = sprintf('post_switching_case_count = %d', summary.post_switching_case_count); %#ok<AGROW>
            lines{end + 1} = sprintf('target_success_rate = %.6f', summary.target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('post_switching_success_rate = %.6f', summary.post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('overall_success_rate = %.6f', summary.overall_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('nominal_target_success_rate_1_2_3N = %.6f', summary.nominal_target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('nominal_post_switching_success_rate_1_2_3N = %.6f', summary.nominal_post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('stress_target_success_rate_5N = %.6f', summary.stress_target_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('stress_post_switching_success_rate_5N = %.6f', summary.stress_post_switching_success_rate); %#ok<AGROW>
            lines{end + 1} = sprintf('overall_pass_nominal_1_2_3N = %d', summary.overall_pass); %#ok<AGROW>
            lines{end + 1} = ['stress_limit_note = ', summary.stress_limit_note]; %#ok<AGROW>
            lines{end + 1} = sprintf('recovery_gate_angle_rad = %.3f', cfg.recovery_angle_rad); %#ok<AGROW>
            lines{end + 1} = sprintf('recovery_gate_velocity_norm = %.3f', cfg.recovery_velocity_norm); %#ok<AGROW>
            lines{end + 1} = sprintf('recovery_gate_cart_abs_m = %.3f', cfg.recovery_cart_abs_m); %#ok<AGROW>
            lines{end + 1} = sprintf('post_switching_initial_state_source = %s', summary.post_switching_initial_state_source); %#ok<AGROW>
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = 'Success rate by target:'; %#ok<AGROW>
            lines = RecoveryIo.appendTableLines(lines, by_target);
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = 'Success rate by force:'; %#ok<AGROW>
            lines = RecoveryIo.appendTableLines(lines, by_force);
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = 'Success rate by link:'; %#ok<AGROW>
            lines = RecoveryIo.appendTableLines(lines, by_link);
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = 'Post switching grouped by force:'; %#ok<AGROW>
            lines = RecoveryIo.appendTableLines(lines, by_post);
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = 'Planner selected variant usage:'; %#ok<AGROW>
            lines = RecoveryIo.appendTableLines(lines, by_planner);
            lines{end + 1} = ' '; %#ok<AGROW>
            lines{end + 1} = ['Planner usage CSV: ', summary.planner_csv]; %#ok<AGROW>
            lines{end + 1} = ['Case CSV: ', summary.case_csv]; %#ok<AGROW>
            lines{end + 1} = ['After switching CSV: ', summary.post_csv]; %#ok<AGROW>
            lines{end + 1} = ['All metrics CSV: ', summary.all_csv]; %#ok<AGROW>
            RecoveryIo.writeText(fullfile(result_dir, 'recoverySummary.txt'), strjoin(lines, newline));
        end

        function writeText(path, text)
            fid = fopen(path, 'w');
            if fid < 0
                error('runRecoveryMatrixCore:FileWriteError', 'Cannot write %s', path);
            end
            cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%s\n', text);
        end

    end
end
