classdef RecoveryEngine
    methods (Static)
        function xaf = afterForceState(sim, event)
            t_after = double(event.start_time_s) + double(event.duration_s) + 0.02;
            idx = find(sim.time_s >= t_after, 1, 'first');
            if isempty(idx), idx = find(sim.time_s >= double(event.start_time_s), 1, 'first'); end
            if isempty(idx), idx = 1; end
            xaf = double(sim.states(idx,:).');
        end

        function ctrl = applyRecoveryRuntimeConstraints(ctrl, x, params, cfg)
            if ~RecoveryUtils.getLogical(cfg, 'recovery_enable_rail_safety_filter', false)
                return;
            end
            x = double(x(:));
            rail = RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.95);
            priority = RecoveryUtils.getDouble(cfg, 'recovery_cart_priority_abs_m', 1.40);
            emergency = RecoveryUtils.getDouble(cfg, 'recovery_rail_emergency_abs_m', 1.82);
            if isfield(cfg, 'recovery_mode') && RecoveryUtils.recoveryIsHardOrUpupMode(cfg)
                priority = min(priority, RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_abs_m', 1.72));
                emergency = min(emergency, RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_emergency_abs_m', 1.88));
            end
            k = RecoveryUtils.getDouble(cfg, 'recovery_rail_guard_kp', 10.0);
            kd = RecoveryUtils.getDouble(cfg, 'recovery_rail_guard_kd', 4.0);
            kem = RecoveryUtils.getDouble(cfg, 'recovery_rail_emergency_kp', 18.0);
            if isfield(cfg, 'recovery_mode') && RecoveryUtils.recoveryIsHardOrUpupMode(cfg)
                k = max(k, RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_kp', 26.0));
                kd = max(kd, RecoveryUtils.getDouble(cfg, 'recovery_hard_catch_barrier_kd', 10.5));
                kem = max(kem, 28.0);
            end
            xcart = x(1); vcart = x(2);
            u_guard = 0.0;
            if abs(xcart) > priority
                excess = abs(xcart) - priority;
                u_guard = u_guard - sign(xcart) * k * excess - kd * vcart;
            end
            if abs(xcart) > emergency && sign(xcart) * vcart > 0
                u_guard = u_guard - sign(xcart) * kem * (abs(xcart) - emergency + 0.15 * abs(vcart));
            end
            if abs(xcart) > rail
                u_guard = u_guard - sign(xcart) * kem * (abs(xcart) - rail + 0.30 * abs(vcart));
            end
            u_raw = double(ctrl.u_raw) + u_guard;
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs([u_min, u_max]));
            if isfield(cfg, 'recovery_mode') && RecoveryUtils.recoveryIsHardOrUpupMode(cfg)
                sx = sign(xcart); if sx == 0, sx = 1; end
                outward = sx * vcart > 0;
                if abs(xcart) > emergency && outward
                    u_raw = -sx * min(0.98*umax, 0.62*umax + 5.0*abs(vcart) + 14.0*max(0, abs(xcart)-emergency));
                elseif abs(xcart) >= rail - 0.02 && outward
                    u_raw = -sx * 0.98 * umax;
                end
            end
            u_cmd = min(max(u_raw, u_min), u_max);
            if abs(u_guard) > 1e-10
                ctrl.u_raw = u_raw;
                ctrl.u_cmd = u_cmd;
                if isfield(ctrl, 'recovery_state') && ~contains(char(ctrl.recovery_state), 'rail_guard')
                    ctrl.recovery_state = [char(ctrl.recovery_state), '_rail_guard'];
                elseif ~isfield(ctrl, 'recovery_state')
                    ctrl.recovery_state = 'recovery_rail_guard';
                end
                ctrl.mode = ctrl.recovery_state;
                ctrl.rail_guard_u = u_guard;
            end
        end

        function [q_ext, fx, fy, active] = externalAtTime(x, t, event, params)
            q_ext = zeros(3, 1);
            fx = 0.0;
            fy = 0.0;
            active = false;
            if isempty(event)
                return;
            end
            active = (t >= double(event.start_time_s)) && (t <= double(event.start_time_s) + double(event.duration_s));
            if active
                out = dipExternalForceGeneralized(x, event, params);
                q_ext = out.Q_external;
                fx = out.force_xy_N(1);
                fy = out.force_xy_N(2);
            end
        end

        function q_ext = externalAtTimeValue(x, t, event, params)
            [q_ext, ~, ~, ~] = RecoveryEngine.externalAtTime(x, t, event, params);
        end

        function out_table = filterForceSet(in_table, force_levels)
            keep = false(height(in_table), 1);
            for i = 1:numel(force_levels)
                keep = keep | abs(in_table.force_N - double(force_levels(i))) < 1e-12;
            end
            out_table = in_table(keep, :);
        end

        function post_events = filterPostSwitchingEvents(events, cfg)
            keep = false(numel(events), 1);
            for i = 1:numel(events)
                keep(i) = any(abs(double(events(i).force_N) - double(cfg.post_switching_force_levels_N)) < 1e-12);
            end
            post_events = events(keep);
        end

        function [post_state, post_target, source] = getSwitchingFinalState(project_root, params, cfg)
            post_target = 'up_up';
            target_library = targetEquilibriumLibrary(params);
            switching_mat = fullfile(project_root, 'results', 'switching', 'switchingResults.mat');
            if isfile(switching_mat)
                try
                    loaded = load(switching_mat);
                    if isfield(loaded, 'command_results') && ~isempty(loaded.command_results)
                        cr = loaded.command_results(end);
                        if isfield(cr, 'end_state') && ~isempty(cr.end_state)
                            post_state = double(cr.end_state(:));
                            if isfield(cr, 'end_target') && ~isempty(cr.end_target)
                                post_target = char(cr.end_target);
                            end
                            source = 'loaded_existing_switching_command_results_end_state';
                            return;
                        end
                    end
                    if isfield(loaded, 'summary') && isfield(loaded.summary, 'command_results') && ~isempty(loaded.summary.command_results)
                        cr = loaded.summary.command_results(end);
                        post_state = double(cr.end_state(:));
                        post_target = char(cr.end_target);
                        source = 'loaded_existing_switching_summary_command_results_end_state';
                        return;
                    end
                catch ME
                    warning('runRecoveryMatrixCore:SwitchingLoadFailed', 'Could not load switching demo MAT: %s', ME.message);
                end
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_require_existing_switching_result', false)
                error('Recovery replay requires existing switching result at %s. Keep/copy results/switching before running this mode.', switching_mat);
            end

            try
                fprintf('switching demo final state not found; running runSwitchingDemo() first.\n');
                s40 = runSwitchingDemo();
                cr = s40.command_results(end);
                post_state = double(cr.end_state(:));
                post_target = char(cr.end_target);
                source = 'generated_by_runSwitchingDemo';
                return;
            catch ME
                warning('runRecoveryMatrixCore:SwitchingRunFailed', 'Could not run switching demo automatically: %s', ME.message);
            end

            target_library = targetEquilibriumLibrary(params);
            post_state = target_library.targets.(post_target).target_state(:) + cfg.initial_error(:);
            source = 'fallback_up_up_target_state_with_small_error';
        end

        function tf = hasCompletedCase(rows, case_type, target_name, event)
            tf = false;
            if isempty(rows), return; end
            names = RecoveryIo.caseTableNames();
            idx_type = find(strcmp(names, 'case_type'), 1);
            idx_target = find(strcmp(names, 'target'), 1);
            idx_event = find(strcmp(names, 'event_name'), 1);
            idx_force = find(strcmp(names, 'force_N'), 1);
            idx_link = find(strcmp(names, 'link_id'), 1);
            idx_ratio = find(strcmp(names, 'contact_ratio'), 1);
            idx_dir = find(strcmp(names, 'direction'), 1);
            for r = 1:size(rows, 1)
                try
                    if strcmp(char(string(rows{r, idx_type})), char(case_type)) && ...
                       strcmp(char(string(rows{r, idx_target})), char(target_name)) && ...
                       strcmp(char(string(rows{r, idx_event})), char(event.name)) && ...
                       abs(double(rows{r, idx_force}) - double(event.force_N)) < 1e-9 && ...
                       abs(double(rows{r, idx_link}) - double(event.link_id)) < 1e-9 && ...
                       abs(double(rows{r, idx_ratio}) - double(event.contact_ratio)) < 1e-9 && ...
                       strcmp(char(string(rows{r, idx_dir})), char(event.direction_name))
                        tf = true; return;
                    end
                catch
                end
            end
        end

        function events = makeRecoveryEvents(cfg, params)
            max_force = double(params.external_force.max_external_force_N);
            template = struct('name', '', 'link_id', 0, 'contact_ratio', 0.0, 'force_N', 0.0, ...
                              'direction_name', '', 'direction_rad', 0.0, 'start_time_s', 0.0, ...
                              'duration_s', 0.0, 'end_time_s', 0.0, 'workflow', 'Recovery');
            n_events = numel(cfg.link_ids) * numel(cfg.contact_ratios) * numel(cfg.direction_names) * numel(cfg.force_levels_N);
            events = repmat(template, n_events, 1);
            idx = 0;
            for i_link = 1:numel(cfg.link_ids)
                for i_ratio = 1:numel(cfg.contact_ratios)
                    for i_dir = 1:numel(cfg.direction_names)
                        for i_force = 1:numel(cfg.force_levels_N)
                            idx = idx + 1;
                            e = template;
                            f = min(double(cfg.force_levels_N(i_force)), max_force);
                            e.name = sprintf('link%d_ratio%.2f_%s_%.0fN', cfg.link_ids(i_link), cfg.contact_ratios(i_ratio), cfg.direction_names{i_dir}, f);
                            e.link_id = cfg.link_ids(i_link);
                            e.contact_ratio = cfg.contact_ratios(i_ratio);
                            e.force_N = f;
                            e.direction_name = cfg.direction_names{i_dir};
                            e.direction_rad = cfg.direction_rad(i_dir);
                            e.start_time_s = cfg.force_start_time_s;
                            e.duration_s = cfg.force_duration_s;
                            e.end_time_s = e.start_time_s + e.duration_s;
                            e.workflow = 'Recovery';
                            events(idx) = e;
                        end
                    end
                end
            end
        end

        function tf = recoveryCaseFilterAllows(case_type, target_name, event, cfg)
            tf = true;
            if RecoveryUtils.getLogical(cfg, 'recovery_autotune_hard_states_only', false)
                tf = strcmp(char(target_name), 'up_up') && abs(double(event.force_N) - 1.0) < 1e-9 && ...
                     ((double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12) || ...
                      (double(event.link_id) == 2 && double(event.contact_ratio) >= 0.75 - 1e-12));
                return;
            end
            if isfield(cfg, 'recovery_case_filter_targets') && ~isempty(cfg.recovery_case_filter_targets)
                targets = RecoveryUtils.getCell(cfg, 'recovery_case_filter_targets', {});
                if ~any(strcmp(char(target_name), targets)), tf = false; return; end
            end
            if isfield(cfg, 'recovery_case_filter_case_types') && ~isempty(cfg.recovery_case_filter_case_types)
                cts = RecoveryUtils.getCell(cfg, 'recovery_case_filter_case_types', {});
                if ~any(strcmp(char(case_type), cts)), tf = false; return; end
            end
            if isfield(cfg, 'recovery_case_filter_link_ids') && ~isempty(cfg.recovery_case_filter_link_ids)
                if ~any(abs(double(event.link_id) - double(cfg.recovery_case_filter_link_ids)) < 1e-12), tf = false; return; end
            end
            if isfield(cfg, 'recovery_case_filter_contact_ratios') && ~isempty(cfg.recovery_case_filter_contact_ratios)
                if ~any(abs(double(event.contact_ratio) - double(cfg.recovery_case_filter_contact_ratios)) < 1e-12), tf = false; return; end
            end
        end

        function summary = runRecoveryMatrixEngine(user_cfg)

            if nargin < 1 || isempty(user_cfg)
                user_cfg = struct();
            end

            project_root = RecoveryUtils.projectRoot();
            addpath(genpath(fullfile(project_root, 'matlab')));

            params = loadParams();
            params = RecoveryUtils.recoveryParams(params);
            cfg = RecoveryUtils.defaultCfg(user_cfg, project_root);

            result_dir = fullfile(project_root, 'results', 'recovery');
            if ~exist(result_dir, 'dir'), mkdir(result_dir); end

            rec_gain_path = fullfile(project_root, 'shared', 'lqr_recovery_gains_all_equilibria.mat');
            if ~isfile(rec_gain_path)
                try
                    designLqrRecoveryAllEquilibria(params);
                catch ME
                    warning('runRecoveryMatrixCore:RecoveryGainDesignFailed', ...
                            'Could not design recovery LQR gains automatically: %s', ME.message);
                end
            end

            opt_lib_path = fullfile(project_root, 'shared', 'recovery_trajectories_opt', 'recovery_recovery_opt_library.mat');
            if RecoveryUtils.getLogical(cfg, 'recovery_enable_optimized_library', true) && RecoveryUtils.getLogical(cfg, 'recovery_auto_build_optimized_library', true) && ~isfile(opt_lib_path)
                try
                    opt_cfg = struct();
                    opt_cfg.max_representatives = RecoveryUtils.getDouble(cfg, 'recovery_opt_max_representatives', 18);
                    opt_cfg.max_iter = RecoveryUtils.getDouble(cfg, 'recovery_opt_max_iter', 60);
                    opt_cfg.control_knot_count = RecoveryUtils.getDouble(cfg, 'recovery_opt_control_knot_count', 13);
                    opt_cfg.optimizer_dt_s = RecoveryUtils.getDouble(cfg, 'recovery_opt_dt_s', 0.025);
                    opt_cfg.candidate_durations_s = RecoveryUtils.getVector(cfg, 'recovery_opt_candidate_durations_s', [2.5 3.5 5.0 6.5]);
                    run_recovery_optimize_recovery_failed_cases(opt_cfg);
                catch ME
                    warning('runRecoveryMatrixCore:OptLibraryBuildFailed', ...
                            'Could not build optimized recovery trajectory library automatically: %s', ME.message);
                end
            end

            status_file = fullfile(result_dir, 'recovery_constrainedRecovery_status.txt');
            if ~exist(fileparts(status_file), 'dir'), mkdir(fileparts(status_file)); end

            fprintf('==============================================\n');
            if isfield(cfg, 'recovery_mode') && strcmpi(char(string(cfg.recovery_mode)), 'BALANCE_WITHIN_2M_RAIL')
                fprintf('Recovery BALANCE_WITHIN_2M_RAIL force recovery matrix\n');
            else
                fprintf('Recovery real-state export REAL FAILED STATE EXTRACTION force recovery matrix\n');
            end
            fprintf('==============================================\n');
            fprintf('External force Nmax = %.3f N\n', params.external_force.max_external_force_N);
            fprintf('Targets: up_up, up_down, down_up\n');
            fprintf('Matrix: link 1/2, contact ratio 0.25/0.50/0.75/1.00, left/right, force levels %s N\n', mat2str(cfg.force_levels_N));
            fprintf('Also tests force recovery after switching demo full switching sequence with an additional settle period before applying force.\n\n');

            target_library = targetEquilibriumLibrary(params);
            events = RecoveryEngine.makeRecoveryEvents(cfg, params);
            target_names = cfg.target_names;

            checkpoint_target_csv = fullfile(result_dir, 'recovery_matrix_checkpoint_target_recovery.csv');
            checkpoint_post_csv = fullfile(result_dir, 'recovery_matrix_checkpoint_after_switching.csv');
            if RecoveryUtils.getLogical(cfg, 'recovery_reset_matrix_checkpoint', false)
                if isfile(checkpoint_target_csv), delete(checkpoint_target_csv); end
                if isfile(checkpoint_post_csv), delete(checkpoint_post_csv); end
                if RecoveryUtils.getLogical(cfg, 'recovery_clear_failed_state_csv_on_reset', true)
                    stale_failed_csv = fullfile(result_dir, 'recovery_failed_disturbed_states.csv');
                    stale_priority_csv = fullfile(result_dir, 'recovery_failed_cases_for_trajectory_recovery.csv');
                    if isfile(stale_failed_csv), delete(stale_failed_csv); end
                    if isfile(stale_priority_csv), delete(stale_priority_csv); end
                end
            end
            if RecoveryUtils.getLogical(cfg, 'recovery_resume_matrix', true)
                rows = RecoveryIo.loadCheckpointRows(checkpoint_target_csv);
                post_rows = RecoveryIo.loadCheckpointRows(checkpoint_post_csv);
            else
                rows = {};
                post_rows = {};
            end
            case_index = size(rows, 1);
            representative_sims = struct();

            for i_target = 1:numel(target_names)
                target_name = target_names{i_target};
                x0 = target_library.targets.(target_name).target_state(:) + cfg.initial_error(:);
                for i_event = 1:numel(events)
                    event = events(i_event);
                    if ~RecoveryEngine.recoveryCaseFilterAllows('target_recovery', target_name, event, cfg)
                        continue;
                    end
                    if RecoveryUtils.getLogical(cfg, 'recovery_resume_matrix', true) && RecoveryEngine.hasCompletedCase(rows, 'target_recovery', target_name, event)
                        continue;
                    end
                    case_index = case_index + 1;
                    cfg_case = cfg;
                    cfg_case.recovery_current_case_type = 'target_recovery';
                    sim = RecoveryEngine.simulateRecoveryCase(x0, target_name, event, params, cfg_case);
                    metrics = recoveryForceMetrics(sim, target_name, target_library.targets.(target_name).target_state(:), event, params, cfg_case);
                    metrics = RecoveryUtils.addSimDiagnostics(metrics, sim, event);
                    metrics = RecoveryControllers.attachCaseTrajectoryExport(result_dir, sim, metrics, case_index, 'target_recovery', target_name, event, cfg_case);
                    rows(end + 1, :) = RecoveryMetrics.metricsToRow(metrics, case_index, 'target_recovery'); %#ok<AGROW>
                    if RecoveryUtils.getLogical(cfg, 'recovery_resume_matrix', true)
                        RecoveryIo.writeCheckpointRows(checkpoint_target_csv, rows);
                    end

                    if case_index == 1 || (strcmp(target_name, 'up_up') && event.link_id == 2 && abs(event.contact_ratio - 1.0) < 1e-12 && strcmp(event.direction_name, 'right') && abs(event.force_N - 3.0) < 1e-12)
                        representative_sims.target_recovery = sim; %#ok<STRNU>
                        representative_sims.target_event = event;
                        representative_sims.target_name = target_name;
                    end
                end
            end

            case_table = cell2table(rows, 'VariableNames', RecoveryIo.caseTableNames());

            [post_state, post_target, post_source] = RecoveryEngine.getSwitchingFinalState(project_root, params, cfg);
            if RecoveryUtils.getDouble(cfg, 'post_switching_settle_time_s', 0.0) > 0
                post_state = RecoveryEngine.settleStateWithoutForce(post_state, post_target, params, cfg, RecoveryUtils.getDouble(cfg, 'post_switching_settle_time_s', 0.0));
                post_source = [post_source, '_then_local_settle_before_force'];
            end
            post_events = RecoveryEngine.filterPostSwitchingEvents(events, cfg);
            for i_event = 1:numel(post_events)
                event = post_events(i_event);
                if ~RecoveryEngine.recoveryCaseFilterAllows('after_full_switching', post_target, event, cfg)
                    continue;
                end
                if RecoveryUtils.getLogical(cfg, 'recovery_resume_matrix', true) && RecoveryEngine.hasCompletedCase(post_rows, 'after_full_switching', post_target, event)
                    continue;
                end
                cfg_case = cfg;
                cfg_case.recovery_current_case_type = 'after_full_switching';
                post_case_index = size(post_rows, 1) + 1;
                sim = RecoveryEngine.simulateRecoveryCase(post_state, post_target, event, params, cfg_case);
                metrics = recoveryForceMetrics(sim, post_target, target_library.targets.(post_target).target_state(:), event, params, cfg_case);
                metrics = RecoveryUtils.addSimDiagnostics(metrics, sim, event);
                metrics = RecoveryControllers.attachCaseTrajectoryExport(result_dir, sim, metrics, post_case_index, 'after_full_switching', post_target, event, cfg_case);
                post_rows(end + 1, :) = RecoveryMetrics.metricsToRow(metrics, post_case_index, 'after_full_switching'); %#ok<AGROW>
                if RecoveryUtils.getLogical(cfg, 'recovery_resume_matrix', true)
                    RecoveryIo.writeCheckpointRows(checkpoint_post_csv, post_rows);
                end
                if i_event == numel(post_events)
                    representative_sims.after_switching = sim;
                    representative_sims.after_switching_event = event;
                    representative_sims.after_switching_target = post_target;
                end
            end
            post_table = cell2table(post_rows, 'VariableNames', RecoveryIo.caseTableNames());

            all_table = [case_table; post_table];
            by_target_table = RecoveryMetrics.groupSuccessByTarget(case_table);
            by_force_table = RecoveryMetrics.groupSuccessByForce(case_table);
            by_link_table = RecoveryMetrics.groupSuccessByLink(case_table);
            by_post_table = RecoveryMetrics.groupSuccessByForce(post_table);
            by_planner_table = RecoveryMetrics.groupSuccessByPlanner(all_table);
            nominal_case_table = RecoveryEngine.filterForceSet(case_table, cfg.nominal_force_levels_N);
            nominal_post_table = RecoveryEngine.filterForceSet(post_table, cfg.nominal_force_levels_N);
            stress_case_table = RecoveryEngine.filterForceSet(case_table, cfg.stress_force_levels_N);
            stress_post_table = RecoveryEngine.filterForceSet(post_table, cfg.stress_force_levels_N);

            case_csv = fullfile(result_dir, 'recoveryCaseMetrics.csv');
            all_csv = fullfile(result_dir, 'recoveryAllMetrics.csv');
            target_csv = fullfile(result_dir, 'recoveryByTarget.csv');
            force_csv = fullfile(result_dir, 'recoveryByForce.csv');
            link_csv = fullfile(result_dir, 'recoveryByLink.csv');
            post_csv = fullfile(result_dir, 'afterSwitchingMetrics.csv');
            planner_csv = fullfile(result_dir, 'plannerVariantUsage.csv');
            writetable(case_table, case_csv);
            writetable(all_table, all_csv);
            writetable(by_target_table, target_csv);
            writetable(by_force_table, force_csv);
            writetable(by_link_table, link_csv);
            writetable(post_table, post_csv);
            writetable(by_planner_table, planner_csv);
            if RecoveryUtils.getLogical(cfg, 'recovery_write_gui_case_csv', true)
                RecoveryIo.writeRecoveryGuiCaseCsv(result_dir, all_table);
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_save_failed_cases', false)
                RecoveryIo.writeFailedDisturbedStatesCsv(all_table, result_dir, cfg);
            end

            target_success_rate = mean(case_table.success);
            post_switching_success_rate = mean(post_table.success);
            overall_success_rate = mean(all_table.success);
            nominal_target_success_rate = mean(nominal_case_table.success);
            nominal_post_switching_success_rate = mean(nominal_post_table.success);
            stress_target_success_rate = RecoveryMetrics.safeSuccessRate(stress_case_table);
            stress_post_switching_success_rate = RecoveryMetrics.safeSuccessRate(stress_post_table);
            overall_pass = nominal_target_success_rate >= cfg.min_target_success_rate && nominal_post_switching_success_rate >= cfg.min_post_switching_success_rate;
            stress_limit_note = RecoveryMetrics.stressLimitNote(stress_target_success_rate, stress_post_switching_success_rate);

            summary = struct();
            summary.workflow = 'recovery';
            summary.overall_pass = overall_pass;
            summary.target_success_rate = target_success_rate;
            summary.post_switching_success_rate = post_switching_success_rate;
            summary.overall_success_rate = overall_success_rate;
            summary.nominal_target_success_rate = nominal_target_success_rate;
            summary.nominal_post_switching_success_rate = nominal_post_switching_success_rate;
            summary.stress_target_success_rate = stress_target_success_rate;
            summary.stress_post_switching_success_rate = stress_post_switching_success_rate;
            summary.stress_limit_note = stress_limit_note;
            summary.target_case_count = height(case_table);
            summary.post_switching_case_count = height(post_table);
            summary.total_case_count = height(all_table);
            summary.max_external_force_N = params.external_force.max_external_force_N;
            summary.post_switching_initial_state_source = post_source;
            summary.post_switching_active_target = post_target;
            summary.result_dir = result_dir;
            summary.case_csv = case_csv;
            summary.all_csv = all_csv;
            summary.target_csv = target_csv;
            summary.force_csv = force_csv;
            summary.link_csv = link_csv;
            summary.post_csv = post_csv;
            summary.planner_csv = planner_csv;
            summary.gui_case_csv = fullfile(result_dir, 'recoveryGuiCases.csv');
            summary.case_trajectory_dir = fullfile(result_dir, char(cfg.recovery_case_trajectory_dir_name));
            summary.status_file = status_file;
            if isfield(cfg, 'recovery_mode') && strcmpi(char(string(cfg.recovery_mode)), 'BALANCE_WITHIN_2M_RAIL')
                summary.note = sprintf('Recovery BALANCE_WITHIN_2M_RAIL evaluates force levels %s N. Angle/velocity gates are preserved; cart gates are relaxed to evaluate balance within ±2 m rail.', mat2str(cfg.force_levels_N));
            elseif isfield(cfg, 'recovery_mode') && strcmpi(char(string(cfg.recovery_mode)), 'HARD_STATE_FALLBACK_1N_CONSTRAINED_RECOVERY')
                summary.note = sprintf('Recovery hard-state fallback evaluates 1N only. best50 pass cases are locked; only the 14 known hard fail states are routed through a constrained hard-state recovery trajectory library with TVLQR tracking and strict unchanged gates.');
            elseif isfield(cfg, 'recovery_mode') && RecoveryUtils.recoveryIsHardOrUpupMode(cfg)
                summary.note = sprintf('Recovery BALANCE_WITHIN_RAIL_HARD_STATE_CATCH_CONTROLLER evaluates 1N only. Local LQR is kept for easy states; the 16 hard fail families are routed through target-specific hard catch controllers with rail barrier, energy/angle catch, damping, and terminal LQR handoff.');
            elseif isfield(cfg, 'recovery_mode') && strcmpi(char(string(cfg.recovery_mode)), 'BALANCE_WITHIN_RAIL_CONSTRAINED_TERMINAL_RECOVERY')
                summary.note = sprintf('Recovery BALANCE_WITHIN_RAIL_CONSTRAINED_TERMINAL_RECOVERY evaluates 1N only. Local LQR is kept for easy states; hard fail families are routed through rail braking, angular damping, and terminal settle candidates before pass-first validation.');
            else
                summary.note = sprintf('Recovery evaluates force levels %s N. Local LQR is default. opt_library is only selected after actual simulation passes strict safety/pass gates.', mat2str(cfg.force_levels_N));
            end

            RecoveryUtils.makePlots(result_dir, case_table, post_table, representative_sims);
            try
                diagnoseRecoveryFailures(result_dir);
            catch ME
                warning('runRecoveryMatrixCore:FailureDiagnosticFailed', ...
                        'Could not write failure diagnostic tables: %s', ME.message);
            end
            try
                recoveryRegionDiagnostics(result_dir);
            catch ME
                warning('runRecoveryMatrixCore:ROARouterDiagnosticFailed', ...
                        'Could not write ROA router diagnostic tables: %s', ME.message);
            end
            RecoveryIo.writeSummaryText(result_dir, summary, cfg, by_target_table, by_force_table, by_link_table, by_post_table, by_planner_table);
            RecoveryIo.writeStatus(status_file, summary, cfg, by_target_table, by_force_table, by_link_table, by_post_table, by_planner_table);

            mat_file = fullfile(result_dir, 'recoveryResults.mat');
            save(mat_file, 'summary', 'case_table', 'post_table', 'all_table', 'by_target_table', 'by_force_table', 'by_link_table', 'by_post_table', 'by_planner_table', 'nominal_case_table', 'nominal_post_table', 'stress_case_table', 'stress_post_table', 'cfg', 'events', 'post_events', 'representative_sims');
            summary.mat_file = mat_file;

            fprintf('\nRecovery target success rate = %.1f %% (%d cases)\n', 100 * target_success_rate, height(case_table));
            fprintf('Recovery post switching success rate = %.1f %% (%d cases)\n', 100 * post_switching_success_rate, height(post_table));
            fprintf('Recovery nominal target success rate = %.1f %% for 1/2/3 N\n', 100 * nominal_target_success_rate);
            fprintf('Recovery nominal post switching success rate = %.1f %% for 1/2/3 N\n', 100 * nominal_post_switching_success_rate);
            fprintf('Recovery force levels evaluated in this run: %s N.\n', mat2str(cfg.force_levels_N));
            fprintf('Recovery overall pass = %d\n', overall_pass);
            fprintf('Case CSV: %s\n', case_csv);
            fprintf('Post switching CSV: %s\n', post_csv);
            fprintf('Summary: %s\n', fullfile(result_dir, 'recoverySummary.txt'));
        end

        function x_settled = settleStateWithoutForce(x0, target_name, params, cfg, settle_time_s)
            dt = double(cfg.dt_s);
            n = max(1, ceil(double(settle_time_s) / dt));
            x = double(x0(:));
            event = struct('name', 'no_force_settle', 'link_id', 1, 'contact_ratio', 0.5, ...
                           'force_N', 0.0, 'direction_name', 'right', 'direction_rad', 0.0, ...
                           'start_time_s', 1e9, 'duration_s', 0.0, 'end_time_s', 1e9, 'workflow', 'Recovery nominal settle');
            ctrl_cfg = cfg;
            ctrl_cfg.target_mode = target_name;
            sub_dt = dt / max(1, round(double(cfg.integration_substeps)));
            n_sub = max(1, round(double(cfg.integration_substeps)));
            for k = 1:n
                t_now = (k - 1) * dt;
                ctrl = disturbanceRecoveryController(t_now, x, params, ctrl_cfg);
                for j = 1:n_sub
                    t_sub = t_now + (j - 1) * sub_dt;
                    x = RecoveryUtils.rk4Step(x, t_sub, ctrl.u_cmd, event, params, sub_dt);
                end
            end
            x_settled = x;
        end

        function sim = simulateRecoveryCase(x0, target_name, event, params, cfg)
            if RecoveryUtils.getLogical(cfg, 'recovery_roa_traj_tvlqr_enable', false)
                sim = RecoveryEngine.simulateRoaTrajTvlqrRouter(x0, target_name, event, params, cfg);
                return;
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_enable_offline_trajectory_planner', true)
                target_library = targetEquilibriumLibrary(params);
                target_state = target_library.targets.(target_name).target_state(:);

                c_local = cfg;
                c_local.recovery_controller_variant = 'local_lqr';
                sim_local = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, c_local);
                met_local = recoveryForceMetrics(sim_local, target_name, target_state, event, params, cfg);
                cost_local = RecoveryUtils.recoveryPlannerCost(met_local, cfg);
                sim_local.planner_candidate = 'local_lqr';
                sim_local.planner_cost = cost_local;
                sim_local.planner_candidate_success = met_local.success;

                [try_opt, try_reason] = RecoveryControllers.shouldTryOptCandidate(met_local, target_name, event, cfg);
                selected_sim = sim_local;
                selected_cost = cost_local;
                selected_reason = ['LOCAL_DEFAULT_', try_reason];
                opt_used = false;
                cost_opt = NaN;
                met_opt = [];

                if try_opt && RecoveryUtils.getLogical(cfg, 'recovery_enable_optimized_library', true)
                    c_opt = cfg;
                    c_opt.recovery_controller_variant = 'opt_library';
                    sim_opt = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, c_opt);
                    met_opt = recoveryForceMetrics(sim_opt, target_name, target_state, event, params, cfg);
                    cost_opt = RecoveryUtils.recoveryPlannerCost(met_opt, cfg);
                    sim_opt.planner_candidate = 'opt_library';
                    sim_opt.planner_cost = cost_opt;
                    sim_opt.planner_candidate_success = met_opt.success;
                    opt_used = RecoveryControllers.simUsedOptTrajectory(sim_opt);
                    improve_margin = RecoveryUtils.getDouble(cfg, 'recovery_planner_opt_cost_improvement_margin', 0.50);
                    choose_opt = RecoveryControllers.shouldSelectOptCandidate(met_local, cost_local, met_opt, cost_opt, opt_used, improve_margin, cfg);
                    if choose_opt
                        selected_sim = sim_opt;
                        selected_cost = cost_opt;
                        selected_reason = 'OPT_SELECTED_safe optimization_SAFE_ACTUAL_SIMULATION_BETTER';
                    else
                        selected_reason = 'LOCAL_SELECTED_safe optimization_OPT_REJECTED_OR_NOT_USED';
                    end
                end

                sim = selected_sim;
                sim.planner_selected_variant = sim.planner_candidate;
                sim.planner_selected_cost = selected_cost;
                sim.planner_try_opt = logical(try_opt);
                sim.planner_try_opt_reason = try_reason;
                sim.planner_opt_used_trajectory = logical(opt_used);
                sim.planner_local_cost = cost_local;
                sim.planner_opt_cost = cost_opt;
                sim.planner_decision_reason = selected_reason;
                if ~isempty(met_opt)
                    sim.planner_opt_success = logical(met_opt.success);
                    sim.planner_opt_roa_score = RecoveryControllers.metricRoaScore(met_opt, cfg);
                else
                    sim.planner_opt_success = false;
                    sim.planner_opt_roa_score = NaN;
                end
                sim.planner_local_success = logical(met_local.success);
                sim.planner_local_roa_score = RecoveryControllers.metricRoaScore(met_local, cfg);
                sim.controller_family = 'recovery_safeOpt_safe_opt_selection_candidate_competition';
            else
                sim = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, cfg);
                sim.planner_selected_variant = 'disabled_single_run';
                sim.planner_selected_cost = NaN;
                sim.controller_family = 'recovery_single_controller';
            end
        end

        function sim = simulateRecoveryCaseSingle(x0, target_name, event, params, cfg)
            dt = double(cfg.dt_s);
            sim_time = double(cfg.sim_time_s);
            if (RecoveryUtils.recoveryIsUpupForceAutotuneMode(cfg) || RecoveryControllers.recoveryIsHardStateLibraryOptimizerMode(cfg) || RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg)) && strcmp(char(target_name), 'up_up') && ...
                    abs(double(event.force_N) - 1.0) < 1e-9 && ...
                    ((double(event.link_id) == 1 && double(event.contact_ratio) >= 1.0 - 1e-12) || ...
                     (double(event.link_id) == 2 && double(event.contact_ratio) >= 0.75 - 1e-12))
                sim_time = sim_time + RecoveryUtils.getDouble(cfg, 'recovery_upup_hard_extra_time_s', 2.00);
            end
            t = (0:dt:sim_time).';
            n = numel(t);
            states = zeros(n, 6);
            u_cmd = zeros(n, 1);
            u_raw = zeros(n, 1);
            q_external = zeros(n, 3);
            force_x_N = zeros(n, 1);
            force_y_N = zeros(n, 1);
            active = false(n, 1);
            recovery_state = cell(n, 1);
            states(1, :) = double(x0(:)).';

            ctrl_cfg = cfg;
            ctrl_cfg.target_mode = target_name;
            variant = RecoveryUtils.getChar(cfg, 'recovery_controller_variant', 'local_lqr');
            sub_dt = dt / max(1, round(double(cfg.integration_substeps)));
            n_sub = max(1, round(double(cfg.integration_substeps)));
            recovery_anchor_state = [];
            recovery_anchor_time = double(event.start_time_s) + double(event.duration_s) + RecoveryUtils.getDouble(cfg, 'recovery_traj_recovery_start_safety_delay', 0.02);

            for k = 1:(n - 1)
                use_traj = ~strcmp(variant, 'local_lqr') && t(k) >= recovery_anchor_time;
                if use_traj && isempty(recovery_anchor_state)
                    recovery_anchor_state = states(k, :).';
                end
                if use_traj
                    if strcmp(variant, 'opt_library')
                        opt_traj = RecoveryControllers.selectOptimizedRecoveryTraj(recovery_anchor_state, target_name, event, cfg);
                        if ~isempty(opt_traj)
                            t_rel = t(k) - recovery_anchor_time;
                            t_end = double(opt_traj.time_s(end));
                            if t_rel <= t_end
                                ctrl = tvlqrRecoveryTracking(t_rel, states(k, :).', opt_traj, params, ctrl_cfg);
                                ctrl.recovery_state = 'optimized_library_recovery_tracking';
                            else
                                ctrl = disturbanceRecoveryController(t(k), states(k, :).', params, ctrl_cfg);
                                ctrl.recovery_state = 'optimized_library_post_trajectory_local_settle';
                            end
                        else
                            ctrl = disturbanceRecoveryController(t(k), states(k, :).', params, ctrl_cfg);
                            ctrl.recovery_state = 'opt_library_unavailable_fallback_local_lqr';
                        end
                    else
                        ctrl = RecoveryControllers.trajectoryRecoveryController(t(k), states(k, :).', target_name, event, params, ctrl_cfg, recovery_anchor_state, recovery_anchor_time, variant);
                    end
                else
                    ctrl = disturbanceRecoveryController(t(k), states(k, :).', params, ctrl_cfg);
                end
                ctrl = RecoveryEngine.applyRecoveryRuntimeConstraints(ctrl, states(k, :).', params, cfg);
                u_cmd(k) = ctrl.u_cmd;
                u_raw(k) = ctrl.u_raw;
                recovery_state{k} = ctrl.recovery_state;
                x_next = states(k, :).';
                for j = 1:n_sub
                    t_sub = t(k) + (j - 1) * sub_dt;
                    x_next = RecoveryUtils.rk4Step(x_next, t_sub, u_cmd(k), event, params, sub_dt);
                end
                states(k + 1, :) = x_next.';
                [q_now, fx_now, fy_now, active_now] = RecoveryEngine.externalAtTime(states(k, :).', t(k), event, params);
                q_external(k, :) = q_now.';
                force_x_N(k) = fx_now;
                force_y_N(k) = fy_now;
                active(k) = active_now;
            end

            if ~strcmp(variant, 'local_lqr') && t(n) >= recovery_anchor_time
                if isempty(recovery_anchor_state), recovery_anchor_state = states(n, :).'; end
                if strcmp(variant, 'opt_library')
                    opt_traj = RecoveryControllers.selectOptimizedRecoveryTraj(recovery_anchor_state, target_name, event, cfg);
                    if ~isempty(opt_traj)
                        t_rel = t(n) - recovery_anchor_time;
                        t_end = double(opt_traj.time_s(end));
                        if t_rel <= t_end
                            ctrl = tvlqrRecoveryTracking(t_rel, states(n, :).', opt_traj, params, ctrl_cfg);
                            ctrl.recovery_state = 'optimized_library_recovery_tracking';
                        else
                            ctrl = disturbanceRecoveryController(t(n), states(n, :).', params, ctrl_cfg);
                            ctrl.recovery_state = 'optimized_library_post_trajectory_local_settle';
                        end
                    else
                        ctrl = disturbanceRecoveryController(t(n), states(n, :).', params, ctrl_cfg);
                        ctrl.recovery_state = 'opt_library_unavailable_fallback_local_lqr';
                    end
                else
                    ctrl = RecoveryControllers.trajectoryRecoveryController(t(n), states(n, :).', target_name, event, params, ctrl_cfg, recovery_anchor_state, recovery_anchor_time, variant);
                end
            else
                ctrl = disturbanceRecoveryController(t(n), states(n, :).', params, ctrl_cfg);
            end
            ctrl = RecoveryEngine.applyRecoveryRuntimeConstraints(ctrl, states(n, :).', params, cfg);
            u_cmd(n) = ctrl.u_cmd;
            u_raw(n) = ctrl.u_raw;
            recovery_state{n} = ctrl.recovery_state;
            [q_last, fx_last, fy_last, active_last] = RecoveryEngine.externalAtTime(states(n, :).', t(n), event, params);
            q_external(n, :) = q_last.';
            force_x_N(n) = fx_last;
            force_y_N(n) = fy_last;
            active(n) = active_last;

            sim = struct();
            sim.time_s = t;
            sim.states = states;
            sim.u_cmd = u_cmd;
            sim.u_raw = u_raw;
            sim.q_external = q_external;
            sim.force_x_N = force_x_N;
            sim.force_y_N = force_y_N;
            sim.active = active;
            sim.recovery_state = recovery_state;
            sim.target_name = target_name;
            sim.event = event;
            sim.controller_variant = variant;
        end

        function sim = simulateRoaTrajTvlqrRouter(x0, target_name, event, params, cfg)
            target_library = targetEquilibriumLibrary(params);
            target_state = target_library.targets.(target_name).target_state(:);

            c_local = cfg;
            c_local.recovery_controller_variant = 'local_lqr';
            sim_local = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, c_local);
            met_local = recoveryForceMetrics(sim_local, target_name, target_state, event, params, cfg);
            cost_local = RecoveryUtils.recoveryPlannerCost(met_local, cfg);
            sim_local.planner_candidate = 'local_lqr';
            sim_local.planner_cost = cost_local;
            sim_local.planner_candidate_success = logical(met_local.success);

            if RecoveryUtils.getLogical(cfg, 'recovery_best50_lock_enable', false)
                [lock_case, lock_variant] = RecoveryVariants.recoveryLookupBest50Lock(target_name, event, cfg);
                if lock_case
                    if strcmp(lock_variant, 'local_lqr')
                        sim = sim_local;
                        met_lock = met_local;
                        cost_lock = cost_local;
                    else
                        c_lock = cfg;
                        c_lock.recovery_controller_variant = lock_variant;
                        sim = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, c_lock);
                        met_lock = recoveryForceMetrics(sim, target_name, target_state, event, params, cfg);
                        cost_lock = RecoveryUtils.recoveryPlannerCost(met_lock, cfg);
                    end
                    sim.planner_selected_variant = lock_variant;
                    sim.planner_selected_cost = cost_lock;
                    sim.planner_try_opt = false;
                    sim.planner_try_opt_reason = 'BEST50_BASELINE_PASS_CASE_LOCKED';
                    sim.planner_force_takeover = false;
                    sim.planner_opt_used_trajectory = ~strcmp(lock_variant, 'local_lqr');
                    sim.planner_local_cost = cost_local;
                    sim.planner_opt_cost = NaN;
                    sim.planner_decision_reason = ['BEST50_LOCKED_BASELINE_PASS_VARIANT_', lock_variant];
                    sim.planner_local_success = logical(met_local.success);
                    sim.planner_local_roa_score = RecoveryControllers.metricRoaScore(met_local, cfg);
                    sim.planner_opt_success = ~strcmp(lock_variant, 'local_lqr') && logical(met_lock.success);
                    sim.planner_opt_roa_score = RecoveryControllers.metricRoaScore(met_lock, cfg);
                    sim.planner_tried_variants = lock_variant;
                    sim.planner_candidate_success_vector = char(string(logical(met_lock.success)));
                    sim.planner_candidate_hard_ok_vector = char(string(RecoveryControllers.candidateSatisfiesHardConstraints(met_lock, cfg)));
                    sim.planner_candidate_roa_score_vector = char(string(RecoveryControllers.metricRoaScore(met_lock, cfg)));
                    sim.planner_candidate_validation_vector = ['BEST50_LOCK_', RecoveryControllers.candidateValidationLabel(met_lock, cfg)];
                    sim.controller_family = 'recovery_balance_locked_router';
                    return;
                end
            end

            [try_traj, try_reason] = RecoveryControllers.shouldTryRoaTrajCandidate(met_local, target_name, event, cfg);
            force_takeover = RecoveryUtils.getLogical(cfg, 'recovery_traj_takeover_enable', false) && try_traj && ...
                             ((~logical(met_local.success) && RecoveryUtils.getLogical(cfg, 'recovery_force_takeover_on_local_fail', true)) || ...
                              (RecoveryVariants.isPriority1nFailState(target_name, event, cfg) && RecoveryUtils.getLogical(cfg, 'recovery_force_takeover_on_priority_fail_state', true)));

            tried_variants = {'local_lqr'};
            candidate_costs = cost_local;
            candidate_success = logical(met_local.success);
            candidate_hard_ok = RecoveryControllers.candidateSatisfiesHardConstraints(met_local, cfg);
            candidate_roa_scores = RecoveryControllers.metricRoaScore(met_local, cfg);
            candidate_decisions = {'LOCAL_BASELINE'};

            best_sim = sim_local;
            best_met = met_local;
            best_cost = cost_local;
            best_variant = 'local_lqr';
            decision = ['LOCAL_ROA_DEFAULT_', try_reason];

            nonlocal_sims = {};
            nonlocal_mets = {};
            nonlocal_costs = [];
            nonlocal_variants = {};
            nonlocal_hard_ok = [];

            if try_traj
                variants = RecoveryControllers.buildTakeoverVariantList(target_name, event, cfg);
                for i = 1:numel(variants)
                    variant = char(variants{i});
                    c_try = cfg;
                    c_try.recovery_controller_variant = variant;
                    sim_try = RecoveryEngine.simulateRecoveryCaseSingle(x0, target_name, event, params, c_try);
                    met_try = recoveryForceMetrics(sim_try, target_name, target_state, event, params, cfg);
                    cost_try = RecoveryUtils.recoveryPlannerCost(met_try, cfg);
                    hard_ok = RecoveryControllers.candidateSatisfiesHardConstraints(met_try, cfg);
                    sim_try.planner_candidate = variant;
                    sim_try.planner_cost = cost_try;
                    sim_try.planner_candidate_success = logical(met_try.success);

                    tried_variants{end+1} = variant; %#ok<AGROW>
                    candidate_costs(end+1) = cost_try; %#ok<AGROW>
                    candidate_success(end+1) = logical(met_try.success); %#ok<AGROW>
                    candidate_hard_ok(end+1) = logical(hard_ok); %#ok<AGROW>
                    candidate_roa_scores(end+1) = RecoveryControllers.metricRoaScore(met_try, cfg); %#ok<AGROW>
                    candidate_decisions{end+1} = RecoveryControllers.candidateValidationLabel(met_try, cfg); %#ok<AGROW>

                    nonlocal_sims{end+1} = sim_try; %#ok<AGROW>
                    nonlocal_mets{end+1} = met_try; %#ok<AGROW>
                    nonlocal_costs(end+1) = cost_try; %#ok<AGROW>
                    nonlocal_variants{end+1} = variant; %#ok<AGROW>
                    nonlocal_hard_ok(end+1) = logical(hard_ok); %#ok<AGROW>
                end

                pass_idx = find(cellfun(@(m) logical(m.success), nonlocal_mets));
                if ~isempty(pass_idx)
                    if (RecoveryUtils.recoveryIsUpupForceAutotuneMode(cfg) || RecoveryControllers.recoveryIsHardStateLibraryOptimizerMode(cfg) || RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg)) && strcmp(char(target_name), 'up_up')
                        pass_scores = zeros(size(pass_idx));
                        for is = 1:numel(pass_idx)
                            pass_scores(is) = RecoveryControllers.upupFailedCandidateScore(nonlocal_mets{pass_idx(is)}, nonlocal_variants{pass_idx(is)}, cfg);
                        end
                        [~, rel] = min(pass_scores);
                    else
                        [~, rel] = min(nonlocal_costs(pass_idx));
                    end
                    idx = pass_idx(rel);
                    best_sim = nonlocal_sims{idx};
                    best_met = nonlocal_mets{idx};
                    best_cost = nonlocal_costs(idx);
                    best_variant = nonlocal_variants{idx};
                    decision = ['TRAJ_TAKEOVER_PASS_SELECTED_', RecoveryControllers.candidateValidationLabel(best_met, cfg)];
                elseif force_takeover && RecoveryUtils.getLogical(cfg, 'recovery_select_failed_nonlocal_for_debug', true) && ~isempty(nonlocal_sims)
                    ok_idx = find(nonlocal_hard_ok);
                    if (RecoveryUtils.recoveryIsUpupForceAutotuneMode(cfg) || RecoveryControllers.recoveryIsHardStateLibraryOptimizerMode(cfg) || RecoveryControllers.recoveryIsStateSpecificGainSearchMode(cfg)) && strcmp(char(target_name), 'up_up')
                        all_scores = zeros(1, numel(nonlocal_mets));
                        for is = 1:numel(nonlocal_mets)
                            all_scores(is) = RecoveryControllers.upupFailedCandidateScore(nonlocal_mets{is}, nonlocal_variants{is}, cfg);
                        end
                        [~, idx] = min(all_scores);
                    elseif ~isempty(ok_idx)
                        [~, rel] = min(nonlocal_costs(ok_idx));
                        idx = ok_idx(rel);
                    else
                        roa_vals = cellfun(@(m) RecoveryControllers.metricRoaScore(m, cfg), nonlocal_mets);
                        [~, idx] = min(roa_vals);
                    end
                    best_sim = nonlocal_sims{idx};
                    best_met = nonlocal_mets{idx};
                    best_cost = nonlocal_costs(idx);
                    best_variant = nonlocal_variants{idx};
                    decision = ['NO_TRAJ_CANDIDATE_PASSED_BEST_NONLOCAL_SELECTED_', RecoveryControllers.candidateValidationLabel(best_met, cfg)];
                else
                    best_sim = sim_local;
                    best_met = met_local;
                    best_cost = cost_local;
                    best_variant = 'local_lqr';
                    if try_traj
                        decision = 'LOCAL_KEPT_NO_TRAJ_PASS_AND_LOCAL_ACCEPTABLE';
                    else
                        decision = ['LOCAL_INSIDE_ROA_', try_reason];
                    end
                end
            end

            sim = best_sim;
            sim.planner_selected_variant = best_variant;
            sim.planner_selected_cost = best_cost;
            sim.planner_try_opt = logical(try_traj);
            sim.planner_try_opt_reason = try_reason;
            sim.planner_force_takeover = logical(force_takeover);
            sim.planner_opt_used_trajectory = ~strcmp(best_variant, 'local_lqr');
            sim.planner_local_cost = cost_local;
            nonlocal_mask = ~strcmp(tried_variants, 'local_lqr');
            if any(nonlocal_mask)
                sim.planner_opt_cost = min(candidate_costs(nonlocal_mask));
            else
                sim.planner_opt_cost = NaN;
            end
            sim.planner_decision_reason = decision;
            sim.planner_local_success = logical(met_local.success);
            sim.planner_local_roa_score = RecoveryControllers.metricRoaScore(met_local, cfg);
            sim.planner_opt_success = ~strcmp(best_variant, 'local_lqr') && logical(best_met.success);
            sim.planner_opt_roa_score = RecoveryControllers.metricRoaScore(best_met, cfg);
            sim.planner_tried_variants = strjoin(tried_variants, '|');
            sim.planner_candidate_success_vector = strjoin(cellstr(string(candidate_success)), '|');
            sim.planner_candidate_hard_ok_vector = strjoin(cellstr(string(candidate_hard_ok)), '|');
            sim.planner_candidate_roa_score_vector = strjoin(cellstr(string(candidate_roa_scores)), '|');
            sim.planner_candidate_validation_vector = strjoin(candidate_decisions, '|');
            if isfield(cfg, 'recovery_mode') && strcmpi(char(string(cfg.recovery_mode)), 'BALANCE_WITHIN_RAIL_UPUP_CATCH_REDESIGN')
                sim.controller_family = 'recovery_upup_catch_router';
            elseif isfield(cfg, 'recovery_mode') && RecoveryUtils.recoveryIsHardOrUpupMode(cfg)
                sim.controller_family = 'recovery_hard_state_catch_router';
            else
                sim.controller_family = 'recovery_terminal_router';
            end
        end

    end
end
