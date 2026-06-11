classdef RecoveryUtils
    methods (Static)
        function m = addSimDiagnostics(m, sim, event)
            xaf = RecoveryEngine.afterForceState(sim, event);
            m.x_after_force = xaf(1);
            m.xdot_after_force = xaf(2);
            m.theta1_after_force = xaf(3);
            m.theta1dot_after_force = xaf(4);
            m.theta2_after_force = xaf(5);
            m.theta2dot_after_force = xaf(6);
            if isfield(sim, 'planner_selected_variant') && ~isempty(sim.planner_selected_variant)
                m.planner_selected_variant = char(sim.planner_selected_variant);
            elseif isfield(sim, 'controller_variant')
                m.planner_selected_variant = char(sim.controller_variant);
            else
                m.planner_selected_variant = 'unknown';
            end
            if isfield(sim, 'planner_selected_cost') && ~isempty(sim.planner_selected_cost)
                m.planner_selected_cost = double(sim.planner_selected_cost);
            else
                m.planner_selected_cost = NaN;
            end
            if ~isfield(m, 'recovery_roa_score') || isempty(m.recovery_roa_score)
                m.recovery_roa_score = RecoveryControllers.metricRoaScore(m, struct());
            end
            if ~isfield(m, 'recovery_margin_score') || isempty(m.recovery_margin_score)
                m.recovery_margin_score = RecoveryMetrics.recoveryMarginScore(m);
            end
            if ~isfield(m, 'primary_fail_reason') || isempty(m.primary_fail_reason)
                m.primary_fail_reason = RecoveryMetrics.primaryFailReason(m.fail_reason);
            end
            if isfield(sim, 'planner_try_opt'), m.planner_try_opt = logical(sim.planner_try_opt); else, m.planner_try_opt = false; end
            if isfield(sim, 'planner_opt_used_trajectory'), m.planner_opt_used_trajectory = logical(sim.planner_opt_used_trajectory); else, m.planner_opt_used_trajectory = false; end
            if isfield(sim, 'planner_local_cost'), m.planner_local_cost = double(sim.planner_local_cost); else, m.planner_local_cost = NaN; end
            if isfield(sim, 'planner_opt_cost'), m.planner_opt_cost = double(sim.planner_opt_cost); else, m.planner_opt_cost = NaN; end
            if isfield(sim, 'planner_decision_reason'), m.planner_decision_reason = char(sim.planner_decision_reason); else, m.planner_decision_reason = 'NO_PLANNER_DECISION_LOG'; end
            if isfield(sim, 'planner_force_takeover'), m.planner_force_takeover = logical(sim.planner_force_takeover); else, m.planner_force_takeover = false; end
            if isfield(sim, 'planner_tried_variants'), m.planner_tried_variants = char(sim.planner_tried_variants); else, m.planner_tried_variants = ''; end
            if isfield(sim, 'planner_candidate_validation_vector'), m.planner_candidate_validation_vector = char(sim.planner_candidate_validation_vector); else, m.planner_candidate_validation_vector = ''; end
        end

        function cfg = defaultCfg(user_cfg, project_root)
            cfg = struct();
            cfg.project_root = project_root;
            cfg.target_names = {'up_up', 'up_down', 'down_up'};
            cfg.link_ids = [1, 2];
            cfg.contact_ratios = [0.25, 0.50, 0.75, 1.00];
            cfg.direction_names = {'right', 'left'};
            cfg.direction_rad = [0.0, pi];
            cfg.force_levels_N = [1.0, 2.0, 3.0, 4.0, 5.0];
            cfg.force_start_time_s = 0.70;
            cfg.force_duration_s = 0.08;
            cfg.dt_s = 0.01;
            cfg.integration_substeps = 4;
            cfg.sim_time_s = 7.00;
            cfg.initial_settle_time_s = 0.30;
            cfg.initial_error = [0.002; 0.00; 0.0015; 0.00; -0.0015; 0.00];
            cfg.recovery_angle_rad = 0.10;
            cfg.recovery_velocity_norm = 0.90;
            cfg.recovery_cart_abs_m = 0.85;
            cfg.global_cart_limit_m = 1.00;
            cfg.max_saturation_fraction = 0.70;
            cfg.settle_hold_s = 0.20;
            cfg.min_target_success_rate = 0.65;
            cfg.min_post_switching_success_rate = 0.50;
            cfg.post_switching_force_levels_N = [1.0, 2.0, 3.0, 4.0, 5.0];
            cfg.nominal_force_levels_N = [1.0, 2.0, 3.0];
            cfg.stress_force_levels_N = [5.0];
            cfg.post_switching_settle_time_s = 3.00;
            cfg.recovery_cart_kp = 1.5;
            cfg.recovery_cart_kd = 0.6;
            cfg.recovery_theta1_damping = 0.10;
            cfg.recovery_theta2_damping = 0.08;
            cfg.recovery_theta1_kp_assist = 0.03;
            cfg.recovery_theta2_kp_assist = 0.02;
            cfg.recovery_lqr_scale_min = 1.00;
            cfg.recovery_lqr_scale_max = 1.20;
            cfg.recovery_aux_limit_fraction = 0.08;
            cfg.recovery_enable_overlay = false;
            cfg.recovery_soft_limit_fraction_min = 0.78;
            cfg.recovery_soft_limit_fraction_max = 0.93;
            cfg.recovery_enable_state_machine = true;
            cfg.recovery_enable_recovery_trajectory_hook = false;
            cfg.recovery_enable_offline_trajectory_planner = true;
            cfg.recovery_enable_optimized_library = true;
            cfg.recovery_auto_build_optimized_library = true;
            cfg.recovery_opt_max_representatives = 18;
            cfg.recovery_opt_max_iter = 60;
            cfg.recovery_opt_control_knot_count = 13;
            cfg.recovery_opt_dt_s = 0.025;
            cfg.recovery_opt_candidate_durations_s = [2.5 3.5 5.0 6.5];
            cfg.recovery_planner_variants = {'local_lqr','opt_library'};
            cfg.recovery_opt_route_require_validated = true;
            cfg.recovery_opt_max_route_distance = 3.0;
            cfg.recovery_opt_max_route_distance_pass = 4.25;
            cfg.recovery_opt_max_route_distance_near = 2.60;
            cfg.recovery_opt_allow_near_pass_route = true;
            cfg.recovery_opt_force_tolerance_pass_N = 1.25;
            cfg.recovery_opt_force_tolerance_near_N = 1.10;
            cfg.recovery_planner_stop_on_first_success = false;
            cfg.recovery_planner_skip_opt_if_local_near_pass = false;
            cfg.recovery_planner_opt_cost_improvement_margin = 0.50;
            cfg.recovery_roaRouter_enable_roa_router = true;
            cfg.recovery_roaRouter_priority_targets = {'up_up','down_up'};
            cfg.recovery_roaRouter_nominal_opt_forces_N = [2.0, 3.0];
            cfg.recovery_roaRouter_local_high_saturation_fraction = 0.42;
            cfg.recovery_roaRouter_local_slow_recovery_s = 2.20;
            cfg.recovery_roaRouter_local_high_roa_score = 7.50;
            cfg.recovery_roaRouter_select_if_opt_success_and_local_fail = true;
            cfg.recovery_roaRouter_select_if_opt_success_and_roa_better = true;
            cfg.recovery_safeOpt_safe_opt_selection = true;
            cfg.recovery_safeOpt_require_opt_success_for_selection = true;
            cfg.recovery_safeOpt_reject_opt_if_rail_fail = true;
            cfg.recovery_safeOpt_opt_max_saturation_fraction = 0.35;
            cfg.recovery_safeOpt_opt_max_final_cart_abs_m = 0.85;
            cfg.recovery_safeOpt_opt_max_peak_cart_abs_m = 0.95;
            cfg.recovery_safeOpt_min_roa_improvement = 0.35;
            cfg.recovery_safeOpt_min_cost_improvement = 50.0;
            cfg.recovery_safeOpt_max_opt_recovery_time_s = 6.00;
            cfg.recovery_safeOpt_allow_safe_failed_opt_selection = false;
            cfg.recovery_safeOpt_route_near_pass_entries = false;
            cfg.recovery_resume_matrix = true;
            cfg.recovery_reset_matrix_checkpoint = false;
            cfg.recovery_export_all_case_trajectories = true;
            cfg.recovery_case_trajectory_dir_name = 'case_trajectories';
            cfg.recovery_write_gui_case_csv = true;
            cfg.recovery_opt_post_traj_settle_s = 1.00;
            cfg.recovery_disable_unvalidated_heuristics = true;
            cfg.recovery_traj_recovery_start_safety_delay = 0.02;
            cfg.recovery_traj_fast_duration_s = 1.60;
            cfg.recovery_traj_nominal_duration_s = 2.80;
            cfg.recovery_traj_slow_duration_s = 4.40;
            cfg.recovery_traj_cart_safe_duration_s = 3.40;
            cfg.recovery_traj_aggressive_duration_s = 2.20;
            cfg.recovery_traj_terminal_blend_min = 0.18;
            cfg.recovery_traj_terminal_blend_max = 0.90;
            cfg.recovery_traj_track_scale = 0.80;
            cfg.recovery_traj_terminal_scale = 1.00;
            cfg.recovery_traj_cart_safe_gain = 4.5;
            cfg.recovery_traj_cart_safe_damping = 1.6;
            cfg.recovery_traj_angle_damping = 0.18;
            cfg.recovery_planner_success_bonus = 2000.0;
            cfg.recovery_roa_angle_weight = 2.0;
            cfg.recovery_roa_velocity_weight = 1.2;
            cfg.recovery_roa_cart_weight = 1.4;
            cfg.recovery_roa_saturation_weight = 2.5;
            cfg.recovery_roa_rail_weight = 3.0;
            cfg.recovery_lqr_scale_min = 1.00;
            cfg.recovery_lqr_scale_max = 1.35;
            cfg.recovery_aux_limit_fraction = 0.18;
            cfg.recovery_cart_safety_abs_m = 0.62;
            cfg.recovery_cart_priority_abs_m = 0.62;
            cfg.recovery_large_angle_rad = 0.22;
            cfg.recovery_large_velocity_norm = 1.65;
            fields = fieldnames(user_cfg);
            for i = 1:numel(fields)
                cfg.(fields{i}) = user_cfg.(fields{i});
            end

            if isfield(user_cfg, 'recovery_enable_roa_router')
                cfg.recovery_roaRouter_enable_roa_router = logical(user_cfg.recovery_enable_roa_router);
            end
            if isfield(user_cfg, 'recovery_enable_opt_library')
                cfg.recovery_enable_optimized_library = logical(user_cfg.recovery_enable_opt_library);
            end
            if isfield(user_cfg, 'recovery_candidate_competition')
                cfg.recovery_enable_offline_trajectory_planner = logical(user_cfg.recovery_candidate_competition);
            end
            if isfield(user_cfg, 'recovery_allow_near_pass_opt_try')
                cfg.recovery_opt_allow_near_pass_route = logical(user_cfg.recovery_allow_near_pass_opt_try);
            end
            if isfield(user_cfg, 'recovery_force_levels_N')
                cfg.force_levels_N = double(user_cfg.recovery_force_levels_N);
            end

            if RecoveryUtils.getLogical(cfg, 'recovery_safeOpt_safe_opt_selection', true) && ~RecoveryUtils.getLogical(cfg, 'recovery_safeOpt_route_near_pass_entries', false)
                cfg.recovery_opt_allow_near_pass_route = false;
            end
        end

        function value = getCell(s, name, default_value)
            value = default_value;
            if nargin >= 2 && isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = s.(name);
            end
        end

        function value = getChar(s, name, default_value)
            value = default_value;
            if nargin >= 2 && isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = char(s.(name));
            end
        end

        function value = getDouble(s, name, default_value)
            value = default_value;
            if nargin >= 2 && isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                raw = s.(name);
                if isnumeric(raw) || islogical(raw)
                    value = double(raw(1));
                else
                    parsed = str2double(char(raw));
                    if isfinite(parsed)
                        value = parsed;
                    end
                end
            end
        end

        function value = getLogical(s, name, default_value)
            value = default_value;
            if nargin >= 2 && isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = logical(s.(name));
            end
        end

        function value = getOptionalChar(s, name, default_value)
            value = default_value;
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = char(string(s.(name)));
            end
        end

        function value = getOptionalDouble(s, name, default_value)
            value = default_value;
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = double(s.(name));
            end
        end

        function value = getVector(s, name, default_value)
            value = default_value;
            if nargin >= 2 && isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                value = double(s.(name));
            end
        end

        function makePlots(result_dir, case_table, post_table, representative_sims)
            fig = figure('Visible', 'off');
            forces = unique(case_table.force_N, 'stable');
            rates = zeros(numel(forces), 1);
            for i = 1:numel(forces)
                mask = abs(case_table.force_N - forces(i)) < 1e-12;
                rates(i) = 100 * mean(case_table.success(mask));
            end
            bar(forces, rates);
            grid on; xlabel('force level (N)'); ylabel('success rate (%)');
            title('Recovery target recovery success rate by force');
            ylim([0, 100]);
            saveas(fig, fullfile(result_dir, 'recovery_success_rate_by_force.png'));
            close(fig);

            fig = figure('Visible', 'off');
            targets = unique(case_table.target, 'stable');
            rates = zeros(numel(targets), 1);
            for i = 1:numel(targets)
                mask = strcmp(case_table.target, targets{i});
                rates(i) = 100 * mean(case_table.success(mask));
            end
            bar(rates);
            set(gca, 'XTick', 1:numel(targets), 'XTickLabel', targets);
            grid on; xlabel('target'); ylabel('success rate (%)');
            title('Recovery recovery success rate by target');
            ylim([0, 100]);
            saveas(fig, fullfile(result_dir, 'recovery_success_rate_by_target.png'));
            close(fig);

            if isfield(representative_sims, 'target_recovery')
                RecoveryUtils.plotSim(fullfile(result_dir, 'recovery_representative_target_recovery.png'), representative_sims.target_recovery, 'Recovery representative target recovery');
            end
            if isfield(representative_sims, 'after_switching')
                RecoveryUtils.plotSim(fullfile(result_dir, 'recovery_after_full_switching_recovery.png'), representative_sims.after_switching, 'Recovery recovery after switching demo full switching');
            end

            fig = figure('Visible', 'off');
            bar(100 * mean(post_table.success));
            grid on; ylabel('success rate (%)');
            set(gca, 'XTickLabel', {'after full switching'});
            title('Recovery post-switching recovery success rate');
            ylim([0, 100]);
            saveas(fig, fullfile(result_dir, 'recovery_after_full_switching_success_rate.png'));
            close(fig);
        end

        function v = nanmean(x)
            x = x(isfinite(x));
            if isempty(x)
                v = NaN;
            else
                v = mean(x);
            end
        end

        function plotSim(path, sim, title_text)
            fig = figure('Visible', 'off');
            subplot(3, 1, 1);
            plot(sim.time_s, sim.force_x_N, 'LineWidth', 1.2); hold on;
            plot(sim.time_s, sim.force_y_N, 'LineWidth', 1.2);
            grid on; ylabel('force (N)'); legend({'F_x', 'F_y'}, 'Location', 'best');
            title(title_text);

            subplot(3, 1, 2);
            plot(sim.time_s, sim.states(:, 1), 'LineWidth', 1.2); hold on;
            plot(sim.time_s, sim.states(:, 3), 'LineWidth', 1.2);
            plot(sim.time_s, sim.states(:, 5), 'LineWidth', 1.2);
            grid on; ylabel('state'); legend({'x', 'theta1', 'theta2'}, 'Location', 'best');

            subplot(3, 1, 3);
            plot(sim.time_s, sim.u_cmd, 'LineWidth', 1.2); hold on;
            plot(sim.time_s, vecnorm(sim.q_external, 2, 2), 'LineWidth', 1.2);
            grid on; xlabel('time (s)'); ylabel('control / Qext'); legend({'u_{cmd}', '||Q_{ext}||'}, 'Location', 'best');
            saveas(fig, path);
            close(fig);
        end

        function project_root = projectRoot()
            this_file = mfilename('fullpath');
            simulation_dir = fileparts(this_file);
            matlab_root = fileparts(simulation_dir);
            project_root = fileparts(matlab_root);
        end

        function tf = recoveryIsHardOrUpupMode(cfg)
            tf = false;
            if ~isfield(cfg, 'recovery_mode'), return; end
            mode = upper(char(string(cfg.recovery_mode)));
            tf = ~isempty(strfind(mode, 'HARD_STATE_CATCH_CONTROLLER')) || ~isempty(strfind(mode, 'UPUP_CATCH_REDESIGN')) || ~isempty(strfind(mode, 'UPUP_FORCE_ROUTING_AUTOTUNE')) || ~isempty(strfind(mode, 'HARD_STATE_LIBRARY_OPTIMIZER')) || ~isempty(strfind(mode, 'STATE_SPECIFIC_GAIN_SEARCH')) || ~isempty(strfind(mode, 'REBASE_BEST50_TARGETED_REPAIR')) || ~isempty(strfind(mode, 'hard-state fallback_1N_CONSTRAINED_HARDSTATE_RECOVERY')) || ~isempty(strfind(mode, 'BALANCE_FALLBACK_1N_TWO_BRANCH_RESCUE')) || ~isempty(strfind(mode, 'RAIL_BRAKE_FALLBACK_RAIL_ONLY_PREEMPTIVE_BRAKE')) || ~isempty(strfind(mode, 'CONSTRAINED_RECOVERY_CONSTRAINED_TRAJ_TVLQR_ROA_MICROBRAKE'));
        end

        function tf = recoveryIsUpupForceAutotuneMode(cfg)
            tf = false;
            if ~isfield(cfg, 'recovery_mode'), return; end
            mode = upper(char(string(cfg.recovery_mode)));
            tf = ~isempty(strfind(mode, 'UPUP_FORCE_ROUTING_AUTOTUNE'));
        end

        function params = recoveryParams(params)
            if ~isfield(params, 'external_force') || isempty(params.external_force)
                params.external_force = struct();
            end
            params.external_force.max_external_force_N = 5.0;
            params.external_force.recovery_note = 'Recovery safe optimization_SAFE_OPT_SELECTION matrix; local LQR remains default. opt_library may compete, but selection is guarded by strict actual-simulation safety gates and pass-first policy.';
        end

        function cost = recoveryPlannerCost(m, cfg)
            gate_a = max(RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10), eps);
            gate_v = max(RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90), eps);
            gate_x = max(RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 0.85), eps);
            rail = max(RecoveryUtils.getDouble(cfg, 'global_cart_limit_m', 1.0), eps);
            success_bonus = RecoveryUtils.getDouble(cfg, 'recovery_planner_success_bonus', 2000.0);
            roa_score = RecoveryControllers.metricRoaScore(m, cfg);
            final_cost = 35.0 * (m.final_angle_error_rad / gate_a) + ...
                         10.0 * (m.final_velocity_norm / gate_v) + ...
                         10.0 * (m.final_cart_abs_m / gate_x);
            peak_cost = 4.0 * (m.max_abs_x_m / rail) + 10.0 * m.saturation_fraction;
            fail_penalty = 0.0;
            if ~m.pass_finite, fail_penalty = fail_penalty + 1.0e9; end
            if ~m.pass_force_applied, fail_penalty = fail_penalty + 1.0e6; end
            if ~m.pass_rail, fail_penalty = fail_penalty + 1.0e7; end
            if ~m.pass_recovered_final, fail_penalty = fail_penalty + 1.0e5; end
            if ~m.pass_saturation, fail_penalty = fail_penalty + 3.0e4; end
            if ~m.pass_recovery_time, fail_penalty = fail_penalty + 2.0e4; end
            cost = final_cost + peak_cost + 5.0 * roa_score + fail_penalty;
            if m.success
                rt = m.recovery_time_s;
                if ~isfinite(rt), rt = 0.0; end
                cost = cost - success_bonus + rt + 5.0 * m.saturation_fraction;
            end
        end

        function x_next = rk4Step(x, t, u, event, params, dt)
            f = @(x_local, t_local) dipDynamicsNonlinear(x_local, u, params, RecoveryEngine.externalAtTimeValue(x_local, t_local, event, params));
            k1 = f(x, t);
            k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt);
            k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt);
            k4 = f(x + dt * k3, t + dt);
            x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        end

        function u = softBoundControl(u_raw, params, cfg, frac)
            if nargin < 4 || isempty(frac), frac = RecoveryUtils.getDouble(cfg, 'recovery_terminal_soft_u_fraction', 0.88); end
            u_min = double(params.control.min_cart_force_N);
            u_max = double(params.control.max_cart_force_N);
            umax = max(abs(u_min), abs(u_max));
            limit = max(1e-6, double(frac) * umax);
            u = limit * tanh(double(u_raw) / limit);
            u = min(max(u, u_min), u_max);
        end

        function y = wrapToPi(angle_rad)
            y = atan2(sin(angle_rad), cos(angle_rad));
        end

    end
end
