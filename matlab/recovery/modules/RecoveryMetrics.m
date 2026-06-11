classdef RecoveryMetrics
    methods (Static)
        function table_out = groupSuccessByForce(case_table)
            forces = unique(case_table.force_N, 'stable');
            rows = {};
            for i = 1:numel(forces)
                mask = abs(case_table.force_N - forces(i)) < 1e-12;
                rows(i, :) = {forces(i), sum(mask), mean(case_table.success(mask)), RecoveryUtils.nanmean(case_table.recovery_time_s(mask)), RecoveryUtils.nanmean(case_table.saturation_fraction(mask))}; %#ok<AGROW>
            end
            table_out = cell2table(rows, 'VariableNames', {'force_N', 'case_count', 'success_rate', 'mean_recovery_time_s', 'mean_saturation_fraction'});
        end

        function table_out = groupSuccessByLink(case_table)
            links = unique(case_table.link_id, 'stable');
            rows = {};
            for i = 1:numel(links)
                mask = case_table.link_id == links(i);
                rows(i, :) = {links(i), sum(mask), mean(case_table.success(mask)), RecoveryUtils.nanmean(case_table.recovery_time_s(mask)), RecoveryUtils.nanmean(case_table.saturation_fraction(mask))}; %#ok<AGROW>
            end
            table_out = cell2table(rows, 'VariableNames', {'link_id', 'case_count', 'success_rate', 'mean_recovery_time_s', 'mean_saturation_fraction'});
        end

        function table_out = groupSuccessByPlanner(case_table)
            if ~ismember('planner_selected_variant', case_table.Properties.VariableNames)
                table_out = cell2table(cell(0,5), 'VariableNames', {'planner_selected_variant','case_count','success_rate','mean_recovery_time_s','mean_saturation_fraction'});
                return;
            end
            variants = unique(case_table.planner_selected_variant, 'stable');
            rows = {};
            for i = 1:numel(variants)
                v = variants{i};
                mask = strcmp(case_table.planner_selected_variant, v);
                rows(i,:) = {v, sum(mask), mean(case_table.success(mask)), RecoveryUtils.nanmean(case_table.recovery_time_s(mask)), RecoveryUtils.nanmean(case_table.saturation_fraction(mask))}; %#ok<AGROW>
            end
            table_out = cell2table(rows, 'VariableNames', {'planner_selected_variant', 'case_count', 'success_rate', 'mean_recovery_time_s', 'mean_saturation_fraction'});
        end

        function table_out = groupSuccessByTarget(case_table)
            targets = unique(case_table.target, 'stable');
            rows = {};
            for i = 1:numel(targets)
                mask = strcmp(case_table.target, targets{i});
                rows(i, :) = {targets{i}, sum(mask), mean(case_table.success(mask)), RecoveryUtils.nanmean(case_table.recovery_time_s(mask)), RecoveryUtils.nanmean(case_table.saturation_fraction(mask))}; %#ok<AGROW>
            end
            table_out = cell2table(rows, 'VariableNames', {'target', 'case_count', 'success_rate', 'mean_recovery_time_s', 'mean_saturation_fraction'});
        end

        function tf = metricsNearPass(m, cfg)
            gate_a = max(RecoveryUtils.getDouble(cfg, 'recovery_angle_rad', 0.10), eps);
            gate_v = max(RecoveryUtils.getDouble(cfg, 'recovery_velocity_norm', 0.90), eps);
            gate_x = max(RecoveryUtils.getDouble(cfg, 'recovery_cart_abs_m', 0.85), eps);
            max_sat = RecoveryUtils.getDouble(cfg, 'max_saturation_fraction', 0.70);
            tf = logical(m.pass_finite) && logical(m.pass_force_applied) && logical(m.pass_rail) && ...
                 double(m.final_angle_error_rad) <= 1.25 * gate_a && ...
                 double(m.final_velocity_norm) <= 1.25 * gate_v && ...
                 double(m.final_cart_abs_m) <= 1.10 * gate_x && ...
                 double(m.saturation_fraction) <= min(0.95, max_sat + 0.15);
        end

        function row = metricsToRow(m, case_index, case_type)
            row = {case_index, case_type, m.target, m.event_name, m.link_id, m.contact_ratio, m.direction, m.force_N, m.duration_s, ...
                   m.success, m.recovery_time_s, m.saturation_fraction, m.fail_reason, ...
                   m.final_angle_error_rad, m.final_velocity_norm, m.final_cart_abs_m, ...
                   m.max_angle_error_rad, m.max_velocity_norm, m.max_abs_x_m, m.max_abs_u_cmd_N, m.max_Q_external_norm, ...
                   m.pass_finite, m.pass_force_applied, m.pass_recovered_final, m.pass_rail, m.pass_saturation, m.pass_recovery_time, ...
                   m.x_after_force, m.xdot_after_force, m.theta1_after_force, m.theta1dot_after_force, m.theta2_after_force, m.theta2dot_after_force, ...
                   m.planner_selected_variant, m.planner_selected_cost, m.recovery_roa_score, m.recovery_margin_score, m.primary_fail_reason, ...
                   m.planner_try_opt, m.planner_opt_used_trajectory, m.planner_local_cost, m.planner_opt_cost, m.planner_decision_reason, ...
                   m.planner_force_takeover, m.planner_tried_variants, m.planner_candidate_validation_vector, ...
                   RecoveryUtils.getOptionalChar(m, 'trajectory_file', ''), RecoveryUtils.getOptionalDouble(m, 'animation_available', 0)};
        end

        function reason = primaryFailReason(fail_reason)
            if strcmp(char(fail_reason), 'PASS')
                reason = 'PASS';
                return;
            end
            parts = strsplit(char(fail_reason), '|');
            reason = parts{1};
        end

        function score = recoveryMarginScore(m)
            angle_margin = double(m.gate_angle_rad) - double(m.final_angle_error_rad);
            velocity_margin = double(m.gate_velocity_norm) - double(m.final_velocity_norm);
            cart_margin = double(m.gate_cart_abs_m) - double(m.final_cart_abs_m);
            rail_margin = double(m.global_cart_limit_m) - double(m.max_abs_x_m);
            score = min([angle_margin, 0.25 * velocity_margin, cart_margin, rail_margin]);
        end

        function rate = safeSuccessRate(tbl)
            if isempty(tbl) || height(tbl) == 0
                rate = NaN;
            else
                rate = mean(tbl.success);
            end
        end

        function note = stressLimitNote(stress_target_success_rate, stress_post_switching_success_rate)
            if isnan(stress_target_success_rate) || isnan(stress_post_switching_success_rate)
                note = '5 N stress group is empty or unavailable; check cfg.stress_force_levels_N.';
            elseif stress_target_success_rate < 0.50 || stress_post_switching_success_rate < 0.50
                note = '5 N retained as stress limit, not guaranteed recovery region.';
            else
                note = '5 N stress recovery is acceptable in this run.';
            end
        end

        function d = weightedStateDistance(x, y)
            e = double(x(:)) - double(y(:));
            e(3) = RecoveryUtils.wrapToPi(e(3));
            e(5) = RecoveryUtils.wrapToPi(e(5));
            w = [3.0; 1.0; 6.0; 1.2; 6.0; 1.2];
            d = sqrt(sum((w .* e).^2));
        end

    end
end
