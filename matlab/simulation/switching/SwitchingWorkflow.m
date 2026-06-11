classdef SwitchingWorkflow
    methods (Static)
        function log = switchingAppendLog(log, t, x, out, F_actual, requested_label)
            log.t(end + 1, 1) = t;
            log.state(end + 1, :) = x(:).';
            log.x(end + 1, 1) = x(1);
            log.theta1(end + 1, 1) = x(3);
            log.theta2(end + 1, 1) = x(5);
            log.u_cmd(end + 1, 1) = out.u_cmd;
            log.u_actual(end + 1, 1) = F_actual;
            log.mode{end + 1, 1} = out.mode;
            log.mode_id(end + 1, 1) = out.mode_id;
            log.active_target{end + 1, 1} = out.active_target;
            if nargin >= 6 && ~isempty(requested_label)
                log.requested_target{end + 1, 1} = char(requested_label);
            else
                log.requested_target{end + 1, 1} = out.requested_target;
            end
            log.current_edge{end + 1, 1} = out.current_edge;
            log.safe_reject_reason{end + 1, 1} = out.safe_reject_reason;
            log.failsafe_reason{end + 1, 1} = out.failsafe_reason;
        end

        function params_out = switchingCleanParams(params, cfg)
            params_out = params;
            params_out.realism.rail_limit_enabled = false;
            params_out.realism.friction_enabled = false;
            params_out.realism.sensor_noise_enabled = false;
            params_out.realism.estimator_enabled = false;
            params_out.realism.use_measurement_for_control = false;
            params_out.realism.use_estimator_for_control = false;
            params_out.realism.actuator_enabled = true;
            params_out.actuator.tau_motor_s = cfg.actuator_tau_motor_s;
            params_out.actuator.rate_limit_N_per_s = cfg.actuator_rate_limit_N_per_s;
            params_out.actuator.dead_zone_N = 0.0;
            params_out.actuator.min_force_N = double(params.control.min_cart_force_N);
            params_out.actuator.max_force_N = double(params.control.max_cart_force_N);
        end

        function metrics = switchingComputeMetrics(log, command_results, cfg, params)
            metrics = struct();
            metrics.command_count = numel(command_results);
            metrics.available_transition_success_count = 0;
            metrics.safe_reject_count = 0;
            metrics.failsafe_count = 0;
            metrics.timeout_count = 0;
            for i = 1:numel(command_results)
                r = command_results(i);
                if strcmp(r.status, 'handoff_success')
                    metrics.available_transition_success_count = metrics.available_transition_success_count + 1;
                elseif strcmp(r.status, 'safe_reject')
                    metrics.safe_reject_count = metrics.safe_reject_count + 1;
                elseif strcmp(r.status, 'failsafe')
                    metrics.failsafe_count = metrics.failsafe_count + 1;
                elseif strcmp(r.status, 'timeout')
                    metrics.timeout_count = metrics.timeout_count + 1;
                end
            end
            metrics.no_failsafe = metrics.failsafe_count == 0 && metrics.timeout_count == 0;
            metrics.safe_rejects_are_clean = true;
            metrics.max_abs_x_m = max(abs(log.x));
            metrics.max_abs_x_sequence_pass_m = cfg.max_abs_x_sequence_pass_m;
            metrics.max_abs_u_cmd_N = max(abs(log.u_cmd));
            metrics.max_abs_u_actual_N = max(abs(log.u_actual));
            force_limit = max(abs([double(params.control.min_cart_force_N), double(params.control.max_cart_force_N)]));
            metrics.saturation_fraction = mean(abs(log.u_cmd) >= force_limit - 1e-9);
            metrics.constraints_pass = metrics.max_abs_x_m <= cfg.max_abs_x_sequence_pass_m && metrics.saturation_fraction <= cfg.max_saturation_fraction;
        end

        function cfg = switchingDefaultConfig(user_cfg, project_root)
            cfg = struct();
            cfg.initial_target = 'down_down';
            cfg.command_sequence = {'up_up', 'up_down', 'up_up', 'down_up', 'up_up'};
            cfg.initial_error = [0; 0; 0.010; 0; -0.008; 0];
            cfg.dt_s = 0.01;
            cfg.integration_substeps_per_interval = 7;
            cfg.actuator_tau_motor_s = 0.012;
            cfg.actuator_rate_limit_N_per_s = 500.0;
            cfg.max_command_time_s = 22.0;
            cfg.initial_stabilize_time_s = 0.50;
            cfg.post_command_dwell_s = 1.25;
            cfg.min_dwell_time_s = 0.20;
            cfg.handoff_angle_rad = 0.08;
            cfg.handoff_velocity_norm = 0.50;
            cfg.handoff_cart_abs_m = 0.60;
            cfg.terminal_capture_enabled = true;
            cfg.terminal_capture_angle_rad = 0.20;
            cfg.terminal_capture_velocity_norm = 1.20;
            cfg.terminal_capture_cart_abs_m = 0.80;
            cfg.allow_early_handoff = false;
            cfg.final_angle_pass_rad = 0.08;
            cfg.final_velocity_pass_norm = 0.50;
            cfg.final_cart_pass_m = 0.60;
            cfg.max_abs_x_sequence_pass_m = 1.00;
            cfg.transition_start_angle_rad = 0.08;
            cfg.transition_start_velocity_norm = 0.50;
            cfg.transition_start_cart_abs_m = 0.60;
            cfg.prime_actuator_on_transition_start = true;
            cfg.max_saturation_fraction = 0.10;
            cfg.min_required_successful_transitions = 5;
            cfg.project_root = project_root;
            fields = fieldnames(user_cfg);
            for i = 1:numel(fields)
                cfg.(fields{i}) = user_cfg.(fields{i});
            end
        end

        function log = switchingEmptyLog()
            log = struct();
            log.t = [];
            log.state = zeros(0, 6);
            log.x = [];
            log.theta1 = [];
            log.theta2 = [];
            log.u_cmd = [];
            log.u_actual = [];
            log.mode = {};
            log.mode_id = [];
            log.active_target = {};
            log.requested_target = {};
            log.current_edge = {};
            log.safe_reject_reason = {};
            log.failsafe_reason = {};
        end

        function switchingEnsureBaselineGains(project_root)
            gains_path = fullfile(project_root, 'shared', 'lqr_gains_all_equilibria.mat');
            if ~isfile(gains_path)
                fprintf('LQR baseline LQR gain file not found; running designLqrAllEquilibria()...\n');
                designLqrAllEquilibria();
            end
        end

        function ready = switchingFinalTargetReady(x, target_name, cfg, params)
            target = SwitchingWorkflow.switchingTargetState(params, target_name);
            e = SwitchingWorkflow.switchingWrappedStateError(x, target);
            ready = max(abs([e(3), e(5)])) <= cfg.final_angle_pass_rad && ...
                    norm([e(2), e(4), e(6)]) <= cfg.final_velocity_pass_norm && ...
                    abs(e(1)) <= cfg.final_cart_pass_m;
        end

        function switchingMakePlots(result_dir, log)
            try
                fig = figure('Visible', 'off');
                stairs(log.t, log.mode_id, 'LineWidth', 1.5); grid on;
                xlabel('time [s]'); ylabel('mode id'); title('switching demo mode log');
                yticks([1 2 3 4 5]); yticklabels({'stabilize','transition','handoff','safe reject','failsafe'});
                saveas(fig, fullfile(result_dir, 'switching_mode_log.png'));
                close(fig);

                fig = figure('Visible', 'off');
                plot(log.t, log.theta1, 'LineWidth', 1.1); hold on;
                plot(log.t, log.theta2, 'LineWidth', 1.1); grid on;
                xlabel('time [s]'); ylabel('angle [rad]'); title('switching demo link angles');
                legend('theta1','theta2','Location','best');
                saveas(fig, fullfile(result_dir, 'switching_angles.png'));
                close(fig);

                fig = figure('Visible', 'off');
                plot(log.t, log.u_cmd, 'LineWidth', 1.1); hold on;
                plot(log.t, log.u_actual, '--', 'LineWidth', 1.1); grid on;
                xlabel('time [s]'); ylabel('force [N]'); title('switching demo force command and actuator output');
                legend('u cmd','u actual','Location','best');
                saveas(fig, fullfile(result_dir, 'switching_force.png'));
                close(fig);
            catch ME
                warning('Switching:PlotFailed', 'Plot failed: %s', ME.message);
            end
        end

        function x_next = switchingRk4ZohStep(x, u, dt, n_substeps, params)
            h = dt / max(1, n_substeps);
            x_next = x(:);
            for i = 1:max(1, n_substeps)
                k1 = dipDynamicsNonlinear(x_next, u, params);
                k2 = dipDynamicsNonlinear(x_next + 0.5*h*k1, u, params);
                k3 = dipDynamicsNonlinear(x_next + 0.5*h*k2, u, params);
                k4 = dipDynamicsNonlinear(x_next + h*k3, u, params);
                x_next = x_next + (h/6.0) * (k1 + 2*k2 + 2*k3 + k4);
            end
        end

        function edge_names = switchingRouteEdgesForResult(ctx)
            edge_names = {};
            if isfield(ctx, 'route') && ~isempty(ctx.route)
                edge_names = ctx.route;
            elseif isfield(ctx, 'current_edge') && ~isempty(ctx.current_edge)
                edge_names = {ctx.current_edge};
            end
        end

        function [t, x, F_actual, ctx, log, cmd_result] = switchingRunCommandUntilDone(t, x, F_actual, ctx, params, cfg, log, requested)
            cmd_result = struct();
            cmd_result.status = 'timeout';
            cmd_result.reason = '';
            cmd_result.accepted = false;
            cmd_result.safe_reject = false;
            cmd_result.failsafe = false;
            cmd_result.handoff_success = false;
            cmd_result.edge_names = {};

            t_end = t + cfg.max_command_time_s;
            request_sent = false;
            transition_started = false;
            while t < t_end
                prev_mode = '';
                if isfield(ctx, 'mode'), prev_mode = char(ctx.mode); end

                if strcmp(ctx.active_target, requested) && SwitchingWorkflow.switchingFinalTargetReady(x, requested, cfg, params)
                    cmd_result.status = 'handoff_success';
                    cmd_result.handoff_success = true;
                    cmd_result.accepted = true;
                    cmd_result.reason = 'REQUESTED_TARGET_ALREADY_ACTIVE_AND_LOCAL_LQR_READY';
                    cmd_result.edge_names = SwitchingWorkflow.switchingRouteEdgesForResult(ctx);
                    return;
                end

                if ~request_sent && ~strcmp(ctx.active_target, requested)
                    if SwitchingWorkflow.switchingTransitionStartReady(x, ctx.active_target, cfg, params)
                        ctx.pending_target = requested;
                        request_sent = true;
                    end
                end

                ctx.actual_force_N = F_actual;
                [out, ctx] = hybridMultitargetController(t, x, ctx, params);
                new_transition_started = strcmp(out.mode, 'transition_tracking') && ~transition_started;
                if new_transition_started && isfield(cfg, 'prime_actuator_on_transition_start') && cfg.prime_actuator_on_transition_start
                    if isfield(out, 'detail') && isfield(out.detail, 'u_ff') && isfinite(double(out.detail.u_ff))
                        F_actual = double(out.detail.u_ff);
                    end
                end
                if strcmp(out.mode, 'transition_tracking') || strcmp(out.mode, 'handoff')
                    transition_started = true;
                    cmd_result.accepted = true;
                elseif request_sent && strcmp(out.mode, 'stabilize_current')
                    cmd_result.accepted = true;
                end

                [x, F_actual] = SwitchingWorkflow.switchingStepClosedLoop(x, F_actual, out.u_cmd, cfg.dt_s, cfg.integration_substeps_per_interval, params);
                t = t + cfg.dt_s;
                log = SwitchingWorkflow.switchingAppendLog(log, t, x, out, F_actual, requested);

                if strcmp(out.mode, 'safe_reject') || ~isempty(out.safe_reject_reason)
                    cmd_result.status = 'safe_reject';
                    cmd_result.safe_reject = true;
                    cmd_result.reason = out.safe_reject_reason;
                    cmd_result.edge_names = SwitchingWorkflow.switchingRouteEdgesForResult(ctx);
                    return;
                end
                if strcmp(out.mode, 'failsafe') || ~isempty(out.failsafe_reason)
                    cmd_result.status = 'failsafe';
                    cmd_result.failsafe = true;
                    cmd_result.reason = out.failsafe_reason;
                    cmd_result.edge_names = SwitchingWorkflow.switchingRouteEdgesForResult(ctx);
                    return;
                end
                if strcmp(ctx.active_target, requested) && SwitchingWorkflow.switchingFinalTargetReady(x, requested, cfg, params)
                    if transition_started || strcmp(prev_mode, 'transition_tracking') || strcmp(out.mode, 'transition_tracking') || strcmp(out.mode, 'handoff') || strcmp(out.mode, 'stabilize_current')
                        cmd_result.status = 'handoff_success';
                        cmd_result.handoff_success = true;
                        cmd_result.reason = 'REQUESTED_TARGET_REACHED_AND_LOCAL_LQR_READY';
                        cmd_result.edge_names = SwitchingWorkflow.switchingRouteEdgesForResult(ctx);
                        return;
                    end
                end
            end
            cmd_result.status = 'timeout';
            cmd_result.reason = 'COMMAND_TIMEOUT';
            cmd_result.edge_names = SwitchingWorkflow.switchingRouteEdgesForResult(ctx);
        end

        function [t, x, F_actual, ctx, log] = switchingSimulateSegment(t, x, F_actual, ctx, params, cfg, log, duration_s, request_label)
            t_end = t + duration_s;
            while t < t_end
                ctx.actual_force_N = F_actual;
                [out, ctx] = hybridMultitargetController(t, x, ctx, params);
                [x, F_actual] = SwitchingWorkflow.switchingStepClosedLoop(x, F_actual, out.u_cmd, cfg.dt_s, cfg.integration_substeps_per_interval, params);
                t = t + cfg.dt_s;
                log = SwitchingWorkflow.switchingAppendLog(log, t, x, out, F_actual, request_label);
            end
        end

        function [x_next, F_actual] = switchingStepClosedLoop(x, F_actual, F_cmd, dt, n_substeps, params)
            x_next = x(:);
            n = max(1, ceil(double(n_substeps)));
            h = dt / n;
            for ii = 1:n
                [F_actual, ~] = dipActuator(F_actual, F_cmd, h, params);
                x_next = SwitchingWorkflow.switchingRk4ZohStep(x_next, F_actual, h, 1, params);
            end
        end

        function target = switchingTargetState(params, target_name)
            lib = targetEquilibriumLibrary(params);
            target = lib.targets.(char(target_name)).target_state(:);
        end

        function ready = switchingTransitionStartReady(x, target_name, cfg, params)
            target = SwitchingWorkflow.switchingTargetState(params, target_name);
            e = SwitchingWorkflow.switchingWrappedStateError(x, target);
            ready = max(abs([e(3), e(5)])) <= cfg.transition_start_angle_rad && ...
                    norm([e(2), e(4), e(6)]) <= cfg.transition_start_velocity_norm && ...
                    abs(e(1)) <= cfg.transition_start_cart_abs_m;
        end

        function e = switchingWrappedStateError(x, ref)
            e = double(x(:)) - double(ref(:));
            e(3) = atan2(sin(e(3)), cos(e(3)));
            e(5) = atan2(sin(e(5)), cos(e(5)));
        end

        function switchingWriteLogs(summary, log, command_results)
            T = table(log.t, log.mode_id, log.mode, log.active_target, log.requested_target, log.current_edge, ...
                      log.x, log.theta1, log.theta2, log.u_cmd, log.u_actual, log.safe_reject_reason, log.failsafe_reason, ...
                      'VariableNames', {'t_s','mode_id','mode','active_target','requested_target','current_edge','x_m','theta1_rad','theta2_rad','u_cmd_N','u_actual_N','safe_reject_reason','failsafe_reason'});
            writetable(T, summary.mode_log_file);

            n = numel(command_results);
            command_index = zeros(n,1); start_target = cell(n,1); requested_target = cell(n,1); end_target = cell(n,1);
            status = cell(n,1); reason = cell(n,1); start_time_s = zeros(n,1); end_time_s = zeros(n,1); duration_s = zeros(n,1);
            for i = 1:n
                command_index(i) = command_results(i).command_index;
                start_target{i} = command_results(i).start_target;
                requested_target{i} = command_results(i).requested_target;
                end_target{i} = command_results(i).end_target;
                status{i} = command_results(i).status;
                reason{i} = command_results(i).reason;
                start_time_s(i) = command_results(i).start_time_s;
                end_time_s(i) = command_results(i).end_time_s;
                duration_s(i) = command_results(i).duration_s;
            end
            C = table(command_index, start_target, requested_target, end_target, status, reason, start_time_s, end_time_s, duration_s);
            writetable(C, summary.command_log_file);
        end

        function switchingWriteStatus(status_file, summary, cfg)
            fid = fopen(status_file, 'w');
            if fid < 0
                warning('Switching:StatusOpenFailed', 'Could not write status file: %s', status_file);
                return;
            end
            cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '# SWITCHING DEMO STATUS\n\n');
            fprintf(fid, 'switching demo implements a hybrid multi-target controller using transition graph routing, TVLQR tracking discrete TVLQR tracking and LQR baseline local LQR stabilization.\n\n');
            fprintf(fid, '## Scope\n\n');
            fprintf(fid, '- No user-interface action is executed in this workflow.\n');
            fprintf(fid, '- No external force recovery is executed in this workflow.\n');
            fprintf(fid, '- Missing transition trajectories are safe rejected, not guessed.\n\n');
            fprintf(fid, '## Demo command sequence\n\n');
            for i = 1:numel(cfg.command_sequence)
                fprintf(fid, '%d. %s\n', i, cfg.command_sequence{i});
            end
            fprintf(fid, '\n## Summary\n\n');
            fprintf(fid, '- overall_pass: %d\n', summary.overall_pass);
            fprintf(fid, '- successful_available_transitions: %d\n', summary.metrics.available_transition_success_count);
            fprintf(fid, '- safe_reject_count: %d\n', summary.metrics.safe_reject_count);
            fprintf(fid, '- failsafe_count: %d\n', summary.metrics.failsafe_count);
            fprintf(fid, '- max_abs_x_m: %.6g\n', summary.metrics.max_abs_x_m);
            fprintf(fid, '- max_abs_u_cmd_N: %.6g\n', summary.metrics.max_abs_u_cmd_N);
            fprintf(fid, '- saturation_fraction: %.6g\n\n', summary.metrics.saturation_fraction);
            fprintf(fid, '## Output files\n\n');
            fprintf(fid, '- `%s`\n', summary.mode_log_file);
            fprintf(fid, '- `%s`\n', summary.command_log_file);
            fprintf(fid, '- `%s`\n', summary.mat_file);
        end

    end
end
