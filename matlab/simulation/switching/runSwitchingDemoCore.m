function summary = runSwitchingDemoCore(user_cfg)

    if nargin < 1 || isempty(user_cfg)
        user_cfg = struct();
    end

    this_file = mfilename('fullpath');
    switching_dir = fileparts(this_file);
    simulation_dir = fileparts(switching_dir);
    matlab_root = fileparts(simulation_dir);
    project_root = fileparts(matlab_root);
    run(fullfile(matlab_root, 'startupProject.m'));

    cfg = SwitchingWorkflow.switchingDefaultConfig(user_cfg, project_root);
    params = SwitchingWorkflow.switchingCleanParams(loadParams(), cfg);
    SwitchingWorkflow.switchingEnsureBaselineGains(project_root);

    result_dir = fullfile(project_root, 'results', 'switching');
    if ~exist(result_dir, 'dir'), mkdir(result_dir); end
    status_file = fullfile(result_dir, 'switching_status.txt');
    if ~exist(fileparts(status_file), 'dir'), mkdir(fileparts(status_file)); end

    fprintf('switching demo - hybrid multi-target controller demo\n');
    fprintf('state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
    fprintf('angle_convention = theta = 0 down, theta = pi up, theta2 absolute\n');
    fprintf('controller_stack = transition graph + TVLQR tracking TVLQR + LQR baseline local LQR\n');
    fprintf('No external force recovery is executed in switching demo.\n\n');

    ctx = struct();
    ctx.active_target = cfg.initial_target;
    ctx.config = struct();
    ctx.config.trajectory_dir = fullfile(project_root, 'shared', 'trajectories');
    ctx.config.tvlqr_gain_dir = fullfile(project_root, 'shared', 'tvlqr_gains');
    ctx.config.min_dwell_time_s = cfg.min_dwell_time_s;
    ctx.config.handoff_angle_rad = cfg.handoff_angle_rad;
    ctx.config.handoff_velocity_norm = cfg.handoff_velocity_norm;
    ctx.config.handoff_cart_abs_m = cfg.handoff_cart_abs_m;
    ctx.config.terminal_capture_enabled = cfg.terminal_capture_enabled;
    ctx.config.terminal_capture_angle_rad = cfg.terminal_capture_angle_rad;
    ctx.config.terminal_capture_velocity_norm = cfg.terminal_capture_velocity_norm;
    ctx.config.terminal_capture_cart_abs_m = cfg.terminal_capture_cart_abs_m;
    ctx.config.allow_early_handoff = cfg.allow_early_handoff;
    ctx.config.integration_substeps_per_interval = cfg.integration_substeps_per_interval;

    x = SwitchingWorkflow.switchingTargetState(params, cfg.initial_target) + cfg.initial_error(:);
    F_actual = 0.0;
    t = 0.0;
    log = SwitchingWorkflow.switchingEmptyLog();
    command_results = [];

    [t, x, F_actual, ctx, log] = SwitchingWorkflow.switchingSimulateSegment(t, x, F_actual, ctx, params, cfg, log, cfg.initial_stabilize_time_s, 'initial_stabilize');

    for i = 1:numel(cfg.command_sequence)
        requested = cfg.command_sequence{i};
        fprintf('\n=== switching demo command %d/%d: %s -> %s ===\n', i, numel(cfg.command_sequence), ctx.active_target, requested);
        cmd_start_t = t;
        start_target = ctx.active_target;
        start_state = x;
        [t, x, F_actual, ctx, log, cmd_result] = SwitchingWorkflow.switchingRunCommandUntilDone(t, x, F_actual, ctx, params, cfg, log, requested);
        cmd_result.command_index = i;
        cmd_result.requested_target = requested;
        cmd_result.start_target = start_target;
        cmd_result.end_target = ctx.active_target;
        cmd_result.start_time_s = cmd_start_t;
        cmd_result.end_time_s = t;
        cmd_result.duration_s = t - cmd_start_t;
        cmd_result.start_state = start_state(:);
        cmd_result.end_state = x(:);
        if isempty(command_results)
            command_results = cmd_result;
        else
            command_results(end + 1) = cmd_result; %#ok<AGROW>
        end
        fprintf('command_result = %s, active_target = %s, reason = %s\n', cmd_result.status, ctx.active_target, cmd_result.reason);
        [t, x, F_actual, ctx, log] = SwitchingWorkflow.switchingSimulateSegment(t, x, F_actual, ctx, params, cfg, log, cfg.post_command_dwell_s, 'post_command_dwell');
    end

    metrics = SwitchingWorkflow.switchingComputeMetrics(log, command_results, cfg, params);
    overall_pass = metrics.no_failsafe && metrics.constraints_pass && ...
                   metrics.available_transition_success_count >= cfg.min_required_successful_transitions && ...
                   metrics.safe_reject_count == 0 && metrics.safe_rejects_are_clean;

    summary = struct();
    summary.workflow = 'switching';
    summary.overall_pass = overall_pass;
    summary.metrics = metrics;
    summary.command_results = command_results;
    summary.result_dir = result_dir;
    summary.status_file = status_file;
    summary.mode_log_file = fullfile(result_dir, 'switching_mode_log.csv');
    summary.command_log_file = fullfile(result_dir, 'switching_command_log.csv');
    summary.mat_file = fullfile(result_dir, 'switchingResults.mat');

    SwitchingWorkflow.switchingWriteLogs(summary, log, command_results);
    SwitchingWorkflow.switchingMakePlots(result_dir, log);
    SwitchingWorkflow.switchingWriteStatus(status_file, summary, cfg);
    save(summary.mat_file, 'summary', 'log', 'command_results', 'cfg');

    fprintf('\nswitching demo summary pass = %d\n', overall_pass);
    fprintf('successful_available_transitions = %d\n', metrics.available_transition_success_count);
    fprintf('safe_reject_count = %d\n', metrics.safe_reject_count);
    fprintf('failsafe_count = %d\n', metrics.failsafe_count);
    fprintf('Mode log CSV: %s\n', summary.mode_log_file);
    fprintf('Command log CSV: %s\n', summary.command_log_file);
    fprintf('Status file: %s\n', status_file);
    if overall_pass
        fprintf('switching demo runSwitchingDemo: PASS_FULL_SWITCHING_SEQUENCE\n');
    else
        fprintf('switching demo runSwitchingDemo: NOT_PASS_REVIEW_REQUIRED\n');
    end
end
