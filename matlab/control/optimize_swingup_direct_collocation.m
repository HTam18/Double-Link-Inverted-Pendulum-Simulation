function opt_result = optimize_swingup_direct_collocation(user_cfg)
% OPTIMIZE_SWINGUP_DIRECT_COLLOCATION Phase 28 constrained swing-up optimizer.
%
% Purpose:
%   Generate an optimized up_up swing-up trajectory for the nonlinear double
%   link pendulum on cart using direct collocation and fmincon.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 rad downward, theta = pi rad upright.
%
% Decision variables:
%   z = [X(:); U(:)]
%   X is 6 x N state trajectory.
%   U is 1 x (N - 1) feedforward cart force trajectory.
%
% Collocation method:
%   Trapezoidal direct collocation with zero-order-hold force per segment.
%
% Output:
%   shared/trajectories/up_up_swingup_optimized.mat
%   results/phase28_direct_collocation/direct_collocation_summary.txt
%   results/phase28_direct_collocation/direct_collocation_convergence.mat
%
% Usage:
%   run('matlab/startup_project.m')
%   opt_result = optimize_swingup_direct_collocation();
%
% Senior note:
%   Direct collocation for a double-link swing-up is sensitive to initial
%   guess and constraints. This first Phase 28 implementation uses a fixed
%   final time and moderate terminal constraints so that the optimizer has a
%   practical chance to find a usable trajectory. Later phases can improve it
%   with variable final time, mesh refinement and TVLQR tracking.

    if nargin < 1 || isempty(user_cfg)
        user_cfg = struct();
    end

    this_file = mfilename('fullpath');
    control_dir = fileparts(this_file);
    matlab_root = fileparts(control_dir);
    project_root = fileparts(matlab_root);

    if exist(fullfile(matlab_root, 'startup_project.m'), 'file')
        run(fullfile(matlab_root, 'startup_project.m'));
    end

    params = load_params();
    cfg = local_default_cfg(user_cfg, params, project_root);
    local_prepare_dirs(cfg);

    fprintf('Phase 28 direct collocation swing-up optimization\n');
    fprintf('state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
    fprintf('angle_convention = theta = 0 downward, theta = pi upright\n');
    fprintf('target_mode = up_up\n');
    fprintf('N = %d\n', cfg.N);
    fprintf('T = %.6g s\n', cfg.T);
    fprintf('dt = %.6g s\n', cfg.dt);
    fprintf('force_limit = [%.6g, %.6g] N\n', cfg.u_min, cfg.u_max);
    fprintf('rail_limit = [%.6g, %.6g] m\n', cfg.x_min, cfg.x_max);
    fprintf('defect_model = RK4 zero_order_hold with %d substeps per interval\n', cfg.integration_substeps_per_interval);

    if exist('fmincon', 'file') ~= 2
        error('Phase28:MissingFmincon', ...
              ['fmincon was not found. Install/enable MATLAB Optimization Toolbox ', ...
               'or run Phase 28 on a MATLAB installation that includes fmincon.']);
    end

    [z0, bounds] = local_make_initial_guess_and_bounds(cfg, params);
    [z0, checkpoint_resume_used] = local_apply_optimizer_checkpoint(z0, bounds, cfg);
    fprintf('optimizer_checkpoint_resume_used = %d\n', checkpoint_resume_used);

    obj_fun = @(z) local_objective(z, cfg);
    nonlcon = @(z) local_constraints(z, cfg, params);

    history = struct();
    history.iteration = [];
    history.fval = [];
    history.firstorderopt = [];
    history.constrviolation = [];
    history.stepsize = [];

    options = optimoptions('fmincon', ...
        'Algorithm', cfg.algorithm, ...
        'Display', cfg.display, ...
        'MaxIterations', cfg.max_iterations, ...
        'MaxFunctionEvaluations', cfg.max_function_evaluations, ...
        'ConstraintTolerance', cfg.constraint_tolerance, ...
        'OptimalityTolerance', cfg.optimality_tolerance, ...
        'StepTolerance', cfg.step_tolerance, ...
        'SpecifyObjectiveGradient', false, ...
        'SpecifyConstraintGradient', false, ...
        'OutputFcn', @(x, optimValues, state) local_output_function(x, optimValues, state, cfg));

    problem = struct();
    problem.objective = obj_fun;
    problem.x0 = z0;
    problem.lb = bounds.lb;
    problem.ub = bounds.ub;
    problem.nonlcon = nonlcon;
    problem.solver = 'fmincon';
    problem.options = options;

    [z_opt, fval, exitflag, output] = fmincon(problem);
    history = local_load_optimizer_history(cfg, history);
    [X_opt, U_opt] = local_unpack(z_opt, cfg);

    [c_final, ceq_final] = local_constraints(z_opt, cfg, params);
    max_collocation_defect = max(abs(ceq_final(:)));
    max_ineq_violation = max([0; c_final(:)]);

    metrics = local_compute_metrics(X_opt, U_opt, cfg, max_collocation_defect, max_ineq_violation, exitflag);

    trajectory = struct();
    trajectory.phase = 28;
    trajectory.method = 'direct_collocation_rk4_zoh_fmincon';
    trajectory.target_mode = 'up_up';
    trajectory.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    trajectory.angle_convention = 'theta = 0 downward, theta = pi upright';
    trajectory.t = cfg.t(:);
    trajectory.x_ref = X_opt.';
    trajectory.u_time = cfg.t(1:end-1).';
    trajectory.u_ff = U_opt(:);
    trajectory.final_time_s = cfg.T;
    trajectory.dt_s = cfg.dt;
    trajectory.config = cfg;
    trajectory.metrics = metrics;
    trajectory.fmincon_exitflag = exitflag;
    trajectory.fmincon_output = output;
    trajectory.fmincon_fval = fval;

    opt_result = struct();
    opt_result.trajectory = trajectory;
    opt_result.metrics = metrics;
    opt_result.exitflag = exitflag;
    opt_result.output = output;
    opt_result.fval = fval;
    opt_result.history = history;
    opt_result.pass = metrics.optimization_usable;

    save(cfg.trajectory_file, 'trajectory', 'opt_result');
    save(cfg.convergence_file, 'history', 'metrics', 'exitflag', 'output', 'fval');
    local_write_summary(cfg.summary_file, cfg, metrics, exitflag, output, fval);
    local_write_status_doc(cfg.status_file, cfg, metrics, exitflag, output);
    local_plot_trajectory(cfg, trajectory);

    fprintf('optimization_exitflag = %d\n', exitflag);
    fprintf('optimization_usable = %d\n', metrics.optimization_usable);
    fprintf('max_collocation_defect = %.12g\n', metrics.max_collocation_defect);
    fprintf('max_ineq_violation = %.12g\n', metrics.max_ineq_violation);
    fprintf('final_max_angle_error_rad = %.12g\n', metrics.final_max_angle_error_rad);
    fprintf('final_velocity_norm = %.12g\n', metrics.final_velocity_norm);
    fprintf('max_abs_x_m = %.12g\n', metrics.max_abs_x_m);
    fprintf('max_abs_u_N = %.12g\n', metrics.max_abs_u_N);
    fprintf('trajectory_file = %s\n', cfg.trajectory_file);
end

function stop = local_output_function(x_current, optimValues, state, cfg)
    stop = false;
    persistent local_history
    switch state
        case 'init'
            local_history = struct('iteration', [], 'fval', [], 'firstorderopt', [], ...
                                   'constrviolation', [], 'stepsize', []);
            local_save_optimizer_checkpoint(x_current, optimValues, state, local_history, cfg);
        case 'iter'
            local_history.iteration(end + 1, 1) = optimValues.iteration;
            local_history.fval(end + 1, 1) = optimValues.fval;
            if isfield(optimValues, 'firstorderopt')
                local_history.firstorderopt(end + 1, 1) = optimValues.firstorderopt;
            else
                local_history.firstorderopt(end + 1, 1) = NaN;
            end
            if isfield(optimValues, 'constrviolation')
                local_history.constrviolation(end + 1, 1) = optimValues.constrviolation;
            else
                local_history.constrviolation(end + 1, 1) = NaN;
            end
            if isfield(optimValues, 'stepsize')
                local_history.stepsize(end + 1, 1) = optimValues.stepsize;
            else
                local_history.stepsize(end + 1, 1) = NaN;
            end
            local_save_optimizer_checkpoint(x_current, optimValues, state, local_history, cfg);
        case 'done'
            % Do not use assignin('caller',...) here. When this optimizer is
            % called through a script executed by run(), MATLAB may execute the
            % output function in a static workspace. assignin then fails with:
            %   Attempt to add "history" to a static workspace.
            % The persistent local_history is saved to optimizer_checkpoint.mat
            % instead, and the main function reloads it after fmincon returns.
            local_save_optimizer_checkpoint(x_current, optimValues, state, local_history, cfg);
    end
end


function history = local_load_optimizer_history(cfg, fallback_history)
    history = fallback_history;
    if ~isfield(cfg, 'checkpoint_file') || isempty(cfg.checkpoint_file)
        return;
    end
    if exist(cfg.checkpoint_file, 'file') ~= 2
        return;
    end
    try
        data = load(cfg.checkpoint_file, 'checkpoint_history');
        if isfield(data, 'checkpoint_history') && isstruct(data.checkpoint_history)
            history = data.checkpoint_history;
        end
    catch load_error
        warning('Phase28:CheckpointHistoryLoadFailed', ...
                'Could not load optimizer checkpoint history %s: %s', cfg.checkpoint_file, load_error.message);
    end
end

function [z0, resumed] = local_apply_optimizer_checkpoint(z0, bounds, cfg)
    resumed = false;
    if ~isfield(cfg, 'resume_from_checkpoint') || ~cfg.resume_from_checkpoint
        return;
    end
    if ~isfield(cfg, 'checkpoint_file') || isempty(cfg.checkpoint_file)
        return;
    end
    if exist(cfg.checkpoint_file, 'file') ~= 2
        return;
    end
    try
        data = load(cfg.checkpoint_file);
    catch load_error
        warning('Phase28:CheckpointLoadFailed', ...
                'Could not load optimizer checkpoint %s: %s', cfg.checkpoint_file, load_error.message);
        return;
    end
    if ~isfield(data, 'x_checkpoint')
        return;
    end
    x_checkpoint = double(data.x_checkpoint(:));
    if numel(x_checkpoint) ~= numel(z0)
        warning('Phase28:CheckpointSizeMismatch', ...
                'Ignoring checkpoint because length is %d, expected %d.', numel(x_checkpoint), numel(z0));
        return;
    end
    if any(~isfinite(x_checkpoint))
        warning('Phase28:CheckpointNonFinite', 'Ignoring checkpoint because it contains NaN or Inf.');
        return;
    end
    % Keep the resumed point inside simple bounds. Nonlinear feasibility is
    % handled by fmincon; this only prevents a stale or partially corrupted
    % checkpoint from immediately failing bound checks.
    x_checkpoint = min(max(x_checkpoint, bounds.lb), bounds.ub);
    z0 = x_checkpoint;
    resumed = true;
    fprintf('optimizer_checkpoint_file = %s\n', cfg.checkpoint_file);
    if isfield(data, 'checkpoint_iteration')
        fprintf('optimizer_checkpoint_iteration = %d\n', data.checkpoint_iteration);
    end
    if isfield(data, 'checkpoint_fval')
        fprintf('optimizer_checkpoint_fval = %.12g\n', data.checkpoint_fval);
    end
end

function local_save_optimizer_checkpoint(x_current, optimValues, state, local_history, cfg)
    if ~isfield(cfg, 'checkpoint_file') || isempty(cfg.checkpoint_file)
        return;
    end
    if isempty(x_current)
        return;
    end
    try
        checkpoint_dir = fileparts(cfg.checkpoint_file);
        if ~exist(checkpoint_dir, 'dir')
            mkdir(checkpoint_dir);
        end
        x_checkpoint = double(x_current(:)); %#ok<NASGU>
        checkpoint_state = state; %#ok<NASGU>
        checkpoint_history = local_history; %#ok<NASGU>
        checkpoint_time = char(datetime('now')); %#ok<NASGU>
        checkpoint_meta = struct(); %#ok<NASGU>
        checkpoint_meta.N = cfg.N;
        checkpoint_meta.T = cfg.T;
        checkpoint_meta.dt = cfg.dt;
        checkpoint_meta.u_min = cfg.u_min;
        checkpoint_meta.u_max = cfg.u_max;
        if isfield(cfg, 'candidate_name')
            checkpoint_meta.candidate_name = cfg.candidate_name;
        else
            checkpoint_meta.candidate_name = '';
        end
        if isfield(optimValues, 'iteration')
            checkpoint_iteration = optimValues.iteration; %#ok<NASGU>
        else
            checkpoint_iteration = -1; %#ok<NASGU>
        end
        if isfield(optimValues, 'fval')
            checkpoint_fval = optimValues.fval; %#ok<NASGU>
        else
            checkpoint_fval = NaN; %#ok<NASGU>
        end
        save(cfg.checkpoint_file, 'x_checkpoint', 'checkpoint_state', ...
             'checkpoint_iteration', 'checkpoint_fval', 'checkpoint_history', ...
             'checkpoint_time', 'checkpoint_meta');
    catch save_error
        warning('Phase28:CheckpointSaveFailed', ...
                'Could not save optimizer checkpoint %s: %s', cfg.checkpoint_file, save_error.message);
    end
end

function cfg = local_default_cfg(user_cfg, params, project_root)
    cfg = struct();
    cfg.N = 61;
    cfg.T = 8.0;
    cfg.dt = cfg.T / (cfg.N - 1);
    cfg.t = linspace(0.0, cfg.T, cfg.N).';
    cfg.target_mode = 'up_up';

    cfg.x0 = [0; 0; 0; 0; 0; 0];
    cfg.xf = [0; 0; pi; 0; pi; 0];

    cfg.x_min = local_get_field(params.rail_limit, 'x_min_m', -2.0);
    cfg.x_max = local_get_field(params.rail_limit, 'x_max_m', 2.0);
    cfg.x_margin = 0.05;
    cfg.u_min = local_get_field(params.control, 'min_cart_force_N', -15.0);
    cfg.u_max = local_get_field(params.control, 'max_cart_force_N', 15.0);

    cfg.x_dot_limit = 4.0;
    cfg.theta_dot_limit = 12.0;
    cfg.terminal_angle_tolerance_rad = 0.25;
    cfg.terminal_velocity_norm_limit = 2.5;
    cfg.terminal_cart_abs_limit_m = 0.75;
    cfg.usable_angle_tolerance_rad = 0.35;
    cfg.usable_velocity_norm_limit = 4.0;
    cfg.max_collocation_defect_for_pass = 5.0e-3;
    cfg.max_ineq_violation_for_pass = 5.0e-4;

    cfg.w_terminal_angle = 1.5e4;
    cfg.w_terminal_velocity = 8.0e2;
    cfg.w_terminal_cart = 4.0e2;
    cfg.w_force = 1.0e-2;
    cfg.w_force_smooth = 5.0e-3;
    cfg.w_cart_center = 2.0e0;
    cfg.w_velocity = 1.0e-2;

    cfg.algorithm = 'sqp';
    cfg.display = 'iter';
    cfg.max_iterations = 350;
    cfg.max_function_evaluations = 250000;
    cfg.constraint_tolerance = 1.0e-5;
    cfg.optimality_tolerance = 1.0e-4;
    cfg.step_tolerance = 1.0e-10;
    cfg.integration_substeps_per_interval = max(1, ceil(cfg.dt / 0.02));

    cfg.result_dir = fullfile(project_root, 'results', 'phase28_direct_collocation');
    cfg.trajectory_dir = fullfile(project_root, 'shared', 'trajectories');
    cfg.trajectory_file = fullfile(cfg.trajectory_dir, 'up_up_swingup_optimized.mat');
    cfg.summary_file = fullfile(cfg.result_dir, 'direct_collocation_summary.txt');
    cfg.convergence_file = fullfile(cfg.result_dir, 'direct_collocation_convergence.mat');
    cfg.trajectory_plot_file = fullfile(cfg.result_dir, 'optimized_trajectory.png');
    cfg.status_file = fullfile(project_root, 'docs', 'PROJECT_STATUS_PHASE28.md');
    cfg.checkpoint_file = fullfile(cfg.result_dir, 'optimizer_checkpoint.mat');
    cfg.resume_from_checkpoint = true;
    cfg.candidate_name = 'phase28_default';

    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
    cfg.dt = cfg.T / (cfg.N - 1);
    cfg.t = linspace(0.0, cfg.T, cfg.N).';
    if ~isfield(cfg, 'integration_substeps_per_interval') || isempty(cfg.integration_substeps_per_interval)
        cfg.integration_substeps_per_interval = max(1, ceil(cfg.dt / 0.02));
    end
end

function local_prepare_dirs(cfg)
    dirs = {cfg.result_dir, cfg.trajectory_dir, fileparts(cfg.status_file)};
    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end
end

function value = local_get_field(s, name, default_value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = double(s.(name));
    else
        value = default_value;
    end
end

function [z0, bounds] = local_make_initial_guess_and_bounds(cfg, params)
    N = cfg.N;
    X0 = zeros(6, N);

    % Initial guess: smooth interpolation from down_down to up_up. This is not
    % dynamically exact, but gives fmincon a feasible direction for terminal
    % objective and bounds. Small velocity waves help avoid a completely static
    % infeasible guess.
    s = linspace(0.0, 1.0, N);
    smooth_s = 3.0 * s.^2 - 2.0 * s.^3;
    X0(1, :) = 0.25 * sin(2.0 * pi * s);
    X0(2, :) = gradient(X0(1, :), cfg.dt);
    X0(3, :) = pi * smooth_s;
    X0(4, :) = gradient(X0(3, :), cfg.dt);
    X0(5, :) = pi * smooth_s;
    X0(6, :) = gradient(X0(5, :), cfg.dt);
    X0(:, 1) = cfg.x0;
    X0(:, end) = cfg.xf;

    U0 = local_initial_force_guess(cfg, params);

    z0 = [X0(:); U0(:)];

    lbX = -inf(6, N);
    ubX = inf(6, N);
    lbX(1, :) = cfg.x_min + cfg.x_margin;
    ubX(1, :) = cfg.x_max - cfg.x_margin;
    lbX(2, :) = -cfg.x_dot_limit;
    ubX(2, :) = cfg.x_dot_limit;
    lbX(3, :) = -2.0 * pi;
    ubX(3, :) = 2.0 * pi;
    lbX(4, :) = -cfg.theta_dot_limit;
    ubX(4, :) = cfg.theta_dot_limit;
    lbX(5, :) = -2.0 * pi;
    ubX(5, :) = 2.0 * pi;
    lbX(6, :) = -cfg.theta_dot_limit;
    ubX(6, :) = cfg.theta_dot_limit;

    % Initial state is fixed by bounds and by equality constraints.
    lbX(:, 1) = cfg.x0;
    ubX(:, 1) = cfg.x0;

    lbU = cfg.u_min * ones(N - 1, 1);
    ubU = cfg.u_max * ones(N - 1, 1);

    bounds.lb = [lbX(:); lbU(:)];
    bounds.ub = [ubX(:); ubU(:)];
end

function U0 = local_initial_force_guess(cfg, params)
    if isfield(params, 'energy_swingup') && isfield(params.energy_swingup, 'force_sequence_N')
        seq = double(params.energy_swingup.force_sequence_N(:));
        if ~isempty(seq)
            old_t = linspace(0.0, cfg.T, numel(seq));
            new_t = cfg.t(1:end-1);
            U0 = interp1(old_t, seq, new_t, 'previous', 'extrap');
            U0 = U0(:);
        else
            U0 = zeros(cfg.N - 1, 1);
        end
    else
        U0 = zeros(cfg.N - 1, 1);
    end
    U0 = min(max(U0, cfg.u_min), cfg.u_max);
end

function [X, U] = local_unpack(z, cfg)
    nX = 6 * cfg.N;
    X = reshape(z(1:nX), 6, cfg.N);
    U = z(nX + 1:end);
end

function J = local_objective(z, cfg)
    [X, U] = local_unpack(z, cfg);
    dt = cfg.dt;

    e_final = X(:, end) - cfg.xf;
    e_final(3) = local_wrap_to_pi(X(3, end) - cfg.xf(3));
    e_final(5) = local_wrap_to_pi(X(5, end) - cfg.xf(5));

    terminal_angle_cost = cfg.w_terminal_angle * (e_final(3)^2 + e_final(5)^2);
    terminal_velocity_cost = cfg.w_terminal_velocity * (e_final(2)^2 + e_final(4)^2 + e_final(6)^2);
    terminal_cart_cost = cfg.w_terminal_cart * (e_final(1)^2);
    force_cost = cfg.w_force * dt * sum(U.^2);
    force_smooth_cost = cfg.w_force_smooth * sum(diff(U).^2);
    cart_center_cost = cfg.w_cart_center * dt * sum(X(1, :).^2);
    velocity_cost = cfg.w_velocity * dt * sum(X(2, :).^2 + X(4, :).^2 + X(6, :).^2);

    J = terminal_angle_cost + terminal_velocity_cost + terminal_cart_cost + ...
        force_cost + force_smooth_cost + cart_center_cost + velocity_cost;
end

function [c, ceq] = local_constraints(z, cfg, params)
    [X, U] = local_unpack(z, cfg);
    N = cfg.N;
    dt = cfg.dt;

    ceq = zeros(6 * (N - 1) + 6, 1);
    ceq(1:6) = X(:, 1) - cfg.x0;
    row = 7;
    for k = 1:(N - 1)
        xk = X(:, k);
        xkp1 = X(:, k + 1);
        uk = U(k);

        % Important Phase 28 hotfix:
        % Use the same zero-order-hold RK4 propagation style as run_simulation.
        % The previous trapezoidal defect could be mathematically feasible on
        % the NLP mesh but replay poorly on the nonlinear plant.
        x_next_pred = local_rk4_zoh_step(xk, uk, dt, cfg.integration_substeps_per_interval, params);
        defect = xkp1 - x_next_pred;
        ceq(row:(row + 5)) = defect;
        row = row + 6;
    end

    final_angle_err_1 = abs(local_wrap_to_pi(X(3, end) - pi));
    final_angle_err_2 = abs(local_wrap_to_pi(X(5, end) - pi));
    final_velocity_norm = norm([X(2, end), X(4, end), X(6, end)]);
    final_cart_abs = abs(X(1, end));

    c = [final_angle_err_1 - cfg.terminal_angle_tolerance_rad; ...
         final_angle_err_2 - cfg.terminal_angle_tolerance_rad; ...
         final_velocity_norm - cfg.terminal_velocity_norm_limit; ...
         final_cart_abs - cfg.terminal_cart_abs_limit_m];
end


function x_next = local_rk4_zoh_step(x, u, dt, n_substeps, params)
    x_next = x(:);
    h = dt / double(n_substeps);
    for ii = 1:n_substeps
        f = @(xx) dip_dynamics_nonlinear(xx, u, params);
        k1 = f(x_next);
        k2 = f(x_next + 0.5 * h * k1);
        k3 = f(x_next + 0.5 * h * k2);
        k4 = f(x_next + h * k3);
        x_next = x_next + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
    end
end

function metrics = local_compute_metrics(X, U, cfg, max_defect, max_ineq, exitflag)
    final_angle_err_1 = abs(local_wrap_to_pi(X(3, end) - pi));
    final_angle_err_2 = abs(local_wrap_to_pi(X(5, end) - pi));
    final_max_angle_error = max([final_angle_err_1, final_angle_err_2]);
    final_velocity_norm = norm([X(2, end), X(4, end), X(6, end)]);
    max_abs_x = max(abs(X(1, :)));
    max_abs_u = max(abs(U(:)));

    rail_pass = max(X(1, :)) <= cfg.x_max + 1.0e-9 && min(X(1, :)) >= cfg.x_min - 1.0e-9;
    force_pass = max_abs_u <= max(abs([cfg.u_min, cfg.u_max])) + 1.0e-9;
    finite_pass = all(isfinite(X(:))) && all(isfinite(U(:)));
    terminal_usable = final_max_angle_error <= cfg.usable_angle_tolerance_rad && ...
                      final_velocity_norm <= cfg.usable_velocity_norm_limit;
    constraint_usable = max_defect <= cfg.max_collocation_defect_for_pass && ...
                        max_ineq <= cfg.max_ineq_violation_for_pass;
    solver_usable = exitflag > 0 || (exitflag == 0 && terminal_usable && constraint_usable);
    optimization_usable = finite_pass && rail_pass && force_pass && terminal_usable && constraint_usable && solver_usable;

    metrics = struct();
    metrics.finite_pass = finite_pass;
    metrics.rail_pass = rail_pass;
    metrics.force_pass = force_pass;
    metrics.terminal_usable = terminal_usable;
    metrics.constraint_usable = constraint_usable;
    metrics.solver_usable = solver_usable;
    metrics.optimization_usable = optimization_usable;
    metrics.max_collocation_defect = max_defect;
    metrics.max_ineq_violation = max_ineq;
    metrics.final_angle_error_1_rad = final_angle_err_1;
    metrics.final_angle_error_2_rad = final_angle_err_2;
    metrics.final_max_angle_error_rad = final_max_angle_error;
    metrics.final_velocity_norm = final_velocity_norm;
    metrics.max_abs_x_m = max_abs_x;
    metrics.max_abs_u_N = max_abs_u;
end

function local_write_summary(path, cfg, metrics, exitflag, output, fval)
    fid = fopen(path, 'w');
    if fid < 0
        warning('Phase28:SummaryOpenFailed', 'Could not write summary file: %s', path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, 'Phase 28 direct collocation swing-up optimization\n');
    fprintf(fid, 'state_order = [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]\n');
    fprintf(fid, 'angle_convention = theta = 0 downward, theta = pi upright\n');
    fprintf(fid, 'target_mode = up_up\n');
    fprintf(fid, 'N = %d\n', cfg.N);
    fprintf(fid, 'T_s = %.12g\n', cfg.T);
    fprintf(fid, 'dt_s = %.12g\n', cfg.dt);
    fprintf(fid, 'exitflag = %d\n', exitflag);
    fprintf(fid, 'fval = %.12g\n', fval);
    if isfield(output, 'message')
        fprintf(fid, 'solver_message = %s\n', output.message);
    end
    fields = fieldnames(metrics);
    for i = 1:numel(fields)
        value = metrics.(fields{i});
        if islogical(value)
            fprintf(fid, '%s = %d\n', fields{i}, value);
        else
            fprintf(fid, '%s = %.12g\n', fields{i}, double(value));
        end
    end
    fprintf(fid, 'trajectory_file = %s\n', cfg.trajectory_file);
end

function local_write_status_doc(path, cfg, metrics, exitflag, output)
    fid = fopen(path, 'w');
    if fid < 0
        warning('Phase28:StatusOpenFailed', 'Could not write status file: %s', path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# PROJECT STATUS - PHASE 28\n\n');
    fprintf(fid, '## Phase 28 - Direct collocation swing-up optimization\n\n');
    fprintf(fid, 'Target mode: `up_up`.  \n');
    fprintf(fid, 'State order: `[x, x_dot, theta1, theta1_dot, theta2, theta2_dot]`.  \n');
    fprintf(fid, 'Angle convention: `theta = 0 downward`, `theta = pi upright`.\n\n');
    fprintf(fid, '## Files created\n\n');
    fprintf(fid, '- `matlab/control/optimize_swingup_direct_collocation.m`\n');
    fprintf(fid, '- `matlab/simulation/run_optimized_swingup_test.m`\n');
    fprintf(fid, '- `shared/trajectories/up_up_swingup_optimized.mat`\n');
    fprintf(fid, '- `results/phase28_direct_collocation/direct_collocation_summary.txt`\n');
    fprintf(fid, '- `results/phase28_direct_collocation/direct_collocation_convergence.mat`\n');
    fprintf(fid, '- `results/phase28_direct_collocation/optimized_trajectory.png`\n\n');
    fprintf(fid, '## Optimization setup\n\n');
    fprintf(fid, '- Method: RK4 zero order hold direct collocation / multiple shooting style with `fmincon`.\n');
    fprintf(fid, '- Nodes: `%d`.\n', cfg.N);
    fprintf(fid, '- Final time: `%.6g s`.\n', cfg.T);
    fprintf(fid, '- RK4 substeps per interval: `%d`.\n', cfg.integration_substeps_per_interval);
    fprintf(fid, '- Force bounds: `[%.6g, %.6g] N`.\n', cfg.u_min, cfg.u_max);
    fprintf(fid, '- Rail bounds: `[%.6g, %.6g] m`.\n\n', cfg.x_min, cfg.x_max);
    fprintf(fid, '## Result\n\n');
    fprintf(fid, '- exitflag: `%d`\n', exitflag);
    if isfield(output, 'message')
        msg = regexprep(output.message, '\s+', ' ');
        fprintf(fid, '- solver message: `%s`\n', msg);
    end
    fprintf(fid, '- optimization_usable: `%d`\n', metrics.optimization_usable);
    fprintf(fid, '- finite_pass: `%d`\n', metrics.finite_pass);
    fprintf(fid, '- rail_pass: `%d`\n', metrics.rail_pass);
    fprintf(fid, '- force_pass: `%d`\n', metrics.force_pass);
    fprintf(fid, '- final_max_angle_error_rad: `%.12g`\n', metrics.final_max_angle_error_rad);
    fprintf(fid, '- final_velocity_norm: `%.12g`\n', metrics.final_velocity_norm);
    fprintf(fid, '- max_abs_x_m: `%.12g`\n', metrics.max_abs_x_m);
    fprintf(fid, '- max_abs_u_N: `%.12g`\n', metrics.max_abs_u_N);
    fprintf(fid, '- max_collocation_defect: `%.12g`\n', metrics.max_collocation_defect);
    fprintf(fid, '- max_ineq_violation: `%.12g`\n\n', metrics.max_ineq_violation);
    fprintf(fid, '## Known limitations\n\n');
    fprintf(fid, '- This phase creates a feedforward optimized trajectory using RK4-consistent defects. It is not yet a robust feedback tracking controller.\n');
    fprintf(fid, '- If replay fails under disturbance, actuator delay or sensor noise, that is expected and should be handled in Phase 29 using tracking control or TVLQR.\n');
end

function local_plot_trajectory(cfg, trajectory)
    try
        t = trajectory.t;
        X = trajectory.x_ref;
        U = trajectory.u_ff;
        fig = figure('Name', 'Phase 28 optimized trajectory', 'Color', 'w');
        tiledlayout(4, 1);
        nexttile;
        plot(t, X(:, 1), 'LineWidth', 1.2); hold on;
        yline(cfg.x_min, '--'); yline(cfg.x_max, '--');
        grid on; ylabel('x [m]'); title('Optimized direct collocation trajectory');
        nexttile;
        plot(t, X(:, 3), 'LineWidth', 1.2); hold on;
        plot(t, X(:, 5), 'LineWidth', 1.2);
        yline(pi, '--'); grid on; ylabel('angle [rad]'); legend('theta1', 'theta2', 'pi', 'Location', 'best');
        nexttile;
        plot(t, X(:, 2), 'LineWidth', 1.2); hold on;
        plot(t, X(:, 4), 'LineWidth', 1.2);
        plot(t, X(:, 6), 'LineWidth', 1.2);
        grid on; ylabel('velocity'); legend('x dot', 'theta1 dot', 'theta2 dot', 'Location', 'best');
        nexttile;
        stairs(trajectory.u_time, U, 'LineWidth', 1.2); hold on;
        yline(cfg.u_min, '--'); yline(cfg.u_max, '--');
        grid on; xlabel('time [s]'); ylabel('u ff [N]');
        saveas(fig, cfg.trajectory_plot_file);
    catch plot_error
        warning('Phase28:PlotFailed', 'Could not create optimized trajectory plot: %s', plot_error.message);
    end
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2.0 * pi) - pi;
end
