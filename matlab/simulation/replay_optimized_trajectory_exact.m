function exact_result = replay_optimized_trajectory_exact(trajectory, params, user_cfg)
% REPLAY_OPTIMIZED_TRAJECTORY_EXACT Exact Phase 28 nonlinear replay.
%
% Purpose:
%   Replay the optimized trajectory on the same nonlinear MATLAB plant using
%   the same zero-order-hold interval structure used by the direct collocation
%   defect. This separates a valid optimized trajectory from later runner or
%   interpolation mismatch issues.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 downward, theta = pi upright.
%
% Important:
%   This is still an open-loop nonlinear plant replay. It does not inject the
%   reference state into the plant. It only applies u_ff(k) over each interval
%   and integrates the nonlinear dynamics forward.

    if nargin < 2 || isempty(params)
        params = load_params();
    end
    if nargin < 3 || isempty(user_cfg)
        user_cfg = struct();
    end

    if isfield(trajectory, 'config') && isfield(trajectory.config, 'integration_substeps_per_interval')
        n_sub_default = double(trajectory.config.integration_substeps_per_interval);
    else
        n_sub_default = 10;
    end
    if isfield(user_cfg, 'integration_substeps_per_interval') && ~isempty(user_cfg.integration_substeps_per_interval)
        n_sub = double(user_cfg.integration_substeps_per_interval);
    else
        n_sub = n_sub_default;
    end
    n_sub = max(1, ceil(n_sub));

    if isfield(user_cfg, 'use_optimizer_realism') && logical(user_cfg.use_optimizer_realism)
        % Keep current params as loaded. This option is reserved for future
        % cases where trajectory stores the full optimizer params.
    else
        % Phase 28 checks the optimized feedforward trajectory itself. Keep
        % friction/noise/actuator off to avoid mixing Phase 29 robustness and
        % tracking questions into Phase 28 open-loop validation.
        if ~isfield(params, 'realism') || isempty(params.realism)
            params.realism = struct();
        end
        params.realism.actuator_enabled = false;
        params.realism.friction_enabled = false;
        params.realism.sensor_noise_enabled = false;
        params.realism.estimator_enabled = false;
        params.realism.use_estimator_for_control = false;
        params.realism.use_measurement_for_control = false;
        if isfield(user_cfg, 'rail_limit_enabled')
            params.realism.rail_limit_enabled = logical(user_cfg.rail_limit_enabled);
        else
            params.realism.rail_limit_enabled = false;
        end
    end

    tref = double(trajectory.t(:));
    U = double(trajectory.u_ff(:));
    Xref = double(trajectory.x_ref);
    if size(Xref, 2) ~= 6
        error('replay_optimized_trajectory_exact:InvalidTrajectory', 'trajectory.x_ref must be N x 6.');
    end
    if numel(U) ~= numel(tref) - 1
        error('replay_optimized_trajectory_exact:InvalidInputLength', 'u_ff length must be numel(t)-1.');
    end

    N = numel(tref);
    X = zeros(N, 6);
    X(1, :) = Xref(1, :);
    Uactual = zeros(N, 1);
    max_defect_vs_ref = 0.0;

    for k = 1:(N - 1)
        dt = tref(k + 1) - tref(k);
        if dt <= 0
            error('replay_optimized_trajectory_exact:InvalidTimeGrid', 'trajectory.t must be strictly increasing.');
        end
        u_k = local_saturate_force(U(k), params);
        x_next = local_rk4_zoh_step(X(k, :).', u_k, dt, n_sub, params);
        X(k + 1, :) = x_next.';
        Uactual(k) = u_k;
        max_defect_vs_ref = max(max_defect_vs_ref, norm(X(k + 1, :).' - Xref(k + 1, :).', inf));
    end
    Uactual(end) = Uactual(end - 1);

    exact_result = struct();
    exact_result.t = tref;
    exact_result.state = X;
    exact_result.u_cmd = [U; U(end)];
    exact_result.u_actual = Uactual;
    exact_result.mode = repmat("optimized_exact_open_loop", N, 1);
    exact_result.measurement = X;
    exact_result.x_hat = X;
    exact_result.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    exact_result.angle_convention = 'theta = 0 downward, theta = pi upright';
    exact_result.max_defect_vs_reference_inf_norm = max_defect_vs_ref;
    exact_result.params = params;
    exact_result.pass = all(isfinite(X(:))) && all(isfinite(Uactual(:)));
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
        x_next = x_next + (h / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    end
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end
