function tracker = tvlqr_discrete_tracking(trajectory, params, user_cfg)
% TVLQR_DISCRETE_TRACKING Clean Phase 29 discrete finite horizon TVLQR.
%
% Purpose:
%   Build a trajectory tracking controller directly from the Phase 28 optimized
%   RK4 zero-order-hold mesh.  This avoids interpolation fallback and keeps the
%   tracking design consistent with the direct-collocation replay model.
%
% State order:
%   [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
%
% Angle convention:
%   theta = 0 downward, theta = pi upright
%
% Control law used by tvlqr_tracking_eval_clean:
%   u = u_ff(k) - K(:,:,k) * e
%
% where e is the wrapped state error relative to x_ref(k).

    if nargin < 2 || isempty(params)
        params = load_params();
    end
    if nargin < 3 || isempty(user_cfg)
        user_cfg = struct();
    end

    cfg = local_default_cfg(user_cfg, params, trajectory, false);
    ref = local_build_reference(trajectory, cfg);
    n = 6;
    m = 1;
    N = numel(ref.t);
    M = N - 1;

    A = zeros(n, n, M);
    B = zeros(n, m, M);
    defect = zeros(M, 1);

    params_lin = params;
    params_lin.realism.actuator_enabled = false;
    params_lin.realism.sensor_noise_enabled = false;
    params_lin.realism.estimator_enabled = false;
    params_lin.realism.use_measurement_for_control = false;
    params_lin.realism.use_estimator_for_control = false;
    params_lin.realism.friction_enabled = cfg.friction_enabled;
    params_lin.realism.rail_limit_enabled = cfg.rail_limit_enabled;

    for k = 1:M
        xk = ref.x(:, k);
        uk = ref.u_actual(k);
        dt = ref.t(k + 1) - ref.t(k);
        phi_nom = local_rk4_zoh_step(xk, uk, dt, cfg.integration_substeps_per_interval, params_lin);
        defect(k) = norm(local_wrapped_state_error(phi_nom, ref.x(:, k + 1)), inf);
        [A(:, :, k), B(:, :, k)] = local_linearize_ideal_map(xk, uk, dt, cfg, params_lin);
    end

    Q = diag(cfg.Q_diag(:));
    Qf = diag(cfg.Qf_diag(:));
    R = cfg.R;
    [K, P, riccati_ok, max_gain] = local_backward_riccati(A, B, Q, R, Qf);

    tracker = struct();
    tracker.type = 'discrete_tvlqr_ideal';
    tracker.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    tracker.angle_convention = 'theta = 0 downward, theta = pi upright';
    tracker.control_law = 'u = u_ff(k) - K(k)*e(k)';
    tracker.t = ref.t;
    tracker.x_ref = ref.x;
    tracker.u_ref = ref.u_actual;
    tracker.u_cmd_ref = ref.u_actual;
    tracker.K = K;
    tracker.A = A;
    tracker.B = B;
    tracker.P = P;
    tracker.Q_diag = cfg.Q_diag;
    tracker.Qf_diag = cfg.Qf_diag;
    tracker.R = cfg.R;
    tracker.integration_substeps_per_interval = cfg.integration_substeps_per_interval;
    tracker.linearization_defect_inf = defect;
    tracker.max_linearization_reference_defect = max(defect);
    tracker.riccati_ok = riccati_ok;
    tracker.max_abs_gain = max_gain;
    tracker.pass = riccati_ok && all(isfinite(K(:))) && tracker.max_linearization_reference_defect <= cfg.max_reference_defect_for_pass;
    tracker.config = cfg;
end

function cfg = local_default_cfg(user_cfg, params, trajectory, is_augmented) %#ok<INUSD>
    cfg = struct();
    cfg.integration_substeps_per_interval = local_get_nested(trajectory, {'config','integration_substeps_per_interval'}, 7);
    cfg.integration_substeps_per_interval = max(1, ceil(cfg.integration_substeps_per_interval));
    cfg.terminal_capture_enabled = true;
    cfg.terminal_capture_time_s = 2.0;
    cfg.terminal_capture_dt_s = local_get_field(trajectory, 'dt_s', median(diff(double(trajectory.t(:)))));
    cfg.friction_enabled = false;
    cfg.rail_limit_enabled = false;
    cfg.max_reference_defect_for_pass = 1.0e-3;
    cfg.state_fd_eps = [1e-5; 1e-5; 1e-5; 1e-5; 1e-5; 1e-5];
    cfg.input_fd_eps = 1.0e-4;
    cfg.Q_diag = [12, 1.5, 120, 5, 120, 5];
    cfg.Qf_diag = [150, 15, 1500, 90, 1500, 90];
    cfg.R = 2.0;

    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
end

function ref = local_build_reference(trajectory, cfg)
    t0 = double(trajectory.t(:)).';
    X0 = double(trajectory.x_ref).';
    U0 = double(trajectory.u_ff(:)).';
    if size(X0, 1) ~= 6
        error('tvlqr_discrete_tracking:InvalidTrajectory', 'trajectory.x_ref must be N x 6.');
    end
    if numel(U0) ~= numel(t0) - 1
        error('tvlqr_discrete_tracking:InvalidTrajectory', 'trajectory.u_ff length must be N-1.');
    end

    t = t0;
    X = X0;
    U = U0;
    if cfg.terminal_capture_enabled
        dt_cap = double(cfg.terminal_capture_dt_s);
        if ~isfinite(dt_cap) || dt_cap <= 0
            dt_cap = median(diff(t0));
        end
        n_cap = max(1, ceil(double(cfg.terminal_capture_time_s) / dt_cap));
        target = [0; 0; pi; 0; pi; 0];
        for j = 1:n_cap
            t(end + 1) = t(end) + dt_cap; %#ok<AGROW>
            X(:, end + 1) = target; %#ok<AGROW>
            U(end + 1) = 0.0; %#ok<AGROW>
        end
    end

    ref = struct();
    ref.t = t(:);
    ref.x = X;
    ref.u_actual = U(:);
end

function [A, B] = local_linearize_ideal_map(x, u, dt, cfg, params)
    n = numel(x);
    A = zeros(n, n);
    for i = 1:n
        h = cfg.state_fd_eps(min(i, numel(cfg.state_fd_eps)));
        dx = zeros(n, 1);
        dx(i) = h;
        xp = local_rk4_zoh_step(x + dx, u, dt, cfg.integration_substeps_per_interval, params);
        xm = local_rk4_zoh_step(x - dx, u, dt, cfg.integration_substeps_per_interval, params);
        A(:, i) = local_wrapped_state_error(xp, xm) / (2.0 * h);
    end

    h = double(cfg.input_fd_eps);
    u_min = double(params.control.min_cart_force_N);
    u_max = double(params.control.max_cart_force_N);
    if u >= u_max - 5*h
        xp = local_rk4_zoh_step(x, u, dt, cfg.integration_substeps_per_interval, params);
        xm = local_rk4_zoh_step(x, u - h, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_state_error(xp, xm) / h;
    elseif u <= u_min + 5*h
        xp = local_rk4_zoh_step(x, u + h, dt, cfg.integration_substeps_per_interval, params);
        xm = local_rk4_zoh_step(x, u, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_state_error(xp, xm) / h;
    else
        xp = local_rk4_zoh_step(x, u + h, dt, cfg.integration_substeps_per_interval, params);
        xm = local_rk4_zoh_step(x, u - h, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_state_error(xp, xm) / (2.0 * h);
    end
end

function [K, P, ok, max_gain] = local_backward_riccati(A, B, Q, R, Qf)
    [n, ~, M] = size(A);
    K = zeros(1, n, M);
    P = zeros(n, n, M + 1);
    P(:, :, M + 1) = Qf;
    ok = true;
    for k = M:-1:1
        Ak = A(:, :, k);
        Bk = B(:, :, k);
        Pn = P(:, :, k + 1);
        S = R + Bk.' * Pn * Bk;
        if ~isfinite(S) || abs(S) < 1.0e-12
            ok = false;
            S = S + 1.0e-9;
        end
        Kk = S \ (Bk.' * Pn * Ak);
        Pk = Q + Ak.' * Pn * (Ak - Bk * Kk);
        Pk = 0.5 * (Pk + Pk.');
        K(:, :, k) = Kk;
        P(:, :, k) = Pk;
        if any(~isfinite(Kk(:))) || any(~isfinite(Pk(:)))
            ok = false;
        end
    end
    max_gain = max(abs(K(:)));
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

function e = local_wrapped_state_error(x, xref)
    e = x(:) - xref(:);
    e(3) = local_wrap_to_pi(e(3));
    e(5) = local_wrap_to_pi(e(5));
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function value = local_get_field(s, name, default_value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        value = double(s.(name));
    else
        value = default_value;
    end
end

function value = local_get_nested(s, names, default_value)
    value = default_value;
    cur = s;
    for i = 1:numel(names)
        if isstruct(cur) && isfield(cur, names{i})
            cur = cur.(names{i});
        else
            return;
        end
    end
    if ~isempty(cur)
        value = double(cur);
    end
end
