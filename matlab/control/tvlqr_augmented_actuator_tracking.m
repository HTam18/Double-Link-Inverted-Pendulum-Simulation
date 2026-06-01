function tracker = tvlqr_augmented_actuator_tracking(trajectory, params, user_cfg)
% TVLQR_AUGMENTED_ACTUATOR_TRACKING Clean Phase 29 actuator-aware TVLQR.
%
% Purpose:
%   Build a finite horizon TVLQR for the augmented system
%       z = [x; F_actual]
%       input = F_cmd
%   so that actuator delay is included in the model instead of being handled
%   by rail guards or manual gain-scale tuning.
%
% Control law:
%   F_cmd = F_cmd_ref(k) - K_aug(:,:,k) * [x_error; F_actual - F_actual_ref]

    if nargin < 2 || isempty(params)
        params = load_params();
    end
    if nargin < 3 || isempty(user_cfg)
        user_cfg = struct();
    end

    cfg = local_default_cfg(user_cfg, params, trajectory);
    params_aug = local_apply_actuator_cfg(params, cfg);
    ref = local_build_reference(trajectory, cfg, params_aug);

    n = 7;
    M = numel(ref.t) - 1;
    A = zeros(n, n, M);
    B = zeros(n, 1, M);
    defect = zeros(M, 1);

    params_lin = params_aug;
    params_lin.realism.actuator_enabled = true;
    params_lin.realism.sensor_noise_enabled = false;
    params_lin.realism.estimator_enabled = false;
    params_lin.realism.use_measurement_for_control = false;
    params_lin.realism.use_estimator_for_control = false;
    params_lin.realism.friction_enabled = cfg.friction_enabled;
    params_lin.realism.rail_limit_enabled = cfg.rail_limit_enabled;

    for k = 1:M
        zk = ref.z(:, k);
        ucmd = ref.u_cmd(k);
        dt = ref.t(k + 1) - ref.t(k);
        phi_nom = local_augmented_step(zk, ucmd, dt, cfg.integration_substeps_per_interval, params_lin);
        defect(k) = norm(local_wrapped_aug_error(phi_nom, ref.z(:, k + 1)), inf);
        [A(:, :, k), B(:, :, k)] = local_linearize_augmented_map(zk, ucmd, dt, cfg, params_lin);
    end

    Q = diag(cfg.Q_aug_diag(:));
    Qf = diag(cfg.Qf_aug_diag(:));
    R = cfg.R_aug;
    [K, P, riccati_ok, max_gain] = local_backward_riccati(A, B, Q, R, Qf);

    tracker = struct();
    tracker.type = 'discrete_tvlqr_augmented_actuator';
    tracker.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot', 'F_actual'};
    tracker.angle_convention = 'theta = 0 downward, theta = pi upright';
    tracker.control_law = 'F_cmd = F_cmd_ref(k) - K_aug(k)*[e_x; e_F]';
    tracker.t = ref.t;
    tracker.x_ref = ref.x;
    tracker.z_ref = ref.z;
    tracker.u_ref = ref.F_actual_ref;
    tracker.F_actual_ref = ref.F_actual_ref;
    tracker.u_cmd_ref = ref.u_cmd;
    tracker.K = K;
    tracker.A = A;
    tracker.B = B;
    tracker.P = P;
    tracker.Q_aug_diag = cfg.Q_aug_diag;
    tracker.Qf_aug_diag = cfg.Qf_aug_diag;
    tracker.R_aug = cfg.R_aug;
    tracker.integration_substeps_per_interval = cfg.integration_substeps_per_interval;
    tracker.linearization_defect_inf = defect;
    tracker.max_linearization_reference_defect = max(defect);
    tracker.feedforward_inverse_feasible = ref.feedforward_inverse_feasible;
    tracker.feedforward_command_saturation_fraction = ref.feedforward_command_saturation_fraction;
    tracker.riccati_ok = riccati_ok;
    tracker.max_abs_gain = max_gain;
    tracker.pass = riccati_ok && all(isfinite(K(:))) && tracker.max_linearization_reference_defect <= cfg.max_reference_defect_for_pass;
    tracker.config = cfg;
end

function cfg = local_default_cfg(user_cfg, params, trajectory)
    cfg = struct();
    cfg.integration_substeps_per_interval = local_get_nested(trajectory, {'config','integration_substeps_per_interval'}, 7);
    cfg.integration_substeps_per_interval = max(1, ceil(cfg.integration_substeps_per_interval));
    cfg.terminal_capture_enabled = true;
    cfg.terminal_capture_time_s = 2.0;
    cfg.terminal_capture_dt_s = local_get_field(trajectory, 'dt_s', median(diff(double(trajectory.t(:)))));
    cfg.friction_enabled = false;
    cfg.rail_limit_enabled = false;
    cfg.max_reference_defect_for_pass = 5.0e-3;
    cfg.state_fd_eps = [1e-5; 1e-5; 1e-5; 1e-5; 1e-5; 1e-5; 1e-4];
    cfg.input_fd_eps = 1.0e-4;
    cfg.actuator_tau_motor_s = 0.012;
    cfg.actuator_rate_limit_N_per_s = 500.0;
    cfg.actuator_dead_zone_N = 0.0;
    cfg.Q_aug_diag = [12, 1.5, 120, 5, 120, 5, 0.08];
    cfg.Qf_aug_diag = [150, 15, 1500, 90, 1500, 90, 0.5];
    cfg.R_aug = 2.5;
    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
end

function params = local_apply_actuator_cfg(params, cfg)
    if ~isfield(params, 'actuator') || isempty(params.actuator)
        params.actuator = struct();
    end
    params.actuator.tau_motor_s = cfg.actuator_tau_motor_s;
    params.actuator.rate_limit_N_per_s = cfg.actuator_rate_limit_N_per_s;
    params.actuator.dead_zone_N = cfg.actuator_dead_zone_N;
    params.actuator.min_force_N = double(params.control.min_cart_force_N);
    params.actuator.max_force_N = double(params.control.max_cart_force_N);
end

function ref = local_build_reference(trajectory, cfg, params)
    t0 = double(trajectory.t(:)).';
    X0 = double(trajectory.x_ref).';
    Uactual0 = double(trajectory.u_ff(:)).';
    if size(X0, 1) ~= 6
        error('tvlqr_augmented_actuator_tracking:InvalidTrajectory', 'trajectory.x_ref must be N x 6.');
    end

    t = t0;
    X = X0;
    Factual = Uactual0;
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
            Factual(end + 1) = 0.0; %#ok<AGROW>
        end
    end

    M = numel(t) - 1;
    Fcmd = zeros(1, M);
    tau = double(params.actuator.tau_motor_s);
    u_min = double(params.actuator.min_force_N);
    u_max = double(params.actuator.max_force_N);
    n_sat = 0;
    for k = 1:M
        if k < M
            dt = t(k + 1) - t(k);
            dF = (Factual(k + 1) - Factual(k)) / max(dt, eps);
        else
            dF = 0.0;
        end
        raw = Factual(k) + tau * dF;
        Fcmd(k) = min(max(raw, u_min), u_max);
        if abs(raw - Fcmd(k)) > 1.0e-9
            n_sat = n_sat + 1;
        end
    end

    Z = zeros(7, numel(t));
    Z(1:6, :) = X;
    Z(7, 1:M) = Factual(1:M);
    Z(7, end) = Factual(end);

    ref = struct();
    ref.t = t(:);
    ref.x = X;
    ref.z = Z;
    ref.F_actual_ref = Factual(:);
    ref.u_cmd = Fcmd(:);
    ref.feedforward_command_saturation_fraction = n_sat / max(1, M);
    ref.feedforward_inverse_feasible = ref.feedforward_command_saturation_fraction <= 0.10;
end

function [A, B] = local_linearize_augmented_map(z, ucmd, dt, cfg, params)
    n = numel(z);
    A = zeros(n, n);
    for i = 1:n
        h = cfg.state_fd_eps(min(i, numel(cfg.state_fd_eps)));
        dz = zeros(n, 1);
        dz(i) = h;
        zp = local_augmented_step(z + dz, ucmd, dt, cfg.integration_substeps_per_interval, params);
        zm = local_augmented_step(z - dz, ucmd, dt, cfg.integration_substeps_per_interval, params);
        A(:, i) = local_wrapped_aug_error(zp, zm) / (2.0 * h);
    end

    h = double(cfg.input_fd_eps);
    u_min = double(params.actuator.min_force_N);
    u_max = double(params.actuator.max_force_N);
    if ucmd >= u_max - 5*h
        zp = local_augmented_step(z, ucmd, dt, cfg.integration_substeps_per_interval, params);
        zm = local_augmented_step(z, ucmd - h, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_aug_error(zp, zm) / h;
    elseif ucmd <= u_min + 5*h
        zp = local_augmented_step(z, ucmd + h, dt, cfg.integration_substeps_per_interval, params);
        zm = local_augmented_step(z, ucmd, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_aug_error(zp, zm) / h;
    else
        zp = local_augmented_step(z, ucmd + h, dt, cfg.integration_substeps_per_interval, params);
        zm = local_augmented_step(z, ucmd - h, dt, cfg.integration_substeps_per_interval, params);
        B = local_wrapped_aug_error(zp, zm) / (2.0 * h);
    end
end

function z_next = local_augmented_step(z, F_cmd, dt, n_substeps, params)
    x = z(1:6);
    F_actual = z(7);
    h = dt / double(n_substeps);
    for ii = 1:n_substeps
        [F_next, ~] = dip_actuator(F_actual, F_cmd, h, params);
        F_mid = 0.5 * (F_actual + F_next);
        x = local_rk4_zoh_step(x, F_mid, h, 1, params);
        F_actual = F_next;
    end
    z_next = [x(:); F_actual];
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

function e = local_wrapped_aug_error(z, zref)
    e = z(:) - zref(:);
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
