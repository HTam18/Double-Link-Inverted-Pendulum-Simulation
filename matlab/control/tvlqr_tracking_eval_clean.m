function out = tvlqr_tracking_eval_clean(t, state, tracker, params, eval_state)
% TVLQR_TRACKING_EVAL_CLEAN Evaluate a clean Phase 29 TVLQR command.
%
% Ideal tracker:
%   u = u_ff(k) - K(k)*e_x
%
% Augmented actuator tracker:
%   F_cmd = F_cmd_ref(k) - K_aug(k)*[e_x; F_actual - F_actual_ref]
%
% This function deliberately contains no rail guard, no hard projection and no
% candidate-selection logic. Constraint handling is evaluated by metrics.

    if nargin < 5 || isempty(eval_state)
        eval_state = struct();
    end
    if nargin < 4 || isempty(params)
        params = load_params();
    end

    idx = local_interval_index(t, tracker.t);
    x = double(state(:));
    xref = tracker.x_ref(:, idx);
    ex = x - xref;
    ex(3) = local_wrap_to_pi(x(3) - xref(3));
    ex(5) = local_wrap_to_pi(x(5) - xref(5));

    if strcmp(char(tracker.type), 'discrete_tvlqr_augmented_actuator')
        if isfield(eval_state, 'F_actual')
            F_actual = double(eval_state.F_actual);
        else
            F_actual = 0.0;
        end
        zerr = [ex; F_actual - tracker.F_actual_ref(idx)];
        K = squeeze(tracker.K(:, :, idx));
        u_raw = tracker.u_cmd_ref(idx) - K * zerr;
        u_ff = tracker.u_cmd_ref(idx);
        F_actual_ref = tracker.F_actual_ref(idx);
        error_for_control = zerr;
    else
        K = squeeze(tracker.K(:, :, idx));
        u_raw = tracker.u_ref(idx) - K * ex;
        u_ff = tracker.u_ref(idx);
        F_actual_ref = NaN;
        error_for_control = ex;
    end

    u_cmd = local_saturate_force(u_raw, params);

    out = struct();
    out.u_cmd = u_cmd;
    out.u_raw = double(u_raw);
    out.u_ff = double(u_ff);
    out.mode = char(tracker.type);
    out.interval_index = idx;
    out.x_ref = xref;
    out.error_state = ex;
    out.error_for_control = error_for_control;
    out.F_actual_ref = F_actual_ref;
    out.measurement = x;
    out.x_hat = x;
end

function idx = local_interval_index(t, time_grid)
    tg = double(time_grid(:));
    idx = find(tg(1:end-1) <= double(t), 1, 'last');
    if isempty(idx)
        idx = 1;
    end
    idx = min(idx, numel(tg) - 1);
end

function force = local_saturate_force(u, params)
    min_force = double(params.control.min_cart_force_N);
    max_force = double(params.control.max_cart_force_N);
    force = min(max(double(u), min_force), max_force);
end

function a = local_wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
