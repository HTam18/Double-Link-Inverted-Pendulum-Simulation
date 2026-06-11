function out = tvlqrRecoveryTracking(t, state, recovery_traj, params, cfg)

    if nargin < 5 || isempty(cfg), cfg = struct(); end
    if nargin < 4 || isempty(params), params = loadParams(); end
    target_name = recovery_traj.target_name;
    ref = local_interp_traj(recovery_traj, t);
    K = local_gain_at_time(recovery_traj, params, target_name, t);
    x = double(state(:));
    e = x - ref;
    e(3) = atan2(sin(x(3)-ref(3)), cos(x(3)-ref(3)));
    e(5) = atan2(sin(x(5)-ref(5)), cos(x(5)-ref(5)));
    u_raw = -K * e;
    if isfield(recovery_traj, 'u_ff') && ~isempty(recovery_traj.u_ff)
        u_raw = u_raw + interp1(recovery_traj.time_s, recovery_traj.u_ff, min(max(double(t), double(recovery_traj.time_s(1))), double(recovery_traj.time_s(end))), 'linear', 'extrap');
    end
    u_cmd = min(max(double(u_raw), double(params.control.min_cart_force_N)), double(params.control.max_cart_force_N));
    out = struct('u_cmd',u_cmd,'u_raw',double(u_raw),'reference_state',ref,'mode','tvlqrRecoveryTracking','target_mode',target_name);
end

function ref = local_interp_traj(traj, t)
    tt = min(max(double(t), double(traj.time_s(1))), double(traj.time_s(end)));
    ref = zeros(6,1);
    for i=1:6
        ref(i) = interp1(traj.time_s, traj.states(:,i), tt, 'linear', 'extrap');
    end
end

function K = local_gain_at_time(traj, params, target_name, t)
    if isfield(traj, 'K_seq') && ~isempty(traj.K_seq) && isfield(traj, 'time_s')
        kk = find(traj.time_s <= t, 1, 'last');
        if isempty(kk), kk = 1; end
        kk = min(max(kk,1), size(traj.K_seq,1));
        K = double(traj.K_seq(kk,:));
        return;
    end
    K = local_load_gain(params, target_name);
end

function K = local_load_gain(params, target_name)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if isfield(params,'meta') && isfield(params.meta,'project_root') && ~isempty(params.meta.project_root), root = char(params.meta.project_root); end
    paths = {fullfile(root,'shared','lqr_recovery_gains_all_equilibria.mat'), fullfile(root,'shared','lqr_gains_all_equilibria.mat')};
    for i=1:numel(paths)
        if isfile(paths{i})
            S = load(paths{i});
            if isfield(S,'gains') && isfield(S.gains,target_name), K = double(S.gains.(target_name).K); return; end
            if isfield(S,'report') && isfield(S.report,'gains') && isfield(S.report.gains,target_name), K = double(S.report.gains.(target_name).K); return; end
        end
    end
    error('tvlqrRecoveryTracking:MissingGain','No gain for %s',target_name);
end
