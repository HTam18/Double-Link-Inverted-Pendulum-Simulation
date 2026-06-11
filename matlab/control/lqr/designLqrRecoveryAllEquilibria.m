function report = designLqrRecoveryAllEquilibria(params, user_cfg)

    if nargin < 1 || isempty(params), params = loadParams(); end
    if nargin < 2 || isempty(user_cfg), user_cfg = struct(); end

    project_root = RecoveryUtils.projectRoot(params);
    out_dir = fullfile(project_root, 'shared');
    result_dir = fullfile(project_root, 'results', 'recovery');
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    if ~exist(result_dir, 'dir'), mkdir(result_dir); end

    lib = targetEquilibriumLibrary(params);
    state_step = RecoveryUtils.getDouble(user_cfg, 'state_step', 1e-6);
    input_step = RecoveryUtils.getDouble(user_cfg, 'input_step', 1e-6);
    u0 = 0.0;

    Q_up_up = diag([5.0, 2.0, 320.0, 26.0, 320.0, 26.0]);
    Q_up_down = diag([4.0, 1.6, 260.0, 22.0, 220.0, 18.0]);
    Q_down_up = diag([5.0, 2.2, 220.0, 18.0, 300.0, 26.0]);
    Q_down_down = diag([2.0, 1.2, 100.0, 10.0, 100.0, 10.0]);
    R_default = RecoveryUtils.getDouble(user_cfg, 'R_default', 0.45);

    gains = struct();
    linear_models = struct();
    controllability = struct();
    lines = {};
    lines{end+1} = 'RECOVERY LQR DESIGN REPORT'; %#ok<AGROW>
    lines{end+1} = 'This file is separate from LQR baseline local LQR gains.'; %#ok<AGROW>
    lines{end+1} = 'Output: shared/lqr_recovery_gains_all_equilibria.mat'; %#ok<AGROW>
    lines{end+1} = ' '; %#ok<AGROW>

    fprintf('Recovery - designing recovery LQR gains for disturbance rejection\n');
    for i = 1:numel(lib.target_order)
        name = lib.target_order{i};
        x0 = lib.targets.(name).target_state(:);
        [A, B] = local_linearize(@(x, u) dipDynamicsNonlinear(x, u, params, zeros(3,1)), x0, u0, state_step, input_step);
        ctrb_rank = rank(local_ctrb(A, B), 1e-8);
        switch name
            case 'up_up'
                Q = Q_up_up;
            case 'up_down'
                Q = Q_up_down;
            case 'down_up'
                Q = Q_down_up;
            otherwise
                Q = Q_down_down;
        end
        R = R_default;
        K = local_lqr(A, B, Q, R);
        eig_closed = eig(A - B * K);
        closed_stable = all(real(eig_closed) < -1e-7);

        gains.(name).K = K;
        gains.(name).Q = Q;
        gains.(name).R = R;
        gains.(name).target_state = x0;
        gains.(name).target_name = name;
        gains.(name).state_order = lib.state_order;
        gains.(name).angle_error_indices = [3, 5];
        gains.(name).u_min_N = double(params.control.min_cart_force_N);
        gains.(name).u_max_N = double(params.control.max_cart_force_N);
        gains.(name).design_note = 'Recovery recovery LQR, not LQR baseline local holding gain.';

        linear_models.(name).A = A;
        linear_models.(name).B = B;
        linear_models.(name).target_state = x0;
        controllability.(name).rank = ctrb_rank;
        controllability.(name).closed_loop_stable = closed_stable;
        controllability.(name).closed_loop_eigenvalues = eig_closed;

        fprintf('%s: rank %d/6, recovery closed-loop stable = %d\n', name, ctrb_rank, closed_stable);
        lines{end+1} = sprintf('Target %s: rank=%d/6, closed_loop_stable=%d, R=%.4g', name, ctrb_rank, closed_stable, R); %#ok<AGROW>
        lines{end+1} = sprintf('  K = [%s]', sprintf(' %.10g', K)); %#ok<AGROW>
    end

    report = struct();
    report.workflow = 'Recovery';
    report.source = 'matlab/control/lqr/designLqrRecoveryAllEquilibria.m';
    report.gains = gains;
    report.linear_models = linear_models;
    report.controllability = controllability;
    report.note = 'Recovery gain file is loaded by disturbanceRecoveryController when available.';

    mat_path = fullfile(out_dir, 'lqr_recovery_gains_all_equilibria.mat');
    txt_path = fullfile(result_dir, 'recovery_recovery_lqr_design_report.txt');
    save(mat_path, 'report', 'gains', 'linear_models', 'controllability');
    RecoveryIo.writeText(txt_path, strjoin(lines, newline));
    fprintf('Saved recovery gains: %s\n', mat_path);
    fprintf('Saved recovery design report: %s\n', txt_path);
end

function [A, B] = local_linearize(f, x0, u0, dx, du)
    n = numel(x0);
    A = zeros(n,n); B = zeros(n,1);
    for k = 1:n
        d = zeros(n,1); d(k) = dx;
        A(:,k) = (f(x0+d, u0) - f(x0-d, u0)) ./ (2*dx);
    end
    B(:,1) = (f(x0, u0+du) - f(x0, u0-du)) ./ (2*du);
end

function C = local_ctrb(A, B)
    n = size(A,1); C = zeros(n,n*size(B,2)); block = B;
    for k = 1:n
        C(:, ((k-1)*size(B,2)+1):(k*size(B,2))) = block;
        block = A * block;
    end
end

function K = local_lqr(A, B, Q, R)
    if exist('lqr','file') == 2
        K = lqr(A, B, Q, R);
    elseif exist('care','file') == 2
        P = care(A, B, Q, R);
        K = R \ (B' * P);
    else
        error('designLqrRecoveryAllEquilibria:MissingToolbox', 'Need lqr or care.');
    end
end

function root = projectRoot(params)
    if isfield(params,'meta') && isfield(params.meta,'project_root') && ~isempty(params.meta.project_root)
        root = char(params.meta.project_root);
    else
        root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    end
end

function v = getDouble(s, name, default_value)
    v = default_value;
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v = double(s.(name)); end
end

function writeText(path, txt)
    fid = fopen(path,'w');
    if fid < 0, error('Cannot write %s', path); end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s', txt);
end
