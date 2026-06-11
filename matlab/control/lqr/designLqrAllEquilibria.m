function report = designLqrAllEquilibria(params)

    if nargin < 1 || isempty(params)
        params = loadParams();
    end

    lib = targetEquilibriumLibrary(params);
    project_root = RecoveryUtils.projectRoot(params);
    output_dir = fullfile(project_root, 'shared');
    result_dir = fullfile(project_root, 'results', 'lqrBaseline_multi_target_stabilize');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    if ~exist(result_dir, 'dir'), mkdir(result_dir); end

    state_step = 1e-6;
    input_step = 1e-6;
    u0 = 0.0;

    Q_default = diag([2.0, 1.0, 180.0, 12.0, 180.0, 12.0]);
    R_default = 0.8;
    Q_down_down = diag([1.0, 0.8, 60.0, 6.0, 60.0, 6.0]);
    R_down_down = 1.2;

    gains = struct();
    linear_models = struct();
    controllability = struct();
    controllers = struct();

    text_lines = {};
    text_lines{end+1} = 'CONTROLLABILITY AND LQR DESIGN REPORT'; %#ok<AGROW>
    text_lines{end+1} = 'State order: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]'; %#ok<AGROW>
    text_lines{end+1} = 'Angle convention: theta = 0 down, theta = pi up, theta2 absolute'; %#ok<AGROW>
    text_lines{end+1} = sprintf('Finite difference: state_step = %.3g, input_step = %.3g', state_step, input_step); %#ok<AGROW>
    text_lines{end+1} = ' '; %#ok<AGROW>

    fprintf('LQR baseline - LQR design for all equilibria\n');
    fprintf('State order and angle convention validated by targetEquilibriumLibrary.m\n\n');

    for i = 1:numel(lib.target_order)
        name = lib.target_order{i};
        target = lib.targets.(name);
        x0 = target.target_state(:);

        [A, B] = local_linearize(@(x, u) dipDynamicsNonlinear(x, u, params, zeros(3, 1)), x0, u0, state_step, input_step);
        Ctrb = local_ctrb(A, B);
        ctrb_rank = rank(Ctrb, 1e-8);
        is_controllable = (ctrb_rank == numel(x0));

        if strcmp(name, 'down_down')
            Q = Q_down_down;
            R = R_down_down;
        else
            Q = Q_default;
            R = R_default;
        end

        if ~is_controllable
            warning('designLqrAllEquilibria:NotFullyControllable', ...
                    'Target %s controllability rank is %d/6. LQR design will still try if stabilizable.', name, ctrb_rank);
        end

        K = local_lqr_or_care(A, B, Q, R);
        eig_open = eig(A);
        eig_closed = eig(A - B * K);
        closed_stable = all(real(eig_closed) < -1e-7);

        linear_models.(name).A = A;
        linear_models.(name).B = B;
        linear_models.(name).eig_open_loop = eig_open;
        linear_models.(name).target_state = x0;
        linear_models.(name).u_equilibrium_N = u0;

        controllability.(name).rank = ctrb_rank;
        controllability.(name).state_count = numel(x0);
        controllability.(name).is_controllable = is_controllable;
        controllability.(name).closed_loop_stable = closed_stable;
        controllability.(name).open_loop_eigenvalues = eig_open;
        controllability.(name).closed_loop_eigenvalues = eig_closed;

        gains.(name).K = K;
        gains.(name).Q = Q;
        gains.(name).R = R;
        gains.(name).target_state = x0;
        gains.(name).target_name = name;
        gains.(name).state_order = lib.state_order;
        gains.(name).angle_error_indices = [3, 5];
        gains.(name).u_min_N = double(params.control.min_cart_force_N);
        gains.(name).u_max_N = double(params.control.max_cart_force_N);

        controllers.(name).mode = name;
        controllers.(name).K = K;
        controllers.(name).Q = Q;
        controllers.(name).R = R;
        controllers.(name).target_state = x0.';
        controllers.(name).controllability_rank = ctrb_rank;
        controllers.(name).is_controllable = is_controllable;
        controllers.(name).closed_loop_stable = closed_stable;
        controllers.(name).closed_loop_eigenvalues = [real(eig_closed), imag(eig_closed)];

        fprintf('%s: controllability rank %d/6, closed-loop stable = %d\n', name, ctrb_rank, closed_stable);
        fprintf('  K = '); disp(K);

        text_lines{end+1} = sprintf('Target: %s', name); %#ok<AGROW>
        text_lines{end+1} = sprintf('  target_state = [%s]', sprintf(' %.8g', x0)); %#ok<AGROW>
        text_lines{end+1} = sprintf('  controllability_rank = %d / 6', ctrb_rank); %#ok<AGROW>
        text_lines{end+1} = sprintf('  is_controllable = %d', is_controllable); %#ok<AGROW>
        text_lines{end+1} = sprintf('  closed_loop_stable = %d', closed_stable); %#ok<AGROW>
        text_lines{end+1} = sprintf('  K = [%s]', sprintf(' %.10g', K)); %#ok<AGROW>
        text_lines{end+1} = ' '; %#ok<AGROW>
    end

    report = struct();
    report.workflow = 'LQR baseline';
    report.source = 'matlab/control/lqr/designLqrAllEquilibria.m';
    report.state_order = lib.state_order;
    report.angle_convention = lib.angle_convention;
    report.target_order = lib.target_order;
    report.targets = lib.targets;
    report.gains = gains;
    report.linear_models = linear_models;
    report.controllability = controllability;
    report.controllers = controllers;
    report.Q_default = Q_default;
    report.R_default = R_default;
    report.Q_down_down = Q_down_down;
    report.R_down_down = R_down_down;
    report.state_step = state_step;
    report.input_step = input_step;

    mat_path = fullfile(output_dir, 'lqr_gains_all_equilibria.mat');
    json_path = fullfile(output_dir, 'lqr_gains_all_equilibria.json');
    report_path = fullfile(result_dir, 'lqrBaseline_controllability_report.txt');

    save(mat_path, 'report', 'gains', 'linear_models', 'controllability', 'controllers');
    RecoveryIo.writeText(report_path, strjoin(text_lines, newline));
    RecoveryIo.writeText(json_path, jsonencode(local_json_safe_report(report), PrettyPrint=true));

    fprintf('\nSaved MAT:  %s\n', mat_path);
    fprintf('Saved JSON: %s\n', json_path);
    fprintf('Saved report: %s\n', report_path);
end

function [A, B] = local_linearize(f, x0, u0, state_step, input_step)
    n = numel(x0);
    A = zeros(n, n);
    B = zeros(n, 1);
    for col = 1:n
        dx = zeros(n, 1);
        dx(col) = state_step;
        A(:, col) = (f(x0 + dx, u0) - f(x0 - dx, u0)) ./ (2.0 * state_step);
    end
    B(:, 1) = (f(x0, u0 + input_step) - f(x0, u0 - input_step)) ./ (2.0 * input_step);
    if any(~isfinite(A(:))) || any(~isfinite(B(:)))
        error('designLqrAllEquilibria:NonFiniteLinearization', 'A or B contains NaN/Inf.');
    end
end

function C = local_ctrb(A, B)
    n = size(A, 1);
    C = zeros(n, n * size(B, 2));
    block = B;
    for k = 1:n
        cols = ((k - 1) * size(B, 2) + 1):(k * size(B, 2));
        C(:, cols) = block;
        block = A * block;
    end
end

function K = local_lqr_or_care(A, B, Q, R)
    if exist('lqr', 'file') == 2
        K = lqr(A, B, Q, R);
    elseif exist('care', 'file') == 2
        [P, ~, ~] = care(A, B, Q, R);
        K = R \ (B' * P);
    else
        error('designLqrAllEquilibria:MissingToolbox', ...
              'Need MATLAB lqr or care for LQR baseline LQR design. Install Control System Toolbox.');
    end
    if any(~isfinite(K(:)))
        error('designLqrAllEquilibria:NonFiniteGain', 'LQR gain contains NaN/Inf.');
    end
end

function project_root = projectRoot(params)
    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        project_root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        control_dir = fileparts(this_file);
        matlab_root = fileparts(control_dir);
        project_root = fileparts(matlab_root);
    end
end

function safe = local_json_safe_report(report)
    safe = struct();
    safe.workflow = report.workflow;
    safe.source = report.source;
    safe.state_order = report.state_order;
    safe.angle_convention = report.angle_convention;
    safe.target_order = report.target_order;
    safe.controllers = report.controllers;
    safe.note = 'MAT file contains full numeric A, B, gains, Q, R and controllability structs.';
end

function writeText(path, text)
    fid = fopen(path, 'w');
    if fid < 0
        error('designLqrAllEquilibria:WriteFailed', 'Cannot write %s.', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', text);
end
