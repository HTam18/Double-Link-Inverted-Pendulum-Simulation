function traj = designTvlqrRecoveryTrajectory(traj, params, cfg)

    if nargin < 3 || isempty(cfg), cfg = struct(); end
    if nargin < 2 || isempty(params), params = loadParams(); end
    X = double(traj.states);
    U = double(traj.u_ff(:));
    t = double(traj.time_s(:));
    n = size(X,1);
    if numel(U) ~= n
        U = interp1(linspace(t(1),t(end),numel(U)).', U, t, 'linear', 'extrap');
    end
    Q = local_get_matrix(cfg, 'Q', diag([35, 8, 450, 35, 450, 35]));
    R = local_get_matrix(cfg, 'R', 0.75);
    Qf = local_get_matrix(cfg, 'Qf', diag([250, 60, 3500, 250, 3500, 250]));
    eps_x = RecoveryUtils.getDouble(cfg, 'finite_diff_state_eps', 1e-5);
    eps_u = RecoveryUtils.getDouble(cfg, 'finite_diff_input_eps', 1e-4);

    K_seq = zeros(max(n-1,1), 6);
    P = Qf;
    A_seq = zeros(6,6,max(n-1,1));
    B_seq = zeros(6,1,max(n-1,1));
    for k = (n-1):-1:1
        dt = max(t(k+1)-t(k), 1e-3);
        xk = X(k,:).';
        uk = U(k);
        [A,B] = local_linearize_discrete(xk, uk, params, dt, eps_x, eps_u);
        A_seq(:,:,k) = A;
        B_seq(:,:,k) = B;
        G = R + B.'*P*B;
        if rcond(G) < 1e-10
            K = (pinv(G) * (B.'*P*A));
        else
            K = G \ (B.'*P*A);
        end
        K_seq(k,:) = K;
        P = Q + A.'*P*(A - B*K);
        P = 0.5*(P+P.');
    end
    traj.K_seq = K_seq;
    traj.A_seq = A_seq;
    traj.B_seq = B_seq;
    traj.tvlqr_Q = Q;
    traj.tvlqr_R = R;
    traj.tvlqr_Qf = Qf;
    traj.tvlqr_status = 'finite_horizon_tvlqr_designed';
end

function [A,B] = local_linearize_discrete(x,u,params,dt,eps_x,eps_u)
    nx = 6;
    f0 = local_step(x,u,params,dt); %#ok<NASGU>
    A = zeros(nx,nx);
    for i=1:nx
        dx = zeros(nx,1); dx(i)=eps_x;
        fp = local_step(x+dx,u,params,dt);
        fm = local_step(x-dx,u,params,dt);
        col = (fp-fm)/(2*eps_x);
        col(3) = atan2(sin(fp(3)-fm(3)), cos(fp(3)-fm(3))) / (2*eps_x);
        col(5) = atan2(sin(fp(5)-fm(5)), cos(fp(5)-fm(5))) / (2*eps_x);
        A(:,i)=col;
    end
    fp = local_step(x,u+eps_u,params,dt);
    fm = local_step(x,u-eps_u,params,dt);
    B = (fp-fm)/(2*eps_u);
    B(3) = atan2(sin(fp(3)-fm(3)), cos(fp(3)-fm(3))) / (2*eps_u);
    B(5) = atan2(sin(fp(5)-fm(5)), cos(fp(5)-fm(5))) / (2*eps_u);
end

function xn = local_step(x,u,params,dt)
    h = dt;
    f = @(xx) dipDynamicsNonlinear(xx, u, params, zeros(3,1));
    k1=f(x); k2=f(x+0.5*h*k1); k3=f(x+0.5*h*k2); k4=f(x+h*k3);
    xn = x + h/6*(k1+2*k2+2*k3+k4);
end

function M = local_get_matrix(s,name,default_value)
    M = default_value;
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), M = double(s.(name)); end
end

function v = getDouble(s,name,default_value)
    v = default_value;
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name)), v = double(s.(name)); end
end
