clear; clc; close all;

%% Make paths robust
% This script can be run from the project root or from the scripts folder.
script_dir = fileparts(mfilename('fullpath'));

if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
addpath(script_dir);

%% Load parameters

run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; therefore rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));

if isempty(script_dir)
    script_dir = pwd;
end

project_root = fileparts(script_dir);
addpath(script_dir);

%% Build state-space model again
% The model is rebuilt here so this Phase 4 script can run even if the
% Phase 3 MAT file is missing. The LQR gain K is loaded from Phase 3 if
% available. If it is not available, the script calculates K using the same
% Q and R values from Phase 3.

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g);

%% Load LQR design result from Phase 3

phase3_data_file = fullfile(project_root, 'results', 'phase3_lqr_design', 'phase3_lqr_design_data.mat');

if exist(phase3_data_file, 'file')
    phase3_data = load(phase3_data_file, 'K', 'Q', 'R', 'closed_loop_poles');
    K = phase3_data.K;
    Q = phase3_data.Q;
    R = phase3_data.R;
    phase3_closed_loop_poles = phase3_data.closed_loop_poles;
    disp('Loaded LQR gain K from results/phase3_lqr_design/phase3_lqr_design_data.mat');
else
    warning('Phase 3 result file was not found. Recalculating K using the Phase 3 Q/R values.');
    Q = diag([5, 1, 80, 5, 80, 5]);
    R = 0.1;
    K = lqr(A, B, Q, R);
    phase3_closed_loop_poles = eig(A - B*K);
end

%% Basic checks

disp(' ');
disp('===== Phase 4: MATLAB Closed-loop Simulation =====');
disp('===== Matrix and Gain Check =====');
disp(['A size: ', mat2str(size(A))]);
disp(['B size: ', mat2str(size(B))]);
disp(['K size: ', mat2str(size(K))]);
disp(['x0 size: ', mat2str(size(x0))]);

if ~isequal(size(A), [6, 6])
    error('A matrix size is wrong. Expected 6x6.');
end

if ~isequal(size(B), [6, 1])
    error('B matrix size is wrong. Expected 6x1.');
end

if ~isequal(size(K), [1, 6])
    error('LQR gain K size is wrong. Expected 1x6.');
end

if ~isequal(size(x0), [6, 1])
    error('Initial state x0 size is wrong. Expected 6x1.');
end

if any(~isfinite(K), 'all')
    error('LQR gain K contains non-finite values.');
end

if any(~isfinite(x0), 'all')
    error('Initial state x0 contains non-finite values.');
end

disp('Matrix and gain check passed.');

%% Build closed-loop system
% Control law:
% u = -K*x
%
% Closed-loop state equation:
% x_dot = (A - B*K)*x

A_cl = A - B*K;
B_cl = zeros(6, 1);
C_cl = eye(6);
D_cl = zeros(6, 1);

closed_loop_poles = eig(A_cl);
closed_loop_stable = all(real(closed_loop_poles) < 0);

disp(' ');
disp('===== Closed-loop Pole Check =====');
disp(closed_loop_poles);

if ~closed_loop_stable
    error('Closed-loop system is not stable. Stop Phase 4 and check Phase 3 Q/R or model signs.');
end

disp('Closed-loop pole check passed. The closed-loop system is stable.');

%% Run closed-loop simulation
% There is no external input in this first Phase 4 simulation.
% The response is caused by the initial condition x0.

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

t = 0:dt:sim_time;
u_zero = zeros(size(t));

[y_out, t_out, x_out] = lsim(sys_cl, u_zero, t, x0);

% Because C_cl = eye(6), y_out and x_out contain the same state data.
cart_position = x_out(:, 1);
cart_velocity = x_out(:, 2);

theta1 = x_out(:, 3);
theta1_dot = x_out(:, 4);

theta2 = x_out(:, 5);
theta2_dot = x_out(:, 6);

% Control force over time.
u_out = -(K * x_out.').';

%% Calculate simple Phase 4 metrics

theta1_deg = rad2deg(theta1);
theta2_deg = rad2deg(theta2);

max_abs_theta1_deg = max(abs(theta1_deg));
max_abs_theta2_deg = max(abs(theta2_deg));
max_abs_cart_position = max(abs(cart_position));
max_abs_cart_velocity = max(abs(cart_velocity));
max_abs_control_force = max(abs(u_out));

final_theta1_deg = theta1_deg(end);
final_theta2_deg = theta2_deg(end);
final_cart_position = cart_position(end);

angle_settle_threshold_deg = 0.5;
cart_settle_threshold_m = 0.02;

theta1_settling_time = local_settling_time(t_out, theta1_deg, angle_settle_threshold_deg);
theta2_settling_time = local_settling_time(t_out, theta2_deg, angle_settle_threshold_deg);
cart_settling_time = local_settling_time(t_out, cart_position, cart_settle_threshold_m);

metrics = struct();
metrics.max_abs_theta1_deg = max_abs_theta1_deg;
metrics.max_abs_theta2_deg = max_abs_theta2_deg;
metrics.max_abs_cart_position_m = max_abs_cart_position;
metrics.max_abs_cart_velocity_m_per_s = max_abs_cart_velocity;
metrics.max_abs_control_force_N = max_abs_control_force;
metrics.final_theta1_deg = final_theta1_deg;
metrics.final_theta2_deg = final_theta2_deg;
metrics.final_cart_position_m = final_cart_position;
metrics.angle_settle_threshold_deg = angle_settle_threshold_deg;
metrics.cart_settle_threshold_m = cart_settle_threshold_m;
metrics.theta1_settling_time_s = theta1_settling_time;
metrics.theta2_settling_time_s = theta2_settling_time;
metrics.cart_settling_time_s = cart_settling_time;

%% Display metrics

disp(' ');
disp('===== Closed-loop Simulation Metrics =====');
disp(['Max abs theta1: ', num2str(max_abs_theta1_deg, '%.4f'), ' deg']);
disp(['Max abs theta2: ', num2str(max_abs_theta2_deg, '%.4f'), ' deg']);
disp(['Max abs cart position: ', num2str(max_abs_cart_position, '%.4f'), ' m']);
disp(['Max abs cart velocity: ', num2str(max_abs_cart_velocity, '%.4f'), ' m/s']);
disp(['Max abs control force: ', num2str(max_abs_control_force, '%.4f'), ' N']);
disp(['Final theta1: ', num2str(final_theta1_deg, '%.6f'), ' deg']);
disp(['Final theta2: ', num2str(final_theta2_deg, '%.6f'), ' deg']);
disp(['Final cart position: ', num2str(final_cart_position, '%.6f'), ' m']);

disp(['Theta1 settling time within +/-', num2str(angle_settle_threshold_deg), ' deg: ', local_time_to_text(theta1_settling_time)]);
disp(['Theta2 settling time within +/-', num2str(angle_settle_threshold_deg), ' deg: ', local_time_to_text(theta2_settling_time)]);
disp(['Cart settling time within +/-', num2str(cart_settle_threshold_m), ' m: ', local_time_to_text(cart_settling_time)]);

if max_abs_control_force > u_max
    warning('Control force exceeds u_max. This Phase 4 script does not apply saturation yet. Force saturation should be tested in a later phase.');
else
    disp('Control force stays within the current u_max value.');
end

%% Plot closed-loop response

results_folder = fullfile(project_root, 'results', 'phase4_closed_loop');

if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

figure('Name', 'Closed-loop Double Link Response');

subplot(4, 1, 1);
plot(t_out, theta1_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta1 (deg)');
title('Closed-loop Link 1 Angle');

subplot(4, 1, 2);
plot(t_out, theta2_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('theta2 (deg)');
title('Closed-loop Link 2 Angle');

subplot(4, 1, 3);
plot(t_out, cart_position, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Closed-loop Cart Position');

subplot(4, 1, 4);
plot(t_out, u_out, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('u (N)');
title('LQR Control Force');

saveas(gcf, fullfile(results_folder, 'closed_loop_response.png'));

figure('Name', 'Closed-loop State Response');
plot(t_out, x_out, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('State value');
title('Closed-loop State Response');
legend('x', 'x dot', 'theta1', 'theta1 dot', 'theta2', 'theta2 dot', 'Location', 'best');

saveas(gcf, fullfile(results_folder, 'closed_loop_states.png'));

%% Save Phase 4 results

save(fullfile(results_folder, 'phase4_closed_loop_data.mat'), ...
    'A', 'B', 'C', 'D', 'A_cl', 'B_cl', 'C_cl', 'D_cl', ...
    'model_info', 'Q', 'R', 'K', ...
    'phase3_closed_loop_poles', 'closed_loop_poles', 'closed_loop_stable', ...
    't_out', 'x_out', 'y_out', 'u_out', ...
    'cart_position', 'cart_velocity', ...
    'theta1', 'theta1_dot', 'theta2', 'theta2_dot', ...
    'theta1_deg', 'theta2_deg', 'metrics');

summary_file = fullfile(results_folder, 'closed_loop_summary.txt');
fid = fopen(summary_file, 'w');

if fid == -1
    warning('Could not create closed-loop summary text file. MAT file and plots were still saved.');
else
    fprintf(fid, 'Phase 4: MATLAB Closed-loop Simulation\n');
    fprintf(fid, '=======================================\n\n');
    fprintf(fid, 'State vector:\n');
    fprintf(fid, 'X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]\n\n');
    fprintf(fid, 'Control law:\n');
    fprintf(fid, 'u = -K*x\n\n');
    fprintf(fid, 'Q matrix:\n%s\n\n', mat2str(Q, 6));
    fprintf(fid, 'R matrix:\n%s\n\n', mat2str(R, 6));
    fprintf(fid, 'LQR gain K:\n%s\n\n', mat2str(K, 6));
    fprintf(fid, 'Closed-loop poles:\n%s\n\n', mat2str(closed_loop_poles, 6));
    fprintf(fid, 'Closed-loop stable: %d\n\n', closed_loop_stable);
    fprintf(fid, 'Max abs theta1: %.6f deg\n', max_abs_theta1_deg);
    fprintf(fid, 'Max abs theta2: %.6f deg\n', max_abs_theta2_deg);
    fprintf(fid, 'Max abs cart position: %.6f m\n', max_abs_cart_position);
    fprintf(fid, 'Max abs cart velocity: %.6f m/s\n', max_abs_cart_velocity);
    fprintf(fid, 'Max abs control force: %.6f N\n', max_abs_control_force);
    fprintf(fid, 'Final theta1: %.9f deg\n', final_theta1_deg);
    fprintf(fid, 'Final theta2: %.9f deg\n', final_theta2_deg);
    fprintf(fid, 'Final cart position: %.9f m\n', final_cart_position);
    fprintf(fid, 'Theta1 settling time threshold: +/-%.3f deg\n', angle_settle_threshold_deg);
    fprintf(fid, 'Theta2 settling time threshold: +/-%.3f deg\n', angle_settle_threshold_deg);
    fprintf(fid, 'Cart settling time threshold: +/-%.3f m\n', cart_settle_threshold_m);
    fprintf(fid, 'Theta1 settling time: %s\n', local_time_to_text(theta1_settling_time));
    fprintf(fid, 'Theta2 settling time: %s\n', local_time_to_text(theta2_settling_time));
    fprintf(fid, 'Cart settling time: %s\n', local_time_to_text(cart_settling_time));
    fprintf(fid, '\nNote:\n');
    fprintf(fid, 'This Phase 4 simulation does not apply input force saturation yet.\n');
    fprintf(fid, 'Force saturation and disturbance tests should be added in later phases.\n');
    fclose(fid);
end

disp(' ');
disp('===== Phase 4 Result =====');
disp('Closed-loop data saved to results/phase4_closed_loop/phase4_closed_loop_data.mat');
disp('Closed-loop summary saved to results/phase4_closed_loop/closed_loop_summary.txt');
disp('Closed-loop plot saved to results/phase4_closed_loop/closed_loop_response.png');
disp('Closed-loop state plot saved to results/phase4_closed_loop/closed_loop_states.png');
disp('Phase 4 script finished successfully.');

%% Local helper functions

function settling_time = local_settling_time(t, signal, threshold)
% Return the first time after which the absolute signal stays inside the
% threshold. If the signal never settles, return NaN.

settling_time = NaN;

for k = 1:length(t)
    if all(abs(signal(k:end)) <= threshold)
        settling_time = t(k);
        return;
    end
end

end

function text_value = local_time_to_text(time_value)
% Convert a settling time value to readable text for display and file logs.

if isnan(time_value)
    text_value = 'not settled within simulation time';
else
    text_value = [num2str(time_value, '%.4f'), ' s'];
end

end
