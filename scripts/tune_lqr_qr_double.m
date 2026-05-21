% tune_lqr_qr_double.m
% Phase 4.5 - Q/R tuning and force check for double link inverted pendulum
% This script compares several LQR Q/R choices before moving to Simulink.

clear; clc; close all;

%% Robust path setup
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
addpath(script_dir);

%% Load parameters and state-space model
run(fullfile(script_dir, 'parameters_double_link.m'));

% parameters_double_link.m contains clear; therefore rebuild path variables.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
project_root = fileparts(script_dir);
addpath(script_dir);

[A, B, C, D, model_info] = state_space_model_double(mc, m1, m2, L1, L2, lc1, lc2, I1, I2, bc, b1, b2, g);

%% Result folder
result_dir = fullfile(project_root, 'results', 'phase45_lqr_tuning');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end

fprintf('\n===== Phase 4.5: Q/R Tuning and Force Check =====\n');
fprintf('Script file: %s\n', mfilename('fullpath'));
fprintf('Project root: %s\n', project_root);
fprintf('Result folder: %s\n', result_dir);

%% Basic checks
n_states = size(A, 1);
Co = ctrb(A, B);
rank_Co = rank(Co);

fprintf('\n===== Controllability Check =====\n');
fprintf('Controllability rank: %d\n', rank_Co);
fprintf('Number of states: %d\n', n_states);

if rank_Co ~= n_states
    error('The system is not controllable. Stop Phase 4.5 tuning.');
end

if ~exist('x0', 'var')
    error('Initial condition x0 was not found in parameters_double_link.m');
end

if ~exist('u_max', 'var')
    warning('u_max was not found. Using u_max = 10 N as default.');
    u_max = 10;
end

if ~exist('sim_time', 'var')
    sim_time = 10;
end

if ~exist('dt', 'var')
    dt = 0.01;
end

t = 0:dt:sim_time;
zero_input = zeros(size(t));

%% Candidate Q/R sets
% Updated parameter set only.
% The tuning logic below is kept unchanged.
% The new set adds finer candidates around C3 and C4,
% because the first Phase 4.5 result showed that C5 reduced force
% but made the cart travel too far.

candidates = struct([]);

% Baseline from Phase 3 and Phase 4.
candidates(1).name = 'C1_baseline_phase3';
candidates(1).Qdiag = [5, 1, 80, 5, 80, 5];
candidates(1).R = 0.1;

% Original coarse tuning candidates.
candidates(2).name = 'C2_medium_force';
candidates(2).Qdiag = [5, 1, 60, 4, 60, 4];
candidates(2).R = 0.5;

candidates(3).name = 'C3_high_R';
candidates(3).Qdiag = [3, 1, 50, 3, 50, 3];
candidates(3).R = 1.0;

candidates(4).name = 'C4_conservative';
candidates(4).Qdiag = [2, 1, 40, 2, 40, 2];
candidates(4).R = 2.0;

candidates(5).name = 'C5_very_conservative';
candidates(5).Qdiag = [1, 0.5, 30, 2, 30, 2];
candidates(5).R = 5.0;

% Fine search around C3 and C4.
% Goal: reduce force compared with C3 while avoiding the large cart motion of C5.
candidates(6).name = 'C6_C3_R12';
candidates(6).Qdiag = [3, 1, 50, 3, 50, 3];
candidates(6).R = 1.2;

candidates(7).name = 'C7_C3_lower_angle_R15';
candidates(7).Qdiag = [3, 1, 45, 3, 45, 3];
candidates(7).R = 1.5;

candidates(8).name = 'C8_C3_lower_angle_R18';
candidates(8).Qdiag = [3, 1, 40, 3, 40, 3];
candidates(8).R = 1.8;

candidates(9).name = 'C9_C4_R25';
candidates(9).Qdiag = [2, 1, 40, 2, 40, 2];
candidates(9).R = 2.5;

candidates(10).name = 'C10_C4_lower_angle_R30';
candidates(10).Qdiag = [2, 1, 35, 2, 35, 2];
candidates(10).R = 3.0;

candidates(11).name = 'C11_C4_lower_velocity_R35';
candidates(11).Qdiag = [2, 0.8, 35, 2, 35, 2];
candidates(11).R = 3.5;

% Extra candidates with higher cart position penalty.
% Goal: keep cart travel closer to C3 while still reducing force.
candidates(12).name = 'C12_cart_weight_5_R20';
candidates(12).Qdiag = [5, 1, 40, 2, 40, 2];
candidates(12).R = 2.0;

candidates(13).name = 'C13_cart_weight_8_R20';
candidates(13).Qdiag = [8, 1, 40, 2, 40, 2];
candidates(13).R = 2.0;

candidates(14).name = 'C14_cart_weight_10_R30';
candidates(14).Qdiag = [10, 1, 35, 2, 35, 2];
candidates(14).R = 3.0;

% Extra middle candidates.
% Goal: find a practical balance near 12 to 16 N without letting cart motion grow too much.
candidates(15).name = 'C15_balanced_cart_angle_R25';
candidates(15).Qdiag = [6, 1, 35, 2.5, 35, 2.5];
candidates(15).R = 2.5;

candidates(16).name = 'C16_balanced_cart_angle_R35';
candidates(16).Qdiag = [8, 1, 30, 2.5, 30, 2.5];
candidates(16).R = 3.5;

n_candidates = numel(candidates);
results = struct([]);

fprintf('\nTesting %d Q/R candidates...\n', n_candidates);
fprintf('Force limit u_max = %.4f N\n', u_max);

%% Simulation options
ode_options = odeset('RelTol', 1e-7, 'AbsTol', 1e-9);

for i = 1:n_candidates
    Q = diag(candidates(i).Qdiag);
    R = candidates(i).R;

    fprintf('\n----------------------------------------\n');
    fprintf('Candidate %d: %s\n', i, candidates(i).name);
    fprintf('Qdiag = [%s]\n', num2str(candidates(i).Qdiag));
    fprintf('R = %.4f\n', R);

    K = lqr(A, B, Q, R);
    A_cl = A - B*K;
    closed_loop_poles = eig(A_cl);
    stable = all(real(closed_loop_poles) < 0);

    sys_cl = ss(A_cl, zeros(n_states, 1), eye(n_states), zeros(n_states, 1));
    [x_no_sat, t_out] = lsim(sys_cl, zero_input, t, x0);
    u_no_sat = -(K * x_no_sat.').';

    % Saturated closed-loop simulation with ode45:
    % x_dot = A*x + B*sat(-K*x)
    sat_fun = @(u) max(min(u, u_max), -u_max);
    ode_fun = @(time_now, x_now) A*x_now + B*sat_fun(-K*x_now);
    [t_sat, x_sat] = ode45(ode_fun, t, x0, ode_options);
    u_sat = zeros(length(t_sat), 1);
    for k = 1:length(t_sat)
        u_sat(k) = sat_fun(-K * x_sat(k, :).');
    end

    metrics_no_sat = local_compute_metrics(t_out, x_no_sat, u_no_sat);
    metrics_sat = local_compute_metrics(t_sat, x_sat, u_sat);

    force_ratio = metrics_no_sat.max_abs_control_force_N / u_max;
    force_ok_no_sat = metrics_no_sat.max_abs_control_force_N <= u_max;

    sat_final_ok = abs(metrics_sat.final_theta1_deg) <= 0.5 && ...
                   abs(metrics_sat.final_theta2_deg) <= 0.5 && ...
                   abs(metrics_sat.final_cart_position_m) <= 0.05;

    fprintf('Stable closed-loop poles: %d\n', stable);
    fprintf('Max force demand without saturation: %.4f N\n', metrics_no_sat.max_abs_control_force_N);
    fprintf('Force ratio compared with u_max: %.4f\n', force_ratio);
    fprintf('Theta1 settling time no saturation: %.4f s\n', metrics_no_sat.theta1_settling_time_s);
    fprintf('Theta2 settling time no saturation: %.4f s\n', metrics_no_sat.theta2_settling_time_s);
    fprintf('Max cart position no saturation: %.4f m\n', metrics_no_sat.max_abs_cart_position_m);
    fprintf('Saturated final theta1: %.6f deg\n', metrics_sat.final_theta1_deg);
    fprintf('Saturated final theta2: %.6f deg\n', metrics_sat.final_theta2_deg);
    fprintf('Saturated final cart position: %.6f m\n', metrics_sat.final_cart_position_m);
    fprintf('Saturated final OK: %d\n', sat_final_ok);

    results(i).name = candidates(i).name;
    results(i).Qdiag = candidates(i).Qdiag;
    results(i).R = R;
    results(i).K = K;
    results(i).closed_loop_poles = closed_loop_poles;
    results(i).stable = stable;
    results(i).force_ok_no_sat = force_ok_no_sat;
    results(i).force_ratio_to_umax = force_ratio;
    results(i).sat_final_ok = sat_final_ok;
    results(i).t_no_sat = t_out;
    results(i).x_no_sat = x_no_sat;
    results(i).u_no_sat = u_no_sat;
    results(i).t_sat = t_sat;
    results(i).x_sat = x_sat;
    results(i).u_sat = u_sat;
    results(i).metrics_no_sat = metrics_no_sat;
    results(i).metrics_sat = metrics_sat;
end

%% Build summary table
names = strings(n_candidates, 1);
R_values = zeros(n_candidates, 1);
max_force = zeros(n_candidates, 1);
force_ratio_values = zeros(n_candidates, 1);
max_theta1 = zeros(n_candidates, 1);
max_theta2 = zeros(n_candidates, 1);
max_cart = zeros(n_candidates, 1);
theta1_settle = zeros(n_candidates, 1);
theta2_settle = zeros(n_candidates, 1);
cart_settle = zeros(n_candidates, 1);
final_theta1_sat = zeros(n_candidates, 1);
final_theta2_sat = zeros(n_candidates, 1);
final_cart_sat = zeros(n_candidates, 1);
stable_values = false(n_candidates, 1);
force_ok_values = false(n_candidates, 1);
sat_final_ok_values = false(n_candidates, 1);

for i = 1:n_candidates
    names(i) = string(results(i).name);
    R_values(i) = results(i).R;
    max_force(i) = results(i).metrics_no_sat.max_abs_control_force_N;
    force_ratio_values(i) = results(i).force_ratio_to_umax;
    max_theta1(i) = results(i).metrics_no_sat.max_abs_theta1_deg;
    max_theta2(i) = results(i).metrics_no_sat.max_abs_theta2_deg;
    max_cart(i) = results(i).metrics_no_sat.max_abs_cart_position_m;
    theta1_settle(i) = results(i).metrics_no_sat.theta1_settling_time_s;
    theta2_settle(i) = results(i).metrics_no_sat.theta2_settling_time_s;
    cart_settle(i) = results(i).metrics_no_sat.cart_settling_time_s;
    final_theta1_sat(i) = results(i).metrics_sat.final_theta1_deg;
    final_theta2_sat(i) = results(i).metrics_sat.final_theta2_deg;
    final_cart_sat(i) = results(i).metrics_sat.final_cart_position_m;
    stable_values(i) = results(i).stable;
    force_ok_values(i) = results(i).force_ok_no_sat;
    sat_final_ok_values(i) = results(i).sat_final_ok;
end

summary_table = table(names, R_values, max_force, force_ratio_values, ...
    max_theta1, max_theta2, max_cart, theta1_settle, theta2_settle, cart_settle, ...
    final_theta1_sat, final_theta2_sat, final_cart_sat, stable_values, force_ok_values, sat_final_ok_values, ...
    'VariableNames', {'Candidate', 'R', 'MaxForceNoSat_N', 'ForceRatioToUmax', ...
    'MaxTheta1_deg', 'MaxTheta2_deg', 'MaxCart_m', 'Theta1Settling_s', 'Theta2Settling_s', 'CartSettling_s', ...
    'FinalTheta1Sat_deg', 'FinalTheta2Sat_deg', 'FinalCartSat_m', 'Stable', 'ForceOkNoSat', 'SatFinalOk'});

fprintf('\n===== Q/R Tuning Summary Table =====\n');
disp(summary_table);

%% Choose recommended candidate
valid_idx = find(stable_values & sat_final_ok_values);
if ~isempty(valid_idx)
    % Prefer lower force, then lower angle settling time.
    score = force_ratio_values(valid_idx) + 0.05 * theta1_settle(valid_idx) + 0.05 * theta2_settle(valid_idx) + 0.5 * max_cart(valid_idx);
    [~, best_local_idx] = min(score);
    recommended_idx = valid_idx(best_local_idx);
else
    stable_idx = find(stable_values);
    if isempty(stable_idx)
        error('No stable candidate found. Check model and Q/R choices.');
    end
    [~, best_local_idx] = min(force_ratio_values(stable_idx));
    recommended_idx = stable_idx(best_local_idx);
end

recommended = results(recommended_idx);

fprintf('\n===== Recommended Candidate =====\n');
fprintf('Recommended candidate: %s\n', recommended.name);
fprintf('Recommended R: %.4f\n', recommended.R);
fprintf('Recommended Qdiag: [%s]\n', num2str(recommended.Qdiag));
fprintf('Recommended max force demand without saturation: %.4f N\n', recommended.metrics_no_sat.max_abs_control_force_N);
fprintf('Recommended force ratio compared with u_max: %.4f\n', recommended.force_ratio_to_umax);
fprintf('Recommended saturated final OK: %d\n', recommended.sat_final_ok);

%% Save data
mat_path = fullfile(result_dir, 'phase45_lqr_tuning_data.mat');
csv_path = fullfile(result_dir, 'phase45_lqr_tuning_summary.csv');
txt_path = fullfile(result_dir, 'phase45_lqr_tuning_summary.txt');
comparison_plot_path = fullfile(result_dir, 'phase45_qr_tuning_comparison.png');
recommended_plot_path = fullfile(result_dir, 'phase45_recommended_response.png');

save(mat_path, 'A', 'B', 'C', 'D', 'model_info', 'x0', 'u_max', 'sim_time', 'dt', ...
    'candidates', 'results', 'summary_table', 'recommended', 'recommended_idx');

writetable(summary_table, csv_path);

fid = fopen(txt_path, 'w');
fprintf(fid, 'Phase 4.5 - Q/R Tuning and Force Check\n');
fprintf(fid, '========================================\n\n');
fprintf(fid, 'Force limit u_max: %.4f N\n\n', u_max);
fprintf(fid, 'Recommended candidate: %s\n', recommended.name);
fprintf(fid, 'Recommended Qdiag: [%s]\n', num2str(recommended.Qdiag));
fprintf(fid, 'Recommended R: %.4f\n', recommended.R);
fprintf(fid, 'Max force without saturation: %.4f N\n', recommended.metrics_no_sat.max_abs_control_force_N);
fprintf(fid, 'Force ratio to u_max: %.4f\n', recommended.force_ratio_to_umax);
fprintf(fid, 'Saturated final OK: %d\n\n', recommended.sat_final_ok);
fprintf(fid, 'Summary table was saved to: %s\n', csv_path);
fclose(fid);

%% Plot comparison
figure('Name', 'Phase 4.5 Q/R Tuning Comparison', 'Color', 'w');

subplot(2, 2, 1);
bar(max_force);
yline(u_max, '--', 'u_{max}');
grid on;
xticks(1:n_candidates);
xticklabels(names);
xtickangle(25);
ylabel('Max force no sat (N)');
title('Control force demand');

subplot(2, 2, 2);
bar([theta1_settle, theta2_settle]);
grid on;
xticks(1:n_candidates);
xticklabels(names);
xtickangle(25);
ylabel('Settling time (s)');
legend('theta1', 'theta2', 'Location', 'best');
title('Angle settling time');

subplot(2, 2, 3);
bar(max_cart);
grid on;
xticks(1:n_candidates);
xticklabels(names);
xtickangle(25);
ylabel('Max cart position (m)');
title('Cart movement');

subplot(2, 2, 4);
bar([abs(final_theta1_sat), abs(final_theta2_sat)]);
grid on;
xticks(1:n_candidates);
xticklabels(names);
xtickangle(25);
ylabel('Final angle with saturation (deg)');
legend('theta1', 'theta2', 'Location', 'best');
title('Saturated final angle error');

saveas(gcf, comparison_plot_path);

%% Plot recommended response
figure('Name', 'Phase 4.5 Recommended Candidate Response', 'Color', 'w');

subplot(3, 1, 1);
plot(recommended.t_no_sat, rad2deg(recommended.x_no_sat(:, 3)), 'LineWidth', 1.2); hold on;
plot(recommended.t_sat, rad2deg(recommended.x_sat(:, 3)), '--', 'LineWidth', 1.2);
grid on;
ylabel('theta1 (deg)');
legend('No saturation', 'With saturation', 'Location', 'best');
title(['Recommended candidate: ', recommended.name]);

subplot(3, 1, 2);
plot(recommended.t_no_sat, rad2deg(recommended.x_no_sat(:, 5)), 'LineWidth', 1.2); hold on;
plot(recommended.t_sat, rad2deg(recommended.x_sat(:, 5)), '--', 'LineWidth', 1.2);
grid on;
ylabel('theta2 (deg)');
legend('No saturation', 'With saturation', 'Location', 'best');

subplot(3, 1, 3);
plot(recommended.t_no_sat, recommended.u_no_sat, 'LineWidth', 1.2); hold on;
plot(recommended.t_sat, recommended.u_sat, '--', 'LineWidth', 1.2);
yline(u_max, ':');
yline(-u_max, ':');
grid on;
ylabel('u (N)');
xlabel('Time (s)');
legend('No saturation', 'With saturation', 'u limits', 'Location', 'best');

saveas(gcf, recommended_plot_path);

fprintf('\n===== Phase 4.5 Result =====\n');
fprintf('Tuning data saved to %s\n', mat_path);
fprintf('Tuning summary CSV saved to %s\n', csv_path);
fprintf('Tuning summary text saved to %s\n', txt_path);
fprintf('Comparison plot saved to %s\n', comparison_plot_path);
fprintf('Recommended response plot saved to %s\n', recommended_plot_path);
fprintf('Phase 4.5 script finished successfully.\n');

%% Local function
function metrics = local_compute_metrics(t, x, u)
    theta1_deg = rad2deg(x(:, 3));
    theta2_deg = rad2deg(x(:, 5));
    cart_position = x(:, 1);
    cart_velocity = x(:, 2);

    metrics.max_abs_theta1_deg = max(abs(theta1_deg));
    metrics.max_abs_theta2_deg = max(abs(theta2_deg));
    metrics.max_abs_cart_position_m = max(abs(cart_position));
    metrics.max_abs_cart_velocity_m_per_s = max(abs(cart_velocity));
    metrics.max_abs_control_force_N = max(abs(u));

    metrics.final_theta1_deg = theta1_deg(end);
    metrics.final_theta2_deg = theta2_deg(end);
    metrics.final_cart_position_m = cart_position(end);

    metrics.theta1_settling_time_s = local_settling_time(t, theta1_deg, 0.5);
    metrics.theta2_settling_time_s = local_settling_time(t, theta2_deg, 0.5);
    metrics.cart_settling_time_s = local_settling_time(t, cart_position, 0.02);
end

function settling_time = local_settling_time(t, signal, band)
    abs_signal = abs(signal);
    settling_time = NaN;
    for idx = 1:length(t)
        if all(abs_signal(idx:end) <= band)
            settling_time = t(idx);
            return;
        end
    end
end
