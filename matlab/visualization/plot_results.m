function fig = plot_results(result, save_path)
% PLOT_RESULTS Standard Phase 21 plot for MATLAB simulation results.
%
% Signals shown:
%   - cart position x
%   - link angles theta1 and theta2
%   - velocities
%   - u_cmd and u_actual
%   - controller mode when available
%
% Usage:
%   fig = plot_results(result);
%   fig = plot_results(result, 'results/phase21_open_loop/open_loop_plot.png');

    if nargin < 2
        save_path = '';
    end

    required = {'t', 'state', 'u_cmd', 'u_actual'};
    for i = 1:numel(required)
        if ~isfield(result, required{i})
            error('plot_results:MissingField', 'result.%s is required.', required{i});
        end
    end

    t = result.t(:);
    X = result.state;
    if size(X, 2) ~= 6
        error('plot_results:InvalidStateHistory', 'result.state must be N x 6.');
    end

    fig = figure('Name', 'MATLAB Simulation Results', 'Color', 'w');
    tiledlayout(5, 1);

    nexttile;
    plot(t, X(:, 1), 'LineWidth', 1.2);
    grid on;
    ylabel('x [m]');
    title('Double Link Pendulum MATLAB simulation result');

    nexttile;
    plot(t, X(:, 3), 'LineWidth', 1.2);
    hold on;
    plot(t, X(:, 5), 'LineWidth', 1.2);
    grid on;
    ylabel('angle [rad]');
    legend('theta1', 'theta2', 'Location', 'best');

    nexttile;
    plot(t, X(:, 2), 'LineWidth', 1.2);
    hold on;
    plot(t, X(:, 4), 'LineWidth', 1.2);
    plot(t, X(:, 6), 'LineWidth', 1.2);
    grid on;
    ylabel('velocity');
    legend('x dot', 'theta1 dot', 'theta2 dot', 'Location', 'best');

    nexttile;
    plot(t, result.u_cmd(:), 'LineWidth', 1.2);
    hold on;
    plot(t, result.u_actual(:), '--', 'LineWidth', 1.2);
    grid on;
    ylabel('force [N]');
    legend('u cmd', 'u actual', 'Location', 'best');

    nexttile;
    if isfield(result, 'mode')
        [mode_id, mode_names] = local_mode_to_numeric(result.mode);
        stairs(t, mode_id, 'LineWidth', 1.2);
        grid on;
        yticks(1:numel(mode_names));
        yticklabels(mode_names);
        ylabel('mode');
    else
        plot(t, zeros(size(t)), 'LineWidth', 1.2);
        grid on;
        ylabel('mode');
    end
    xlabel('time [s]');

    if ~isempty(save_path)
        save_dir = fileparts(save_path);
        if ~isempty(save_dir) && ~exist(save_dir, 'dir')
            mkdir(save_dir);
        end
        saveas(fig, save_path);
    end
end

function [mode_id, mode_names] = local_mode_to_numeric(mode_values)
    mode_values = string(mode_values(:));
    mode_names = unique(mode_values, 'stable');
    mode_id = zeros(size(mode_values));
    for i = 1:numel(mode_names)
        mode_id(mode_values == mode_names(i)) = i;
    end
end
