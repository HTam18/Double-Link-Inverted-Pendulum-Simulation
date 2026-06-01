function plot_robustness_summary(result_file)
% PLOT_ROBUSTNESS_SUMMARY Plot Phase 31 Monte Carlo robustness metrics.
%
% Usage:
%   plot_robustness_summary('results/phase31_monte_carlo/phase31_monte_carlo_results.mat')
%
% The function saves figures next to the supplied result file.

    if nargin < 1 || isempty(result_file)
        this_file = mfilename('fullpath');
        visualization_dir = fileparts(this_file);
        matlab_root = fileparts(visualization_dir);
        project_root = fileparts(matlab_root);
        result_file = fullfile(project_root, 'results', 'phase31_monte_carlo', 'phase31_monte_carlo_results.mat');
    end
    if ~exist(result_file, 'file')
        error('plot_robustness_summary:MissingResult', 'Missing result file: %s', result_file);
    end

    S = load(result_file);
    run_metrics = S.run_metrics;
    result_dir = fileparts(result_file);

    idx = [run_metrics.run_index];
    success = double([run_metrics.success]);
    final_angle = [run_metrics.final_angle_error_rad];
    final_velocity = [run_metrics.final_velocity_norm];
    max_x = [run_metrics.max_abs_x_m];
    sat = [run_metrics.saturation_fraction];

    fig = figure('Name', 'Phase 31 success by run', 'Visible', 'off');
    stem(idx, success, 'filled');
    ylim([-0.1, 1.1]);
    xlabel('Monte Carlo run');
    ylabel('success flag');
    title('Phase 31 Monte Carlo success flag');
    grid on;
    saveas(fig, fullfile(result_dir, 'phase31_success_by_run.png'));
    close(fig);

    fig = figure('Name', 'Phase 31 final angle error', 'Visible', 'off');
    plot(idx, final_angle, 'o-');
    xlabel('Monte Carlo run');
    ylabel('final max angle error (rad)');
    title('Phase 31 final angle error');
    grid on;
    saveas(fig, fullfile(result_dir, 'phase31_final_angle_error.png'));
    close(fig);

    fig = figure('Name', 'Phase 31 final velocity norm', 'Visible', 'off');
    plot(idx, final_velocity, 'o-');
    xlabel('Monte Carlo run');
    ylabel('final velocity norm');
    title('Phase 31 final velocity norm');
    grid on;
    saveas(fig, fullfile(result_dir, 'phase31_final_velocity_norm.png'));
    close(fig);

    fig = figure('Name', 'Phase 31 constraint metrics', 'Visible', 'off');
    plot(idx, max_x, 'o-');
    hold on;
    plot(idx, sat, 's-');
    xlabel('Monte Carlo run');
    ylabel('metric value');
    title('Phase 31 constraint metrics');
    legend({'max |x| (m)', 'saturation fraction'}, 'Location', 'best');
    grid on;
    saveas(fig, fullfile(result_dir, 'phase31_constraints.png'));
    close(fig);

    reasons = string({run_metrics.failure_reason});
    unique_reasons = unique(reasons);
    counts = zeros(numel(unique_reasons), 1);
    for i = 1:numel(unique_reasons)
        counts(i) = sum(reasons == unique_reasons(i));
    end
    fig = figure('Name', 'Phase 31 failure reasons', 'Visible', 'off');
    bar(counts);
    set(gca, 'XTick', 1:numel(unique_reasons), 'XTickLabel', cellstr(unique_reasons));
    xtickangle(30);
    ylabel('count');
    title('Phase 31 failure reason counts');
    grid on;
    saveas(fig, fullfile(result_dir, 'phase31_failure_reasons.png'));
    close(fig);
end
