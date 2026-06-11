function paths = recoveryOutputPaths(project_root)
    if nargin < 1 || isempty(project_root)
        project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    end
    paths = struct();
    paths.result_dir = fullfile(project_root, 'results', 'recovery');
    paths.baseline_dir = fullfile(project_root, 'results', 'recoveryBaseline');
    paths.switching_dir = fullfile(project_root, 'results', 'switching');
    paths.result_mat = fullfile(paths.result_dir, 'recoveryResults.mat');
    paths.all_metrics_csv = fullfile(paths.result_dir, 'recoveryAllMetrics.csv');
    paths.case_metrics_csv = fullfile(paths.result_dir, 'recoveryCaseMetrics.csv');
    paths.gui_cases_csv = fullfile(paths.result_dir, 'recoveryGuiCases.csv');
    paths.switching_mat = fullfile(paths.switching_dir, 'switchingResults.mat');
end
