project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
result_dir = fullfile(project_root, 'results', 'recovery');
if ~exist(result_dir, 'dir')
    mkdir(result_dir);
end
files = {
    'recoveryResults.mat'
    'recoveryAllMetrics.csv'
    'recoveryCaseMetrics.csv'
    'afterSwitchingMetrics.csv'
    'recoverySummary.txt'
    'recoverySummary.csv'
    'plannerVariantUsage.csv'
    'recoveryMatrixSummary.txt'
};
for k = 1:numel(files)
    path = fullfile(result_dir, files{k});
    if isfile(path)
        delete(path);
    end
end
