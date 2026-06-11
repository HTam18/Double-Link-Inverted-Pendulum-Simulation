project_root = fileparts(fileparts(mfilename('fullpath')));
cd(project_root);
run(fullfile(project_root, 'matlab', 'startupProject.m'));
resume = false;
if exist('RECOVERY_RESUME', 'var') && ~isempty(RECOVERY_RESUME)
    resume = logical(RECOVERY_RESUME);
end
if ~resume
    run('scripts/internal/resetRecoveryRuntimeState.m');
end
cfg = defaultRecoveryConfig(project_root, struct('resume', resume));
summaryRecovery = runRecoveryMatrixCore(cfg);
disp(summaryRecovery);
