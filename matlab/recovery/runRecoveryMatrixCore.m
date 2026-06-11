function summary = runRecoveryMatrixCore(user_cfg)
    if nargin < 1 || isempty(user_cfg)
        user_cfg = struct();
    end
    project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cfg = defaultRecoveryConfig(project_root, user_cfg);
    cfg = prepareRecoveryRun(cfg);
    summary = RecoveryEngine.runRecoveryMatrixEngine(cfg);
end
