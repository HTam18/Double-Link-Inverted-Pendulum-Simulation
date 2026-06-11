function cfg = prepareRecoveryRun(cfg)
    if nargin < 1 || isempty(cfg)
        cfg = defaultRecoveryConfig();
    end
    if ~isfield(cfg, 'project_root')
        cfg.project_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    end
    paths = recoveryOutputPaths(cfg.project_root);
    if ~exist(paths.result_dir, 'dir')
        mkdir(paths.result_dir);
    end
    params = loadParams();
    generateHardStateLibrary(params, cfg);
    cfg.recovery_paths = paths;
end
