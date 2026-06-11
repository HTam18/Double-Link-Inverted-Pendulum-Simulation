thisFile = mfilename('fullpath');
matlabRoot = fileparts(thisFile);
projectRoot = fileparts(matlabRoot);
activeDirs = {
    matlabRoot
    fullfile(matlabRoot, 'params')
    fullfile(matlabRoot, 'dynamics')
    fullfile(matlabRoot, 'config')
    fullfile(matlabRoot, 'control')
    fullfile(matlabRoot, 'evaluation')
    fullfile(matlabRoot, 'recovery')
    fullfile(matlabRoot, 'simulation')
};
for k = 1:numel(activeDirs)
    if isfolder(activeDirs{k})
        addpath(genpath(activeDirs{k}));
    else
        warning('startupProject:MissingFolder', 'Missing active MATLAB folder: %s', activeDirs{k});
    end
end
fprintf('Double Link Pendulum MATLAB backend path configured.\n');
fprintf('Project root: %s\n', projectRoot);
