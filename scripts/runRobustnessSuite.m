project_root = fileparts(fileparts(mfilename('fullpath')));
cd(project_root);
run(fullfile(project_root, 'matlab', 'startupProject.m'));
cfg = struct();
cfg.project_root = project_root;
cfg.result_dir = fullfile(project_root, 'results', 'robustness');
summaryRobustness = runRobustnessSuiteCore(cfg);
disp(summaryRobustness);
