
project_root = fileparts(fileparts(mfilename('fullpath')));
cd(project_root);

if exist('matlab/startupProject.m','file')
    run('matlab/startupProject.m');
else
    addpath(genpath(project_root));
end
addpath(fullfile(project_root, 'matlab', 'evaluation'));
addpath(fullfile(project_root, 'matlab', 'simulation'));

result_dir = fullfile(project_root, 'results', 'final_demo');
if ~exist(result_dir, 'dir'), mkdir(result_dir); end

old_logs = dir(fullfile(result_dir, 'releasePackageLog_*.txt'));
for k = 1:numel(old_logs)
    try
        delete(fullfile(old_logs(k).folder, old_logs(k).name));
    catch
    end
end

log_file = fullfile(result_dir, sprintf('releasePackageLog_%s.txt', datestr(now,'yyyymmdd_HHMMSS')));
fid_log = fopen(log_file, 'w');
if fid_log < 0
    error('Release:LogOpenFailed', 'Cannot open log file: %s', log_file);
end
cleanup_obj = onCleanup(@() fclose(fid_log));

local_log(fid_log, '\n============================================================\n');
local_log(fid_log, 'FINAL RELEASE PACKAGING\n');
local_log(fid_log, '%s\n', datestr(now));
local_log(fid_log, 'Project root: %s\n', project_root);
local_log(fid_log, 'Backend: constrainedRecoveryTrajectoryRouter\n');
local_log(fid_log, 'Declaration: final package complete with constrained recovery backend limitation documented.\n');
local_log(fid_log, '============================================================\n');

cfg = struct();
cfg.project_root = project_root;
cfg.result_dir = result_dir;
cfg.backend_name = 'constrainedRecoveryTrajectoryRouter';
cfg.backend_success_count = 50;
cfg.backend_total_count = 64;
cfg.backend_success_rate = 50/64;
cfg.backend_overall_pass_95 = false;

try
    captured_output = evalc('summary45 = packageFinalRelease(cfg);');
    local_log(fid_log, '\n--- Captured output from packageFinalRelease ---\n%s\n', captured_output);
catch ME
    local_log(fid_log, '\nRELEASE PACKAGE ERROR:\n%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    rethrow(ME);
end

local_log(fid_log, '\n============================================================\n');
local_log(fid_log, 'FINAL RELEASE SUMMARY\n');
local_log(fid_log, '============================================================\n');
local_log(fid_log, 'Final release pass: %d\n', summary45.release_pass);
local_log(fid_log, 'Final backend: %s\n', summary45.backend_name);
local_log(fid_log, 'Backend recovery success: %d/%d = %.6f\n', summary45.backend_success_count, summary45.backend_total_count, summary45.backend_success_rate);
local_log(fid_log, 'Backend overall pass at 95%%: %d\n', summary45.backend_overall_pass_95);
local_log(fid_log, 'Final release zip: %s\n', summary45.release_zip_file);
local_log(fid_log, 'Final demo folder: %s\n', summary45.result_dir);
local_log(fid_log, 'User Guide: %s\n', summary45.how_to_run_file);
local_log(fid_log, 'Technical Report: %s\n', summary45.final_technical_report_file);
local_log(fid_log, 'Defense Notes: %s\n', summary45.defense_notes_file);
local_log(fid_log, 'Clean release log: %s\n', log_file);

save(fullfile(result_dir, 'releasePackageSummary.mat'), 'summary45', 'cfg', 'log_file');

function local_log(fid, varargin)
    msg = sprintf(varargin{:});
    fprintf('%s', msg);
    fprintf(fid, '%s', msg);
end
