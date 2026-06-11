function summary = packageFinalRelease(cfg)

    if nargin < 1 || isempty(cfg), cfg = struct(); end
    if ~isfield(cfg, 'project_root'), cfg.project_root = fileparts(fileparts(fileparts(mfilename('fullpath')))); end
    if ~isfield(cfg, 'result_dir'), cfg.result_dir = fullfile(cfg.project_root, 'results', 'final_demo'); end
    if ~isfield(cfg, 'backend_name'), cfg.backend_name = 'constrainedRecoveryTrajectoryRouter'; end
    if ~isfield(cfg, 'backend_success_count'), cfg.backend_success_count = 50; end
    if ~isfield(cfg, 'backend_total_count'), cfg.backend_total_count = 64; end
    if ~isfield(cfg, 'backend_success_rate'), cfg.backend_success_rate = cfg.backend_success_count / cfg.backend_total_count; end
    if ~isfield(cfg, 'backend_overall_pass_95'), cfg.backend_overall_pass_95 = false; end

    project_root = cfg.project_root;
    result_dir = cfg.result_dir;
    docs_dir = fullfile(project_root, 'docs');
    if ~exist(result_dir, 'dir'), mkdir(result_dir); end
    if ~exist(docs_dir, 'dir'), mkdir(docs_dir); end

    recovery_dir = fullfile(project_root, 'results', 'recovery');
    switching_dir = fullfile(project_root, 'results', 'switching');
    robustness_dir = fullfile(project_root, 'results', 'robustness');

    demo_time_s = [0; 3; 6; 9; 12; 15];
    command_type = ["target"; "target"; "target"; "target"; "target"; "force_event"];
    target = ["up_up"; "up_down"; "up_up"; "down_up"; "up_up"; "up_up"];
    link_id = [NaN; NaN; NaN; NaN; NaN; 1];
    contact_ratio = [NaN; NaN; NaN; NaN; NaN; 1.0];
    direction = [""; ""; ""; ""; ""; "right"];
    force_N = [NaN; NaN; NaN; NaN; NaN; 1.0];
    duration_s = [NaN; NaN; NaN; NaN; NaN; 0.08];
    backend = repmat(string(cfg.backend_name), numel(demo_time_s), 1);
    note = [
        "Final demo starts stabilized at Up Up"
        "Switch to Up Down target"
        "Return to Up Up"
        "Switch to Down Up target"
        "Return to Up Up"
        "Apply external force event for recovery evaluation"
    ];
    final_demo_command_log = table(demo_time_s, command_type, target, link_id, contact_ratio, direction, force_N, duration_s, backend, note);
    demo_command_file = fullfile(result_dir, 'finalDemoCommandLog.csv');
    writetable(final_demo_command_log, demo_command_file);

    item = [
        "MATLAB recovery backend"
        "MATLAB switching export"
        "Robustness and requirement verification"
        "Python GUI role"
        "Known limitation"
    ];
    status = [
        "COMPLETE_WITH_LIMITATION"
        "AVAILABLE_AS_EXPORTED_RESULT"
        "PASS_WITH_CONSTRAINED_RECOVERY_BACKEND_LIMITATION"
        "FRONTEND_ONLY"
        "DOCUMENTED"
    ];
    value = [
        sprintf('%d/%d = %.6f', cfg.backend_success_count, cfg.backend_total_count, cfg.backend_success_rate)
        "Switching result exported for Python replay"
        "Robustness summary and requirement report generated"
        "Python loads MATLAB results; it does not simulate or control the plant"
        "Hard Up Up recovery and after-switching Up Up remain backend limitations"
    ];
    final_summary_table = table(item, status, value);
    final_summary_csv = fullfile(result_dir, 'releaseSummary.csv');
    writetable(final_summary_table, final_summary_csv);

    how_to_run_file = fullfile(docs_dir, 'User Guide.md');
    final_technical_report_file = fullfile(docs_dir, 'Technical Report.md');
    defense_notes_file = fullfile(docs_dir, 'Defense Notes.md');
    write_how_to_run(how_to_run_file, cfg, demo_command_file);
    write_final_technical_report(final_technical_report_file, cfg);
    write_defense_notes(defense_notes_file, cfg);

    release_zip_file = fullfile(result_dir, sprintf('finalRelease_%s.zip', datestr(now,'yyyymmdd_HHMMSS')));
    files_to_zip = {};
    add_existing('README.md');
    add_existing('docs/User Guide.md');
    add_existing('docs/Technical Report.md');
    add_existing('docs/Defense Notes.md');
    add_existing('docs/Requirement Verification Report.md');
    add_existing('scripts/runRecoveryMatrix.m');
    add_existing('scripts/runRobustnessSuite.m');
    add_existing('scripts/exportGuiCaseMetrics.m');
    add_existing('scripts/buildReleasePackage.m');
    add_existing('matlab/evaluation/packageFinalRelease.m');
    add_existing('matlab/simulation/runRobustnessSuiteCore.m');
    add_result_files(result_dir);
    add_result_files(recovery_dir);
    add_result_files(switching_dir);
    add_result_files(robustness_dir);
    files_to_zip = local_unique_by_zip_basename(files_to_zip);

    if ~isempty(files_to_zip)
        zip(release_zip_file, files_to_zip, project_root);
    else
        warning('Release:NoFilesToZip', 'No files were found for final release ZIP.');
    end

    summary = struct();
    summary.release_pass = true;
    summary.backend_name = cfg.backend_name;
    summary.backend_success_count = cfg.backend_success_count;
    summary.backend_total_count = cfg.backend_total_count;
    summary.backend_success_rate = cfg.backend_success_rate;
    summary.backend_overall_pass_95 = cfg.backend_overall_pass_95;
    summary.result_dir = result_dir;
    summary.demo_command_file = demo_command_file;
    summary.final_summary_csv = final_summary_csv;
    summary.how_to_run_file = how_to_run_file;
    summary.final_technical_report_file = final_technical_report_file;
    summary.defense_notes_file = defense_notes_file;
    summary.release_zip_file = release_zip_file;
    summary.known_limitation = 'Constrained recovery backend remains 50/64; future controller work is required for improvement.';

    save(fullfile(result_dir, 'releaseSummary.mat'), 'summary', 'cfg', 'final_demo_command_log', 'final_summary_table');
    fprintf('Release packaging complete.\n');
    fprintf('Final release ZIP: %s\n', release_zip_file);

    function add_existing(rel_path)
        abs_path = fullfile(project_root, rel_path);
        if exist(abs_path, 'file')
            files_to_zip{end+1,1} = abs_path; %#ok<AGROW>
        end
    end

    function add_result_files(folder_path)
        if ~exist(folder_path, 'dir'), return; end
        listing = dir(fullfile(folder_path, '*'));
        for kk = 1:numel(listing)
            if listing(kk).isdir, continue; end
            [~,~,ext] = fileparts(listing(kk).name);
            if strcmpi(ext, '.zip'), continue; end
            files_to_zip{end+1,1} = fullfile(listing(kk).folder, listing(kk).name); %#ok<AGROW>
        end
    end

    function out = local_unique_by_zip_basename(in)
        out = {};
        seen = containers.Map('KeyType','char','ValueType','logical');
        for ii = 1:numel(in)
            [~,base,ext] = fileparts(in{ii});
            key = lower([base ext]);
            if ~isKey(seen, key)
                seen(key) = true;
                out{end+1,1} = in{ii}; %#ok<AGROW>
            end
        end
    end
end

function write_how_to_run(file_path, cfg, demo_command_file)
    fid = fopen(file_path, 'w');
    assert(fid >= 0, 'Cannot write %s', file_path);
    c = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '# User Guide\n\n');
    fprintf(fid, '## Backend used in the final demo\n\n');
    fprintf(fid, '- Backend: `%s`\n', cfg.backend_name);
    fprintf(fid, '- Recovery success rate: `%d/%d = %.3f%%%%`\n', cfg.backend_success_count, cfg.backend_total_count, 100*cfg.backend_success_rate);
    fprintf(fid, '- 95%%%% recovery target pass: `%d`\n\n', logical(cfg.backend_overall_pass_95));
    fprintf(fid, '## Run MATLAB backend\n\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, "run('matlab/startupProject.m')\n");
    fprintf(fid, "run('scripts/runRecoveryMatrix.m')\n");
    fprintf(fid, "run('scripts/runRobustnessSuite.m')\n");
    fprintf(fid, "run('scripts/buildReleasePackage.m')\n");
    fprintf(fid, '```\n\n');
    fprintf(fid, '## Run Python GUI\n\n');
    fprintf(fid, '```bash\npython python/gui/runGui.py\n```\n\n');
    fprintf(fid, 'Command log: `%s`.\n\n', demo_command_file);
    fprintf(fid, '## Important limitation\n\n');
    fprintf(fid, 'The recovery quality remains at `%d/%d = %.3f%%%%`. Hard upright states are the main remaining limitation.\n', cfg.backend_success_count, cfg.backend_total_count, 100*cfg.backend_success_rate);
end

function write_final_technical_report(file_path, cfg)
    fid = fopen(file_path, 'w');
    assert(fid >= 0, 'Cannot write %s', file_path);
    c = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '# Technical Report\n\n');
    fprintf(fid, '## Project scope\n\n');
    fprintf(fid, 'This project implements a MATLAB backend for double link pendulum dynamics, target switching, external force recovery evaluation, metrics export and release packaging. Python is the only GUI layer and loads MATLAB-exported results for replay and inspection.\n\n');
    fprintf(fid, '## Backend declaration\n\n');
    fprintf(fid, '- Backend: `%s`\n', cfg.backend_name);
    fprintf(fid, '- Recovery matrix success: `%d/%d = %.3f%%%%`\n', cfg.backend_success_count, cfg.backend_total_count, 100*cfg.backend_success_rate);
    fprintf(fid, '- 95%%%% recovery target pass: `%d`\n\n', logical(cfg.backend_overall_pass_95));
    fprintf(fid, '## System architecture\n\n');
    fprintf(fid, 'MATLAB owns the plant model, controller logic, simulation workflows, recovery evaluation and file export. Python does not simulate or control the plant; it is a result viewer.\n\n');
    fprintf(fid, '## Known limitation\n\n');
    fprintf(fid, 'The current backend remains at `%d/%d = %.3f%%%%`. Hard upright and after-switching upright recovery states are the main limitation.\n\n', cfg.backend_success_count, cfg.backend_total_count, 100*cfg.backend_success_rate);
    fprintf(fid, '## Future work\n\n');
    fprintf(fid, '1. Improve remaining hard upright recovery cases with stronger constrained trajectory optimization.\n');
    fprintf(fid, '2. Add stronger MATLAB regression tests before changing controller internals.\n');
    fprintf(fid, '3. Re-run recovery, robustness and packaging after controller changes are verified.\n');
end

function write_defense_notes(file_path, cfg)
    fid = fopen(file_path, 'w');
    assert(fid >= 0, 'Cannot write %s', file_path);
    c = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '# Defense Notes\n\n');
    fprintf(fid, '## One minute project statement\n\n');
    fprintf(fid, 'The project builds a MATLAB backend and Python GUI for a cart-mounted double link pendulum. MATLAB runs dynamics, control, recovery evaluation and exports results. Python replays and visualizes those results.\n\n');
    fprintf(fid, '## What is complete\n\n');
    fprintf(fid, '- Multi target architecture and target command sequence.\n');
    fprintf(fid, '- External force event representation: link, contact ratio, direction, force and duration.\n');
    fprintf(fid, '- Recovery matrix metrics and representative trajectories for GUI replay.\n');
    fprintf(fid, '- Robustness and requirement verification.\n');
    fprintf(fid, '- Final demo guide, technical report, defense notes and release package.\n\n');
    fprintf(fid, '## Honest limitation to state during defense\n\n');
    fprintf(fid, 'The recovery backend is not hidden. It uses `%s` and currently achieves `%d/%d = %.3f%%%%`. Hard upright recovery states remain the main limitation.\n\n', cfg.backend_name, cfg.backend_success_count, cfg.backend_total_count, 100*cfg.backend_success_rate);
    fprintf(fid, '## Demo commands\n\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, "run('matlab/startupProject.m')\n");
    fprintf(fid, "run('scripts/runRecoveryMatrix.m')\n");
    fprintf(fid, "run('scripts/runRobustnessSuite.m')\n");
    fprintf(fid, "run('scripts/buildReleasePackage.m')\n");
    fprintf(fid, '```\n');
end
