function out = exportGuiCaseMetricsCore(user_cfg)

    if nargin < 1 || isempty(user_cfg)
        user_cfg = struct();
    end

    project_root = RecoveryUtils.projectRoot();
    result_dir = fullfile(project_root, 'results', 'recovery');
    if isfield(user_cfg, 'result_dir') && ~isempty(user_cfg.result_dir)
        result_dir = char(user_cfg.result_dir);
    end

    source_csv = fullfile(result_dir, 'recoveryAllMetrics.csv');
    if ~isfile(source_csv)
        error('export_recovery_gui_case_metrics:MissingSourceCSV', ...
              'Missing Recovery metrics CSV: %s', source_csv);
    end

    T = readtable(source_csv, 'TextType', 'string');
    T = local_normalize_gui_columns(T);

    gui_columns = {'case_index','case_type','target','event_name','link_id','contact_ratio','direction','force_N', ...
                   'success','recovery_time_s','saturation_fraction','fail_reason','final_angle_error_rad', ...
                   'final_velocity_norm','max_abs_x_m','max_abs_u_cmd_N','planner_selected_variant', ...
                   'trajectory_file','animation_available','animation_status'};
    gui_columns = gui_columns(ismember(gui_columns, T.Properties.VariableNames));
    gui_table = T(:, gui_columns);

    out_csv = fullfile(result_dir, 'recoveryGuiCases.csv');
    writetable(gui_table, out_csv);

    db = struct();
    db.created_at = string(datetime('now'));
    db.source_csv = string(source_csv);
    db.gui_csv = string(out_csv);
    db.case_count = height(gui_table);
    db.metrics_table = gui_table;
    db.case_types = unique(string(gui_table.case_type));
    db.targets = unique(string(gui_table.target));
    db.link_ids = unique(double(gui_table.link_id));
    db.contact_ratios = unique(double(gui_table.contact_ratio));
    db.directions = unique(string(gui_table.direction));
    db.force_levels_N = unique(double(gui_table.force_N));
    db.note = "GUI manifest only. Full animation requires trajectory_file exported by runRecoveryMatrixCore.";

    out_mat = fullfile(result_dir, 'recoveryCaseDatabase.mat');
    save(out_mat, 'db', '-v7.3');

    out = struct();
    out.source_csv = source_csv;
    out.gui_csv = out_csv;
    out.database_mat = out_mat;
    out.case_count = height(gui_table);

    fprintf('Recovery GUI case CSV: %s\n', out_csv);
    fprintf('Recovery GUI case database: %s\n', out_mat);
    fprintf('Case count: %d\n', height(gui_table));
end

function T = local_normalize_gui_columns(T)
    if ~ismember('trajectory_file', T.Properties.VariableNames)
        T.trajectory_file = strings(height(T), 1);
    else
        T.trajectory_file = string(T.trajectory_file);
    end

    if ~ismember('animation_available', T.Properties.VariableNames)
        T.animation_available = double(strlength(T.trajectory_file) > 0);
    else
        T.animation_available = double(T.animation_available);
    end

    if ~ismember('animation_status', T.Properties.VariableNames)
        status = strings(height(T), 1);
        for i = 1:height(T)
            if double(T.animation_available(i)) ~= 0 && strlength(string(T.trajectory_file(i))) > 0
                status(i) = "trajectory_exported";
            else
                status(i) = "metrics_only";
            end
        end
        T.animation_status = status;
    end

    string_cols = {'case_type','target','event_name','direction','fail_reason','planner_selected_variant','trajectory_file','animation_status'};
    for i = 1:numel(string_cols)
        name = string_cols{i};
        if ismember(name, T.Properties.VariableNames)
            T.(name) = string(T.(name));
        end
    end
end

function project_root = projectRoot()
    here = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(here));
end
