function route = transitionRoutePlanner(current_target, requested_target, params, user_cfg)

    if nargin < 3 || isempty(params)
        params = loadParams();
    end
    if nargin < 4 || isempty(user_cfg)
        user_cfg = struct();
    end

    current_target = char(current_target);
    requested_target = char(requested_target);
    cfg = RecoveryUtils.defaultCfg(user_cfg, params);
    graph = transitionGraphLibrary(params);

    route = struct();
    route.current_target = current_target;
    route.requested_target = requested_target;
    route.pass = false;
    route.safe_reject = false;
    route.reject_reason = '';
    route.edge_names = {};
    route.node_path = {current_target};
    route.available_edges = {};
    route.missing_edges = {};
    route.route_source = 'matlab/control/transitionRoutePlanner.m';

    if ~isfield(graph.nodes, current_target)
        route.safe_reject = true;
        route.reject_reason = sprintf('UNKNOWN_CURRENT_TARGET_%s', current_target);
        return;
    end
    if ~isfield(graph.nodes, requested_target)
        route.safe_reject = true;
        route.reject_reason = sprintf('UNKNOWN_REQUESTED_TARGET_%s', requested_target);
        return;
    end
    if strcmp(current_target, requested_target)
        route.pass = true;
        route.edge_names = {};
        route.node_path = {current_target};
        route.reject_reason = 'ALREADY_AT_REQUESTED_TARGET';
        return;
    end

    node_order = graph.node_order;
    n = numel(node_order);
    adj = cell(n, 1);
    available_edges = {};
    missing_edges = {};
    for i = 1:numel(graph.edge_order)
        edge_name = graph.edge_order{i};
        edge = graph.edges.(edge_name);
        traj_file = fullfile(cfg.trajectory_dir, sprintf('transition_%s_tracking_ready.mat', edge_name));
        gain_file = fullfile(cfg.tvlqr_gain_dir, sprintf('transition_%s_K.mat', edge_name));
        edge_available = isfile(traj_file);
        if cfg.require_tvlqr_gain_file
            edge_available = edge_available && isfile(gain_file);
        end
        if edge_available
            sidx = find(strcmp(node_order, edge.source), 1);
            adj{sidx}{end + 1} = edge_name; %#ok<AGROW>
            available_edges{end + 1} = edge_name; %#ok<AGROW>
        else
            missing_edges{end + 1} = edge_name; %#ok<AGROW>
        end
    end

    route.available_edges = available_edges;
    route.missing_edges = missing_edges;

    direct_edge = sprintf('%s_to_%s', current_target, requested_target);
    if ismember(direct_edge, available_edges)
        route.pass = true;
        route.edge_names = {direct_edge};
        route.node_path = {current_target, requested_target};
        route.reject_reason = '';
        return;
    end

    if ~cfg.allow_multihop
        route.safe_reject = true;
        route.reject_reason = sprintf('NO_DIRECT_TRACKING_READY_TRAJECTORY_%s', direct_edge);
        return;
    end

    start_idx = find(strcmp(node_order, current_target), 1);
    goal_idx = find(strcmp(node_order, requested_target), 1);
    [found, edge_path, node_path] = local_bfs_available_edges(start_idx, goal_idx, node_order, adj, graph);
    if found
        route.pass = true;
        route.edge_names = edge_path;
        route.node_path = node_path;
        route.reject_reason = '';
    else
        route.safe_reject = true;
        route.reject_reason = sprintf('NO_AVAILABLE_ROUTE_%s_TO_%s', current_target, requested_target);
    end
end

function cfg = defaultCfg(user_cfg, params)
    cfg = struct();
    project_root = RecoveryUtils.projectRoot(params);
    cfg.trajectory_dir = fullfile(project_root, 'shared', 'trajectories');
    cfg.tvlqr_gain_dir = fullfile(project_root, 'shared', 'tvlqr_gains');
    cfg.allow_multihop = true;
    cfg.require_tvlqr_gain_file = false;
    fields = fieldnames(user_cfg);
    for i = 1:numel(fields)
        cfg.(fields{i}) = user_cfg.(fields{i});
    end
end

function project_root = projectRoot(params)
    if isfield(params, 'meta') && isfield(params.meta, 'project_root') && ~isempty(params.meta.project_root)
        project_root = char(params.meta.project_root);
    else
        this_file = mfilename('fullpath');
        control_dir = fileparts(this_file);
        matlab_root = fileparts(control_dir);
        project_root = fileparts(matlab_root);
    end
end

function [found, edge_path, node_path] = local_bfs_available_edges(start_idx, goal_idx, node_order, adj, graph)
    n = numel(node_order);
    visited = false(n, 1);
    prev_node = zeros(n, 1);
    prev_edge = cell(n, 1);
    queue = start_idx;
    visited(start_idx) = true;
    found = false;

    while ~isempty(queue)
        u = queue(1);
        queue(1) = [];
        if u == goal_idx
            found = true;
            break;
        end
        for k = 1:numel(adj{u})
            edge_name = adj{u}{k};
            edge = graph.edges.(edge_name);
            v = find(strcmp(node_order, edge.target), 1);
            if ~visited(v)
                visited(v) = true;
                prev_node(v) = u;
                prev_edge{v} = edge_name;
                queue(end + 1) = v; %#ok<AGROW>
            end
        end
    end

    edge_path = {};
    node_path = {};
    if ~found
        return;
    end

    node_indices = goal_idx;
    cur = goal_idx;
    while cur ~= start_idx
        edge_path = [{prev_edge{cur}}, edge_path]; %#ok<AGROW>
        cur = prev_node(cur);
        node_indices = [cur, node_indices]; %#ok<AGROW>
    end
    for i = 1:numel(node_indices)
        node_path{end + 1} = node_order{node_indices(i)}; %#ok<AGROW>
    end
end
