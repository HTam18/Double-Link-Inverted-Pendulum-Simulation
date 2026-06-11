function library = targetEquilibriumLibrary(params)

    if nargin < 1 || isempty(params)
        params = loadParams();
    end

    local_validate_state_definition(params);

    order = {'up_up', 'up_down', 'down_up', 'down_down'};
    descriptions = struct();
    descriptions.up_up = 'link 1 upright, link 2 upright';
    descriptions.up_down = 'link 1 upright, link 2 downward';
    descriptions.down_up = 'link 1 downward, link 2 upright';
    descriptions.down_down = 'link 1 downward, link 2 downward';

    angle_table = struct();
    angle_table.up_up = [pi; pi];
    angle_table.up_down = [pi; 0.0];
    angle_table.down_up = [0.0; pi];
    angle_table.down_down = [0.0; 0.0];

    targets = struct();
    for i = 1:numel(order)
        name = order{i};
        theta = local_get_target_angles(params, name, angle_table.(name));
        target_state = [0.0; 0.0; theta(1); 0.0; theta(2); 0.0];

        item = struct();
        item.name = name;
        item.index = i;
        item.target_state = target_state;
        item.theta1_target_rad = theta(1);
        item.theta2_target_rad = theta(2);
        item.theta_targets_rad = theta(:).';
        item.u_equilibrium_N = 0.0;
        item.description = descriptions.(name);
        item.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
        item.angle_convention = 'theta = 0 downward, theta = pi upright, theta2 absolute';
        if strcmp(name, 'down_down')
            item.local_behavior_note = ['Passive gravitational equilibrium. LQR baseline still designs ', ...
                'a controlled local LQR mode so every target has a uniform controller interface.'];
        else
            item.local_behavior_note = 'Inverted or mixed equilibrium requiring active local stabilization.';
        end
        targets.(name) = item;
    end

    library = struct();
    library.workflow = 'LQR baseline';
    library.source = 'matlab/control/targetEquilibriumLibrary.m';
    library.target_order = order;
    library.targets = targets;
    library.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    library.angle_convention = 'theta = 0 rad down, theta = pi rad up, theta2 absolute';
    library.error_convention = 'cart states subtract directly; theta1/theta2 errors are wrapped to [-pi, pi] around the selected target';
end

function theta = local_get_target_angles(params, name, fallback)
    theta = fallback(:);
    if isfield(params, 'target_modes') && isfield(params.target_modes, name)
        raw = params.target_modes.(name);
        if isfield(raw, 'theta1_target_rad')
            theta(1) = double(raw.theta1_target_rad);
        end
        if isfield(raw, 'theta2_target_rad')
            theta(2) = double(raw.theta2_target_rad);
        end
    end
end

function local_validate_state_definition(params)
    expected = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    if ~isfield(params, 'state_definition') || ~isfield(params.state_definition, 'state_order')
        error('targetEquilibriumLibrary:MissingStateDefinition', ...
              'params.state_definition.state_order is required.');
    end
    actual = local_as_cellstr(params.state_definition.state_order);
    if numel(actual) ~= numel(expected) || ~all(strcmp(actual(:), expected(:)))
        error('targetEquilibriumLibrary:InvalidStateOrder', ...
              'Expected state order [%s], got [%s].', strjoin(expected, ', '), strjoin(actual, ', '));
    end
    if ~isfield(params.state_definition, 'angle_convention')
        error('targetEquilibriumLibrary:MissingAngleConvention', ...
              'params.state_definition.angle_convention is required.');
    end
    convention = lower(char(params.state_definition.angle_convention));
    if isempty(strfind(convention, '0')) || isempty(strfind(convention, 'down')) || isempty(strfind(convention, 'pi')) || isempty(strfind(convention, 'up'))
        error('targetEquilibriumLibrary:InvalidAngleConvention', ...
              'Angle convention must state theta = 0 down and theta = pi up.');
    end
end

function out = local_as_cellstr(value)
    if iscell(value)
        out = cellfun(@char, value, 'UniformOutput', false);
    elseif isstring(value)
        out = cellstr(value);
    elseif ischar(value)
        out = {value};
    else
        error('targetEquilibriumLibrary:InvalidTextArray', 'Cannot convert state order to cellstr.');
    end
end
