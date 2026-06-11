function [params, meta] = loadParams(parameter_file)

    if nargin < 1 || isempty(parameter_file)
        this_file = mfilename('fullpath');
        params_dir = fileparts(this_file);
        matlab_root = fileparts(params_dir);
        project_root = fileparts(matlab_root);
        parameter_file = fullfile(project_root, 'shared', 'parameters.json');
    else
        parameter_file = char(parameter_file);
        if ~isfile(parameter_file)
            parameter_file = fullfile(pwd, parameter_file);
        end
        project_root = fileparts(fileparts(parameter_file));
    end

    if ~isfile(parameter_file)
        error('loadParams:FileNotFound', 'Parameter file not found: %s', parameter_file);
    end

    raw_text = fileread(parameter_file);
    params = jsondecode(raw_text);

    validate_required_fields(params, parameter_file);
    params = applyRealismDefaults(params);

    state_order = as_cellstr(params.state_definition.state_order);
    expected_state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};

    if numel(state_order) ~= numel(expected_state_order) || ~all(strcmp(state_order(:), expected_state_order(:)))
        error('loadParams:InvalidStateOrder', ...
            'Unexpected state order in %s. Expected [%s]. Got [%s].', ...
            parameter_file, strjoin(expected_state_order, ', '), strjoin(state_order, ', '));
    end

    target_names = fieldnames(params.target_modes);

    meta = struct();
    meta.project_root = project_root;
    meta.parameter_file = parameter_file;
    meta.state_order = state_order(:).';
    meta.target_names = target_names(:).';
    meta.angle_convention = params.state_definition.angle_convention;
    meta.loader = 'matlab/params/loadParams.m';

    params.meta = meta;
end

function validate_required_fields(params, parameter_file)
    required_top = {'project', 'physical_parameters', 'simulation', 'control', ...
                    'state_definition', 'target_modes', 'default_initial_state'};
    for i = 1:numel(required_top)
        if ~isfield(params, required_top{i})
            error('loadParams:MissingField', ...
                'Missing required top-level field "%s" in %s.', required_top{i}, parameter_file);
        end
    end

    required_physical = {'cart_mass_kg', 'link1_mass_kg', 'link2_mass_kg', ...
                         'link1_length_m', 'link2_length_m', 'gravity_m_s2', ...
                         'cart_damping_n_s_per_m', ...
                         'link1_damping_n_m_s_per_rad', ...
                         'link2_damping_n_m_s_per_rad'};
    p = params.physical_parameters;
    for i = 1:numel(required_physical)
        if ~isfield(p, required_physical{i})
            error('loadParams:MissingPhysicalField', ...
                'Missing physical parameter "%s" in %s.', required_physical{i}, parameter_file);
        end
    end

    if ~isfield(params.simulation, 'simulation_time_s') || ~isfield(params.simulation, 'time_step_s')
        error('loadParams:MissingSimulationField', ...
            'Simulation must include simulation_time_s and time_step_s.');
    end

    if ~isfield(params.control, 'min_cart_force_N') || ~isfield(params.control, 'max_cart_force_N')
        error('loadParams:MissingControlField', ...
            'Control must include min_cart_force_N and max_cart_force_N.');
    end
end

function out = as_cellstr(value)
    if iscell(value)
        out = cellfun(@char, value, 'UniformOutput', false);
    elseif isstring(value)
        out = cellstr(value);
    elseif ischar(value)
        out = {value};
    else
        error('loadParams:InvalidTextArray', 'Value cannot be converted to a cell array of chars.');
    end
end


function params = applyRealismDefaults(params)
    if ~isfield(params, 'realism') || isempty(params.realism)
        params.realism = struct();
    end
    if ~isfield(params.realism, 'rail_limit_enabled')
        params.realism.rail_limit_enabled = false;
    end
    if ~isfield(params, 'rail_limit') || isempty(params.rail_limit)
        params.rail_limit = struct();
    end
    params.rail_limit = local_set_default(params.rail_limit, 'x_min_m', -2.0);
    params.rail_limit = local_set_default(params.rail_limit, 'x_max_m', 2.0);
    params.rail_limit = local_set_default(params.rail_limit, 'x_soft_m', 0.15);
    params.rail_limit = local_set_default(params.rail_limit, 'k_wall_N_per_m', 80.0);
    params.rail_limit = local_set_default(params.rail_limit, 'c_wall_N_s_per_m', 8.0);
    params.rail_limit = local_set_default(params.rail_limit, 'max_wall_force_N', 50.0);

    if ~isfield(params.realism, 'actuator_enabled')
        params.realism.actuator_enabled = false;
    end
    if ~isfield(params.realism, 'friction_enabled')
        params.realism.friction_enabled = false;
    end
    if ~isfield(params.realism, 'sensor_noise_enabled')
        params.realism.sensor_noise_enabled = false;
    end
    if ~isfield(params.realism, 'use_measurement_for_control')
        params.realism.use_measurement_for_control = false;
    end
    if ~isfield(params.realism, 'estimator_enabled')
        params.realism.estimator_enabled = false;
    end
    if ~isfield(params.realism, 'use_estimator_for_control')
        params.realism.use_estimator_for_control = false;
    end
    if ~isfield(params.realism, 'energy_swingup_enabled')
        params.realism.energy_swingup_enabled = false;
    end

    if ~isfield(params, 'actuator') || isempty(params.actuator)
        params.actuator = struct();
    end
    params.actuator = local_set_default(params.actuator, 'tau_motor_s', 0.03);
    params.actuator = local_set_default(params.actuator, 'rate_limit_N_per_s', 250.0);
    params.actuator = local_set_default(params.actuator, 'dead_zone_N', 0.0);
    params.actuator = local_set_default(params.actuator, 'min_force_N', double(params.control.min_cart_force_N));
    params.actuator = local_set_default(params.actuator, 'max_force_N', double(params.control.max_cart_force_N));

    if ~isfield(params, 'friction') || isempty(params.friction)
        params.friction = struct();
    end
    params.friction = local_set_default(params.friction, 'viscous_enabled', true);
    params.friction = local_set_default(params.friction, 'coulomb_enabled', true);
    params.friction = local_set_default(params.friction, 'cart_viscous_extra_N_s_per_m', 0.02);
    params.friction = local_set_default(params.friction, 'link1_viscous_extra_N_m_s_per_rad', 0.001);
    params.friction = local_set_default(params.friction, 'link2_viscous_extra_N_m_s_per_rad', 0.001);
    params.friction = local_set_default(params.friction, 'cart_coulomb_N', 0.03);
    params.friction = local_set_default(params.friction, 'link1_coulomb_N_m', 0.0015);
    params.friction = local_set_default(params.friction, 'link2_coulomb_N_m', 0.0015);
    params.friction = local_set_default(params.friction, 'coulomb_smoothing_velocity', 0.02);

    if ~isfield(params, 'sensor_noise') || isempty(params.sensor_noise)
        params.sensor_noise = struct();
    end
    params.sensor_noise = local_set_default(params.sensor_noise, 'x_std_m', 0.001);
    params.sensor_noise = local_set_default(params.sensor_noise, 'x_dot_std_m_s', 0.005);
    params.sensor_noise = local_set_default(params.sensor_noise, 'theta1_std_rad', 0.001);
    params.sensor_noise = local_set_default(params.sensor_noise, 'theta1_dot_std_rad_s', 0.005);
    params.sensor_noise = local_set_default(params.sensor_noise, 'theta2_std_rad', 0.001);
    params.sensor_noise = local_set_default(params.sensor_noise, 'theta2_dot_std_rad_s', 0.005);
    params.sensor_noise = local_set_default(params.sensor_noise, 'seed', 25);

    if ~isfield(params, 'estimator') || isempty(params.estimator)
        params.estimator = struct();
    end
    params.estimator = local_set_default(params.estimator, 'velocity_filter_alpha', 0.95);
    params.estimator = local_set_default(params.estimator, 'blend_measured_velocity', false);
    params.estimator = local_set_default(params.estimator, 'velocity_measurement_blend', 0.0);
    params.estimator = local_set_default(params.estimator, 'max_position_error_for_pass', 0.02);
    params.estimator = local_set_default(params.estimator, 'max_velocity_error_rms_for_pass', 0.35);

    if ~isfield(params, 'energy_swingup') || isempty(params.energy_swingup)
        params.energy_swingup = struct();
    end
    params.energy_swingup = local_set_default(params.energy_swingup, 'target_mode', 'up_up');
    params.energy_swingup = local_set_default(params.energy_swingup, 'max_force_N', 12.0);
    params.energy_swingup = local_set_default(params.energy_swingup, 'energy_gain', 1.8);
    params.energy_swingup = local_set_default(params.energy_swingup, 'velocity_gain', 1.2);
    params.energy_swingup = local_set_default(params.energy_swingup, 'cart_center_gain', 1.0);
    params.energy_swingup = local_set_default(params.energy_swingup, 'cart_velocity_gain', 0.6);
    params.energy_swingup = local_set_default(params.energy_swingup, 'angle_align_gain', 3.0);
    params.energy_swingup = local_set_default(params.energy_swingup, 'switch_angle_threshold_rad', 0.18);
    params.energy_swingup = local_set_default(params.energy_swingup, 'switch_velocity_threshold_rad_s', 1.2);
    params.energy_swingup = local_set_default(params.energy_swingup, 'switch_cart_threshold_m', 1.2);
    params.energy_swingup = local_set_default(params.energy_swingup, 'switch_hold_time_s', 0.05);
    params.energy_swingup = local_set_default(params.energy_swingup, 'lqr_handoff_enabled', true);
    params.energy_swingup = local_set_default(params.energy_swingup, 'rail_margin_m', 0.25);
end

function s = local_set_default(s, field_name, default_value)
    if ~isfield(s, field_name) || isempty(s.(field_name))
        s.(field_name) = default_value;
    end
end
