function [rail_force, info] = dipRailLimit(x, x_dot, params)

    if ~isfield(params, 'rail_limit')
        rail_force = 0.0;
        info = local_info(rail_force, 0.0, 0.0, false, false);
        return;
    end

    cfg = params.rail_limit;
    x_min = double(cfg.x_min_m);
    x_max = double(cfg.x_max_m);
    x_soft = double(cfg.x_soft_m);
    k_wall = double(cfg.k_wall_N_per_m);
    c_wall = double(cfg.c_wall_N_s_per_m);
    max_wall_force = double(cfg.max_wall_force_N);

    if x_min >= x_max
        error('dipRailLimit:InvalidRailRange', 'x_min_m must be smaller than x_max_m.');
    end
    if x_soft < 0 || k_wall < 0 || c_wall < 0 || max_wall_force <= 0
        error('dipRailLimit:InvalidRailParams', ...
              'x_soft, k_wall, c_wall and max_wall_force must be nonnegative/positive.');
    end

    left_penetration = max(0.0, (x_min + x_soft) - double(x));
    right_penetration = max(0.0, double(x) - (x_max - x_soft));

    left_active = left_penetration > 0.0;
    right_active = right_penetration > 0.0;

    force_left = 0.0;
    if left_active
        force_left = k_wall * left_penetration - c_wall * min(double(x_dot), 0.0);
    end

    force_right = 0.0;
    if right_active
        force_right = -k_wall * right_penetration - c_wall * max(double(x_dot), 0.0);
    end

    rail_force = force_left + force_right;
    rail_force = min(max(rail_force, -max_wall_force), max_wall_force);

    if ~isfinite(rail_force)
        error('dipRailLimit:NonFiniteForce', 'rail_force became NaN or Inf.');
    end

    info = local_info(rail_force, left_penetration, right_penetration, left_active, right_active);
end

function info = local_info(rail_force, left_penetration, right_penetration, left_active, right_active)
    info = struct();
    info.rail_force = rail_force;
    info.left_penetration = left_penetration;
    info.right_penetration = right_penetration;
    info.left_active = left_active;
    info.right_active = right_active;
end
