function kin = dipContactPointKinematics(state, link_id, contact_ratio, params)

    if nargin < 4 || isempty(params)
        params = loadParams();
    end

    state = state(:);
    if numel(state) ~= 6
        error('dipContactPointKinematics:InvalidState', 'state must have 6 elements.');
    end

    link_id = round(double(link_id));
    if ~(link_id == 1 || link_id == 2)
        error('dipContactPointKinematics:InvalidLinkId', 'link_id must be 1 or 2.');
    end

    contact_ratio = min(max(double(contact_ratio), 0.0), 1.0);

    p = params.physical_parameters;
    l1 = double(p.link1_length_m);
    l2 = double(p.link2_length_m);

    x = double(state(1));
    theta1 = double(state(3));
    theta2 = double(state(5));

    if link_id == 1
        r1 = contact_ratio * l1;
        position_xy_m = [x + r1 * sin(theta1); ...
                         -r1 * cos(theta1)];

        jacobian_xy_q = [1.0, r1 * cos(theta1), 0.0; ...
                         0.0, r1 * sin(theta1), 0.0];
    else
        r2 = contact_ratio * l2;
        position_xy_m = [x + l1 * sin(theta1) + r2 * sin(theta2); ...
                         -l1 * cos(theta1) - r2 * cos(theta2)];

        jacobian_xy_q = [1.0, l1 * cos(theta1), r2 * cos(theta2); ...
                         0.0, l1 * sin(theta1), r2 * sin(theta2)];
    end

    if any(~isfinite(position_xy_m)) || any(~isfinite(jacobian_xy_q(:)))
        error('dipContactPointKinematics:NonFiniteOutput', 'Contact kinematics returned NaN or Inf.');
    end

    kin = struct();
    kin.position_xy_m = position_xy_m;
    kin.jacobian_xy_q = jacobian_xy_q;
    kin.link_id = link_id;
    kin.contact_ratio = contact_ratio;
    kin.state_order = {'x', 'x_dot', 'theta1', 'theta1_dot', 'theta2', 'theta2_dot'};
    kin.q_order = {'x', 'theta1', 'theta2'};
    kin.source = 'matlab/dynamics/dipContactPointKinematics.m';
end
