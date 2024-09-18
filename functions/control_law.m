function vw = control_law(pos, pos_des, vel_des, p, D, k)
    % pos: Current point of interest position
    % pos_des: Desired position
    % vel_des: Desired velocity
    % p: Robot's state [x; y; theta]
    % D: Distance from robot's center
    % k: Control gain matrix

    % Compute position error
    e = pos_des - pos;

    % Desired velocity of point of interest
    v_pi = vel_des + k * e;

    % Transformation matrix
    J = [cos(p(3)), -D * sin(p(3));
         sin(p(3)),  D * cos(p(3))];

    % Compute robot's velocities
    vw = J \ v_pi;
end
