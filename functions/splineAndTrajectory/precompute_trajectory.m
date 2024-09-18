function [desired_x, desired_y, desired_vel_x, desired_vel_y] = precompute_trajectory(...
    time_steps, t_points, Spx, Spy)

    num_steps = length(time_steps);
    desired_x = zeros(1, num_steps);
    desired_y = zeros(1, num_steps);
    desired_vel_x = zeros(1, num_steps);
    desired_vel_y = zeros(1, num_steps);

    for idx = 1:num_steps
        t = time_steps(idx);
        [xk, xpk] = evaluar_splines(Spx, t_points, t);
        [yk, ypk] = evaluar_splines(Spy, t_points, t);
        desired_x(idx) = xk;
        desired_y(idx) = yk;
        desired_vel_x(idx) = xpk;
        desired_vel_y(idx) = ypk;
    end
end