function [t_points, x_points, y_points] = define_custom_points(total_duration)
    % Define custom points for the trajectory
    t_points = [0, 10, 20, 30, 40];
    x_points = [0, 0.5, 0.5, -0.5, -0.5];
    y_points = [0, 0.5, -0.5, -0.5, 0.5];
    % Extend the last point to match total duration
    t_points = [t_points, total_duration];
    x_points = [x_points, x_points(end)];
    y_points = [y_points, y_points(end)];
end