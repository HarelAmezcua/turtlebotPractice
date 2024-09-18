function [t_points, x_points, y_points] = define_custom_points(total_duration)
    % Define custom points for the trajectory
    t_points = [0, 15, 30, 45, 60];
    x_points = [0.1, 0.2, 0.3, 0.4, 0.5];
    y_points = [0, 0.1, 0.2, 0.3, 0.4];
    % Extend the last point to match total duration
    t_points = [t_points, total_duration];
    x_points = [x_points, x_points(end)];
    y_points = [y_points, y_points(end)];
end