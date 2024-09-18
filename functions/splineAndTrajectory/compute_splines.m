function [Spx, Spy] = compute_splines(t_points, x_points, y_points)
    % Compute cubic spline coefficients for x and y
    Spx = obtener_splines(t_points, x_points);
    Spy = obtener_splines(t_points, y_points);
end