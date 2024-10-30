clear;
clc;
close all;

% Initialize simulation parameters
[dt, k, D] = init_simulation_parameters();

% Define the desired trajectory points
total_duration = 50;  % Set a value for total_duration
[t_points, x_points, y_points] = define_custom_points(total_duration);

% Compute cubic spline coefficients
[Spx, Spy] = compute_splines(t_points, x_points, y_points);

% Precompute the desired trajectory and velocities
time_steps = 0:dt:total_duration;
[desired_x, desired_y, desired_vel_x, desired_vel_y] = precompute_trajectory(...
    time_steps, t_points, Spx, Spy);

% Plotting the results with enhancements
figure;
hold on;

% Plot the trajectory in space (desired_x vs desired_y)
plot(desired_x, desired_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Trajectory');
plot(desired_x(1), desired_y(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');  % Starting point
plot(desired_x(end), desired_y(end), 'rx', 'MarkerSize', 10, 'DisplayName', 'End'); % End point

% Plot velocities over time (desired_vel_x vs time_steps)
% plot(time_steps, desired_vel_x, 'r--', 'LineWidth', 2, 'DisplayName', 'Velocity X');

% Add labels, title, grid, and legend
xlabel('X Position / Velocity X');
ylabel('Y Position / Velocity Y');
title('Trajectory and Velocity over Time');
legend('show');
grid on;

% Highlight the custom points used for the spline
plot(x_points, y_points, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Control Points');

hold off;


 save("Trajectory.mat","desired_x","desired_y","desired_vel_x","desired_vel_y");