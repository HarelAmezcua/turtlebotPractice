clear;
clc;
close all;


% Define parameters for the polar rose trajectory
a = 0.25;  % Amplitude (size of the rose)
k_rose = 5;  % Number of petals control (even k => 2k petals)
dt = 0.01;
total_duration = 100;

% Precompute the desired trajectory and velocities
time_steps = 0:dt:total_duration;
desired_x = zeros(1, length(time_steps));
desired_y = zeros(1, length(time_steps));
desired_vel_x = zeros(1, length(time_steps));
desired_vel_y = zeros(1, length(time_steps));

for i = 1:length(time_steps)
    t = time_steps(i);
    theta_t = 2 * pi * t / total_duration; % Angle that evolves over time
    
    % Polar rose trajectory
    r_t = a * cos(k_rose * theta_t);
    
    % Desired position
    desired_x(i) = r_t * cos(theta_t);
    desired_y(i) = r_t * sin(theta_t);
    
    % First derivatives (velocities)
    dr_dt = -a * k_rose * sin(k_rose * theta_t); % Derivative of r(t)
    dtheta_dt = 2 * pi / total_duration; % Constant angular velocity
    
    % Velocities in Cartesian coordinates
    desired_vel_x(i) = dr_dt * cos(theta_t) - r_t * sin(theta_t) * dtheta_dt;
    desired_vel_y(i) = dr_dt * sin(theta_t) + r_t * cos(theta_t) * dtheta_dt;
end

% Plot the desired trajectory before starting the control loop
figure;
plot(desired_x, desired_y);
title('Desired Polar Rose Trajectory');
xlabel('X position');
ylabel('Y position');
axis equal;
grid on;
