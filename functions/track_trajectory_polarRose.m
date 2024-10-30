function track_trajectory(velPub, velMsg, odomSub, total_duration)
    % Initialize simulation parameters
    [dt, k, D] = init_simulation_parameters();
    
    % Define parameters for the polar rose trajectory
    a = 1;  % Amplitude (size of the rose)
    k_rose = 2;  % Number of petals control (even k => 2k petals)
    
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
    
    % Initialize logs for data recording
    time_log = [];
    x_log = [];
    y_log = [];
    theta_log = [];
    v_log = [];
    w_log = [];
    x_des_log = [];
    y_des_log = [];
        
    
    % Start the control loop
    tic;
    idx = 1;
    while toc <= total_duration
        t = toc;
        
        % Read current pose from odometry
        [x, y, theta] = get_pose(odomSub);
        
        % Get desired position and velocity at current time
        if idx > length(time_steps)
            idx = length(time_steps);
        end
        pos_des = [desired_x(idx); desired_y(idx)];
        vel_des = [desired_vel_x(idx); desired_vel_y(idx)];
        
        % Current position of the point of interest
        pos = [x + D * cos(theta); y + D * sin(theta)];
        
        % Compute control action
        vw = control_law(pos, pos_des, vel_des, [x; y; theta], D, k);
        
        % Clamp velocities to TurtleBot's allowed ranges
        v = max(min(vw(1), 0.2), -0.2);
        w = max(min(vw(2), 0.9), -0.9);
        
        % Update and send velocity command
        velMsg.Linear.X = v;
        velMsg.Angular.Z = w;
        send(velPub, velMsg);
        
        % Record data
        time_log = [time_log; t];
        x_log = [x_log; pos(1)];
        y_log = [y_log; pos(2)];
        theta_log = [theta_log; theta];
        v_log = [v_log; v];
        w_log = [w_log; w];
        x_des_log = [x_des_log; desired_x(idx)];
        y_des_log = [y_des_log; desired_y(idx)];
        
        % Increment index
        idx = idx + 1;
        
        % Wait for next time step
        pause(dt);
    end
    
    % Stop the robot
    velMsg.Linear.X = 0;
    velMsg.Angular.Z = 0;
    send(velPub, velMsg);
    
    % Plot results
    plot_results(time_log, x_log, y_log, x_des_log, y_des_log, v_log, w_log);
end
