function track_trajectory(velPub, velMsg, odomSub, total_duration)
    addpath("splineAndTrajectory\");
    addpath("others\");

    % Initialize simulation parameters
    [dt, k, D] = init_simulation_parameters();
    
    % Define the desired trajectory points
    [t_points, x_points, y_points] = define_custom_points(total_duration);
    
    % Compute cubic spline coefficients
    [Spx, Spy] = compute_splines(t_points, x_points, y_points);
    
    % Precompute the desired trajectory and velocities
    time_steps = 0:dt:total_duration;
    [desired_x, desired_y, desired_vel_x, desired_vel_y] = precompute_trajectory(...
        time_steps, t_points, Spx, Spy);
    
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
        
        % Display current pose
        % fprintf('Time: %.2f s, Pose: (%.2f, %.2f, %.2f)\n', t, x, y, theta);
        
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