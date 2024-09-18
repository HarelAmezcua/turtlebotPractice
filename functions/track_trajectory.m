function track_trajectory(velPub, velMsg, odomSub, total_duration)
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
        x_log = [x_log; x];
        y_log = [y_log; y];
        theta_log = [theta_log; theta];
        v_log = [v_log; v];
        w_log = [w_log; w];
        x_des_log = [x_des_log; desired_x(idx)];
        y_des_log = [y_des_log; desired_y(idx)];
        
        % Display current pose (optional)
        fprintf('Time: %.2f s, Pose: (%.2f, %.2f, %.2f)\n', t, x, y, theta);
        
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


%% Functions

function [dt, k, D] = init_simulation_parameters()
    % Control gains
    kx = 1;
    ky = 1;
    k = [kx 0; 0 ky];

    % Time step
    dt = 0.01;

    % Distance from robot's center to point of interest
    D = 0.1;
end


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

function [Spx, Spy] = compute_splines(t_points, x_points, y_points)
    % Compute cubic spline coefficients for x and y
    Spx = obtener_splines(t_points, x_points);
    Spy = obtener_splines(t_points, y_points);
end

function S = obtener_splines(t, q)
    % Natural cubic spline computation
    n = numel(t);
    h = diff(t);
    diag_princ = 2 * (h(1:end-1) + h(2:end));
    diag_sup = h(2:end-1);
    diag_inf = h(2:end-1);
    M = diag(diag_princ) + diag(diag_sup, 1) + diag(diag_inf, -1);
    delta = diff(q) ./ h;
    b = 6 * diff(delta);
    g_interior = M \ b';
    g = [0; g_interior; 0];
    a = (g(2:end) - g(1:end-1)) ./ (6 * h');
    b_coef = g(1:end-1) / 2;
    c = delta' - h' .* (2 * g(1:end-1) + g(2:end)) / 6;
    d = q(1:end-1)';
    S = [a b_coef c d];
end


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

function [qk, qpk] = evaluar_splines(S, t_array, time)
    n = numel(t_array);
    if time > t_array(end)
        k = n - 1;
    else
        k = find(t_array(1:end-1) <= time & time <= t_array(2:end), 1);
    end
    if isempty(k)
        k = 1;
    end
    h = time - t_array(k);
    qk = S(k, :) * [h^3; h^2; h; 1];
    qpk = S(k, :) * [3*h^2; 2*h; 1; 0];
end
