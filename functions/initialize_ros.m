function [velPub, velMsg, odomSub, resetPub, resetMsg] = initialize_ros()
% initialize_ros Initializes ROS and sets up publishers and subscribers
%
% Outputs:
%   velPub   - Publisher for velocity commands
%   velMsg   - Message template for velocity commands
%   odomSub  - Subscriber for odometry data
%   resetPub - Publisher for reset commands
%   resetMsg - Message template for reset commands

    % Shutdown any existing ROS connections
    rosshutdown;
    % Initialize ROS master node at specified IP
    rosinit("http://192.168.50.113:11311");
    
    % Set up velocity publisher and message
    velPub = rospublisher('/bot13/cmd_vel', 'geometry_msgs/Twist');
    velMsg = rosmessage(velPub);
    
    % Set up odometry subscriber
    odomSub = rossubscriber('/bot13/odom', 'nav_msgs/Odometry');
    
    % Set up reset publisher and message
    resetPub = rospublisher('/bot13/reset', 'std_msgs/Empty');
    resetMsg = rosmessage(resetPub);
end
