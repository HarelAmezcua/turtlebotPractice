clear;
close all;
clc;

addpath('functions\');

% Initialize ROS and set up publishers and subscribers
%Hola Mundo
[velPub, velMsg, odomSub, resetPub, resetMsg] = initialize_ros();

% Flush odometry messages to ensure up-to-date readings
for i = 1:10
    receive(odomSub);
end

% Get and display the initial pose of the robot
[x, y, theta] = get_pose(odomSub);
disp(['Initial Pose: (' num2str(x) ', ' num2str(y) ', ' num2str(theta) ')']);

% Move the robot forward for 5 seconds
track_trajectory(velPub, velMsg, odomSub, 45);

% Stop and reset the robot
stop_robot(velPub, velMsg, resetPub, resetMsg);