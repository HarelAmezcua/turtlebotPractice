function [x, y, theta] = get_pose(odomSub)
% get_pose Receives an odometry message and extracts the robot's pose
%
% Inputs:
%   odomSub - Subscriber for odometry data
%
% Outputs:
%   x     - X-coordinate of the robot
%   y     - Y-coordinate of the robot
%   theta - Orientation of the robot (yaw angle)

    % Receive the latest odometry message
    odomMsg = receive(odomSub);
    
    % Extract position data
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    
    % Extract orientation data and convert quaternion to Euler angles
    quat = pose.Orientation;
    angles = quat2eul([quat.W, quat.X, quat.Y, quat.Z]);
    theta = angles(1);  % Yaw angle
end
