function stop_robot(velPub, velMsg, resetPub, resetMsg)
% stop_robot Stops the robot and sends a reset command
%
% Inputs:
%   velPub   - Publisher for velocity commands
%   velMsg   - Message template for velocity commands
%   resetPub - Publisher for reset commands
%   resetMsg - Message template for reset commands

    % Stop the robot by setting velocities to zero
    velMsg.Linear.X = 0.0;
    velMsg.Angular.Z = 0.0;
    send(velPub, velMsg);
    
    % Send reset command to the robot
    send(resetPub, resetMsg);
    pause(0.1);  % Brief pause to allow the reset to take effect
end
