function [dt, k, D] = init_simulation_parameters()
    % Control gains
    kx = 1;
    ky = 1;
    k = [kx 0; 0 ky];

    % Time step
    dt = 0.1;

    % Distance from robot's center to point of interest
    D = 0.1;
end