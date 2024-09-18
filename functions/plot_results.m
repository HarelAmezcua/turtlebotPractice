function plot_results(time_log, x_log, y_log, x_des_log, y_des_log, v_log, w_log)
    % Plot X and Y positions over time
    figure;
    subplot(2,1,1);
    plot(time_log, x_log, 'b-', time_log, x_des_log, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('X Position [m]');
    legend('Actual X', 'Desired X');
    grid on;

    subplot(2,1,2);
    plot(time_log, y_log, 'b-', time_log, y_des_log, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Y Position [m]');
    legend('Actual Y', 'Desired Y');
    grid on;

    % Plot control inputs over time
    figure;
    subplot(2,1,1);
    plot(time_log, v_log, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Linear Velocity v [m/s]');
    grid on;

    subplot(2,1,2);
    plot(time_log, w_log, 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Angular Velocity \omega [rad/s]');
    grid on;

    % Plot the trajectory
    figure;
    plot(x_log, y_log, 'b-', x_des_log, y_des_log, 'r--', 'LineWidth', 1.5);
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    legend('Actual Trajectory', 'Desired Trajectory');
    axis equal;
    grid on;
end
