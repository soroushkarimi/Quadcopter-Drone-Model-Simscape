for i = [10, 46]
    man_filename = ['run\fault_',num2str(1),'_maneuver_', num2str(i)];
    A = readmatrix(man_filename);
    maneuver = A(5:end,:);
    timestamp = maneuver(:,1);
    pos_x = maneuver(:,2);
    pos_y = maneuver(:,3);
    pos_z = maneuver(:,4);
    vel_x = maneuver(:,5);
    vel_y = maneuver(:,6);
    vel_z = maneuver(:,7);
    roll = maneuver(:,8);
    pitch = maneuver(:,9);
    yaw = maneuver(:,10);

    % X
    simlog_handles(1) = subplot(3, 3, 1);
    plot(timestamp, pos_x, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos x (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(2) = subplot(3, 3, 2);
    plot(timestamp, vel_x, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel x (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(3) = subplot(3, 3, 3);
    plot(timestamp, roll, 'LineWidth', 1,'DisplayName','Meas');
    title('Roll (deg)')
    linkaxes(simlog_handles, 'x')
    hold all

    % Y
    simlog_handles(4) = subplot(3, 3, 4);
    plot(timestamp, pos_y, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos y (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(5) = subplot(3, 3, 5);
    plot(timestamp, vel_y, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel y (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(6) = subplot(3, 3, 6);
    plot(timestamp, pitch, 'LineWidth', 1,'DisplayName','Meas');
    title('Pitch (deg)')
    linkaxes(simlog_handles, 'x')
    hold all

    % Z
    simlog_handles(7) = subplot(3, 3, 7);
    plot(timestamp, pos_z, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos z (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(8) = subplot(3, 3, 8);
    plot(timestamp, vel_z, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel z (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(9) = subplot(3, 3, 9);
    plot(timestamp, yaw, 'LineWidth', 1,'DisplayName','Meas');
    title('Yaw (deg)')
    linkaxes(simlog_handles, 'x')
    xlabel('Time (s)')
end
    hold all
    % legendstr{i} = sprintf('maneuver %f',i);
    % legend(legendstr,'Location','best')
    xlabel('Time (s)')
    clear simlog_handles