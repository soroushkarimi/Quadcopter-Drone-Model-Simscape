%% Use Parallel Computing and Fast Restart to sweep parameter value
% Copyright 2021-2023 The MathWorks, Inc.

% Move to folder where script is saved
cd(fileparts(which(mfilename)));

% Open model and save under another name for test
orig_mdl = 'quadcopter_package_delivery';
open_system(orig_mdl);
quadcopter_package_parameters
[waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
    quadcopter_package_select_trajectory(7,2,2,5);
motor_gain_mult = 1;
mdl = [orig_mdl '_pct_temp'];
save_system(orig_mdl,mdl);

%% Generate parameter sets
pkgSize = evalin('base','pkgSize');
pkgVol  = pkgSize(1)*pkgSize(2)*pkgSize(3);
pkgDensity_array = linspace(0.5/pkgVol,1.5/pkgVol,4);
gain_array = linspace(1,0.8,3);

n = 5;
[x, y, z] = sphere(n);
sp_x = x;
sp_y = y;
sp_z = z+5;
target_waypoint = zeros(3,n^2);
for i=1:n+1
    for j=1:n+1
        target_waypoint(1,(n)*i + j) = sp_x(i,j);
        target_waypoint(2,(n)*i + j) = sp_y(i,j);
        target_waypoint(3,(n)*i + j) = sp_z(i,j);
    end
end


clear simInput
% simInput(1:length(z_array)) = Simulink.SimulationInput(mdl);
simInput(1:length(gain_array)*length(target_waypoint)) = Simulink.SimulationInput(mdl);
runNum = 0;
for i=1:length(gain_array)
    for j=1:length(target_waypoint)
        runNum = runNum+1;
        [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
        quadcopter_package_select_trajectory(7,target_waypoint(1,j),target_waypoint(2,j),target_waypoint(3,j));
        simInput(runNum) = simInput(runNum).setVariable('waypoints',waypoints);
        simInput(runNum) = simInput(runNum).setVariable('timespot_spl',timespot_spl);
        simInput(runNum) = simInput(runNum).setVariable('spline_data',spline_data);
        simInput(runNum) = simInput(runNum).setVariable('spline_yaw',spline_yaw);
        simInput(runNum) = simInput(runNum).setVariable('wayp_path_vis',wayp_path_vis);
        simInput(runNum) = simInput(runNum).setVariable('motor_gain_mult',gain_array(i));
    end
end

pkgDensity = pkgDensity_array(1);
wind_speed = 0;
save_system(mdl);

%% Run one simulation to see time used
timerVal = tic;
sim(mdl)
Elapsed_Sim_Time_single = toc(timerVal);
disp(['Elapsed Simulation Time Single Run: ' num2str(Elapsed_Sim_Time_single)]);

%% Adjust settings and save
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
set_param(mdl,'SimscapeLogType','none');
set_param(mdl,'SimscapeLogToSDI','off');
save_system(mdl)

%% Run parameter sweep in parallel
warning off physmod:common:logging2:mli:mcos:kernel:SdiStreamSaveError
timerVal = tic;
simOut = parsim(simInput,'ShowSimulationManager','on',...
    'ShowProgress','on','UseFastRestart','off',...
    'TransferBaseWorkspaceVariables','on');
Elapsed_Time_Time_parallel  = toc(timerVal);
warning on physmod:common:logging2:mli:mcos:kernel:SdiStreamSaveError

%% Calculate elapsed time less setup of parallel
Elapsed_Time_Sweep = ...
    (datenum(simOut(end).SimulationMetadata.TimingInfo.WallClockTimestampStop) - ...
    datenum(simOut(1).SimulationMetadata.TimingInfo.WallClockTimestampStart)) * 86400;
disp(['Elapsed Sweep Time Total:       ' sprintf('%5.2f',Elapsed_Time_Sweep)]);
disp(['Elapsed Sweep Time/(Num Tests): ' sprintf('%5.2f',Elapsed_Time_Sweep/length(simOut))]);
disp(' ');

%% Plot results
plot_sim_res(simInput,simOut,waypoints,planex,planey,'Parallel Test',Elapsed_Time_Time_parallel)
% plot_sim_res_batt(simInput,simOut)
% plot_sim_res_att(simInput,simOut)
save_sim_data(simInput, simOut)

%% Close parallel pool
delete(gcp);

%% Cleanup directory
bdclose(mdl);
delete([mdl '.slx']);

%%  Function to plot paths during tests
function plot_sim_res(simInput,simOut,waypoints,planex,planey,annotation_str,elapsed_time)

% Plot Results
fig_handle_name =   'h4_quadcopter_package_delivery_pct_mass';

handle_var = evalin('base',['who(''' fig_handle_name ''')']);
if(isempty(handle_var))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
elseif ~isgraphics(evalin('base',handle_var{:}))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
end
figure(evalin('base',fig_handle_name))
clf(evalin('base',fig_handle_name))

% Plot trajectories
legendstr = cell(1,length(simOut));
% pkgSize = evalin('base','pkgSize');
for i=1:length(simOut)
    data_px = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.px;
    data_py = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.py;
    data_pz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.pz;
    missionStatus = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Load.status.Data(end);
    LineStyle = '-';
    LineWidth = 2;
    if(missionStatus == 0)
        % If joint is still engaged at finish, mission failed
        LineStyle = ':';
        LineWidth = 0.5;
    end
    plot3(data_px.Data,data_py.Data,data_pz.Data,'LineWidth',LineWidth,'LineStyle',LineStyle)
    % legendstr{i} = sprintf('%3.2d kg',simInput(i).Variables(1).Value(end)*pkgSize(1)*pkgSize(2)*pkgSize(3));
    legendstr{i} = sprintf('maneuver %d',i);
    hold all
end

% Plot waypoints
wayp_unique = unique(waypoints','rows');
plot3(wayp_unique(:,1),wayp_unique(:,2),wayp_unique(:,3),'o','MarkerSize',6,'MarkerFaceColor','cyan','MarkerEdgeColor','none','DisplayName','Waypoints')
legendstr{end+1} = 'Waypoints';
[planeMeshx,planeMeshy] = meshgrid([-0.5 0.5]*planex,[-0.5 0.5]*planey);
surf(planeMeshx, planeMeshy, zeros(size(planeMeshx)),'FaceColor',[0.8 0.9 0.8],'DisplayName','')
legendstr{end+1} = '';
hold off

% Setup axes
ah = gca;
% Uncomment to disable clipping 
% Keep clipping on if missions go far from target
%ah.Clipping = 'off';
grid on
title('Parameter Sweep Results (Package Mass)');
box on
legend(legendstr,'Location','West');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal
set(gca,'XLim',[-0.5 0.5]*planex,'YLim',[-0.5 0.5]*planey)
view(-5,16)

text(0.5,0.15,sprintf('%s\n%s',annotation_str,['Elapsed Time: ' num2str(elapsed_time)]),'Color',[1 1 1]*0.6,'Units','Normalized');
end

%%  Function to plot electrical quantities during tests
function plot_sim_res_batt(simInput,simOut)

% Create/Reuse figure and define handle in workspace
fig_handle_name =   'h5_quadcopter_package_delivery_pct_mass';

handle_var = evalin('base',['who(''' fig_handle_name ''')']);
if(isempty(handle_var))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
elseif ~isgraphics(evalin('base',handle_var{:}))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
end
figure(evalin('base',fig_handle_name))
clf(evalin('base',fig_handle_name))

% Plot state of charge
legendstr = cell(1,length(simOut));
pkgSize = evalin('base','pkgSize');
for i=1:length(simOut)
    data_soc = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Motor.Battery.SOC;
    missionStatus = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Load.status.Data(end);
    LineStyle = '-';
    LineWidth = 2;
    if(missionStatus == 0)
        % If joint is still engaged at finish, mission failed
        LineStyle = ':';
        LineWidth = 0.5;
    end
    plot(data_soc.Time,data_soc.Data,'LineWidth',LineWidth,'LineStyle',LineStyle)
    legendstr{i} = sprintf('%3.2f kg',simInput(i).Variables(1).Value(end)*pkgSize(1)*pkgSize(2)*pkgSize(3));
    hold all
end

grid on
title('Effect of Package Mass on Battery SOC');
box on
legend(legendstr,'Location','Best');
xlabel('Time (sec)');
ylabel('SOC (A*hr)');

end

%%  Function to plot Attitude of aircraft during tests
function plot_sim_res_att(simInput,simOut)

% Create/Reuse figure and define handle in workspace
fig_handle_name =   'h6_quadcopter_package_delivery_pct_mass';

handle_var = evalin('base',['who(''' fig_handle_name ''')']);
if(isempty(handle_var))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
elseif ~isgraphics(evalin('base',handle_var{:}))
    evalin('base',[fig_handle_name ' = figure(''Name'', ''' fig_handle_name ''');']);
end
figure(evalin('base',fig_handle_name))
clf(evalin('base',fig_handle_name))

% Plot state of charge
for i=1:length(simOut)
    simlog_px = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.px.Data;
    simlog_py = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.py.Data;
    simlog_pz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.pz.Data;
    simlog_vx = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vx.Data;
    simlog_vy = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vy.Data;
    simlog_vz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vz.Data;
    simlog_qx = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.roll.Data;
    simlog_qy = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.pitch.Data;
    simlog_qz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.yaw.Data;
    simlog_t  = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.px.Time;
    
    result_matrix = [simlog_t,simlog_px, simlog_py, ...
        simlog_pz, simlog_vx, simlog_vy, ...
        simlog_vz,squeeze(simlog_qx)*180/pi,squeeze(simlog_qy)*180/pi,squeeze(simlog_qz)*180/pi];
    result_matrix_filename = ['run\fault_',num2str(1),'_maneuver_', num2str(i)];
    writematrix(result_matrix, result_matrix_filename);
    % ref_pxyz = simOut(i).logsout_quadcopter_package_delivery.get('Ref').Values.pos.Data(:,:)';
    % ref_vxyz = simOut(i).logsout_quadcopter_package_delivery.get('Ref').Values.vel.Data(:,:)';
    % ref_roll = simOut(i).logsout_quadcopter_package_delivery.get('Ref').Values.roll.Data(:,:)';
    % ref_pitch = simOut(i).logsout_quadcopter_package_delivery.get('Ref').Values.pitch.Data(:,:)';
    % ref_yaw = simOut(i).logsout_quadcopter_package_delivery.get('Ref').Values.yaw.Data(:,:)';
    % 
    % missionStatus = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Load.status.Data(end);
    % LineStyle = '-';
    % LineWidth = 2;

    % if(missionStatus == 0)
    %     % If joint is still engaged at finish, mission failed
    %     LineStyle = ':';
    %     LineWidth = 0.5;
    % end
    
    % if(size(ref_pxyz,2)>3)
    % ref_pxyz = ref_pxyz';
    % ref_vxyz = ref_vxyz';
    % end

    % X
    simlog_handles(1) = subplot(3, 3, 1);
    plot(simlog_t, simlog_px, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos x (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(2) = subplot(3, 3, 2);
    plot(simlog_t, simlog_vx, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel x (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(3) = subplot(3, 3, 3);
    plot(simlog_t, squeeze(simlog_qx)*180/pi, 'LineWidth', 1,'DisplayName','Meas');
    title('Roll (deg)')
    linkaxes(simlog_handles, 'x')
    hold all

    % Y
    simlog_handles(4) = subplot(3, 3, 4);
    plot(simlog_t, simlog_py, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos y (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(5) = subplot(3, 3, 5);
    plot(simlog_t, simlog_vy, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel y (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(6) = subplot(3, 3, 6);
    plot(simlog_t, squeeze(simlog_qy)*180/pi, 'LineWidth', 1,'DisplayName','Meas');
    title('Pitch (deg)')
    linkaxes(simlog_handles, 'x')
    hold all

    % Z
    simlog_handles(7) = subplot(3, 3, 7);
    plot(simlog_t, simlog_pz, 'LineWidth', 1,'DisplayName','Meas');
    title('Pos z (m)')
    linkaxes(simlog_handles, 'x')
    hold all

    simlog_handles(8) = subplot(3, 3, 8);
    plot(simlog_t, simlog_vz, 'LineWidth', 1,'DisplayName','Meas');
    title('Vel z (m/s)')
    linkaxes(simlog_handles, 'x')
    hold all
    
    simlog_handles(9) = subplot(3, 3, 9);
    plot(simlog_t, squeeze(simlog_qz)*180/pi, 'LineWidth', 1,'DisplayName','Meas');
    title('Yaw (deg)')
    linkaxes(simlog_handles, 'x')
    xlabel('Time (s)')

    hold all
    legendstr{i} = sprintf('maneuver %f',i);
end
    legend(legendstr,'Location','best')
    xlabel('Time (s)')
    clear simlog_handles
end

%% Save Sim Data

function save_sim_data(simInput, simOut)
    for i=1:length(simOut)
    simlog_px = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.px.Data;
    simlog_py = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.py.Data;
    simlog_pz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.pz.Data;
    simlog_vx = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vx.Data;
    simlog_vy = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vy.Data;
    simlog_vz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.vz.Data;
    simlog_qx = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.roll.Data;
    simlog_qy = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.pitch.Data;
    simlog_qz = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.yaw.Data;
    simlog_t  = simOut(i).logsout_quadcopter_package_delivery.get('Quadcopter').Values.Chassis.px.Time;
    
    result_matrix = [simlog_t,simlog_px, simlog_py, ...
        simlog_pz, simlog_vx, simlog_vy, ...
        simlog_vz,squeeze(simlog_qx)*180/pi,squeeze(simlog_qy)*180/pi,squeeze(simlog_qz)*180/pi];
    result_matrix_filename = ['run\fault_',num2str(1),'_maneuver_', num2str(i)];
    writematrix(result_matrix, result_matrix_filename);
    end
end