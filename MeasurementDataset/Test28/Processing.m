addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\DataProcessing\functions');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Model\generated');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Parameters');
Constants_Kugle;
Parameters_General;
Parameters_Estimators;

DumpFolder = [pwd '\'];
data = LoadDump(DumpFolder, '');
vicon = LoadVicon(DumpFolder, '');
[data, vicon] = DumpViconTimeSynchronization(data, vicon);

%%
vicon = RemoveTime(vicon, 11.10, 11.14); % Fix glitches in data
vicon = RemoveTime(vicon, 43.06, 43.16); % Fix glitches in data
vicon = RemoveTime(vicon, 83.30, 83.35); % Fix glitches in data
vicon = RemoveTime(vicon, 106.5, 107); % Fix glitches in data
vicon = RemoveTime(vicon, 133.7, 133.85); % Fix glitches in data
vicon = RemoveTime(vicon, 141.2, 141.45); % Fix glitches in data
vicon = RemoveTime(vicon, 172.65, 172.85); % Fix glitches in data
vicon = RemoveTime(vicon, 195.1, 195.25); % Fix glitches in data
vicon = RemoveTime(vicon, 201.5, 202.7); % Fix glitches in data
vicon = RemoveTime(vicon, 207.0, 207.35); % Fix glitches in data
vicon = RemoveTime(vicon, 210.2, 210.3); % Fix glitches in data
vicon = RemoveTime(vicon, 211.30, 211.6); % Fix glitches in data
vicon = RemoveTime(vicon, 242.3, 242.5); % Fix glitches in data
vicon = RemoveTime(vicon, 242.60, 242.9); % Fix glitches in data
vicon = RemoveTime(vicon, 248.9, 249.1); % Fix glitches in data
vicon = RemoveTime(vicon, 249.25, 249.5); % Fix glitches in data
vicon = RemoveTime(vicon, 249.9, 250.25); % Fix glitches in data
vicon = RemoveTime(vicon, 251.4, 251.55); % Fix glitches in data
vicon = RemoveTime(vicon, 255.23, 255.71); % Fix glitches in data
vicon = RemoveTime(vicon, 261.57, 261.98); % Fix glitches in data
vicon = RemoveTime(vicon, 268.85, 269.08); % Fix glitches in data
vicon = RemoveTime(vicon, 309.85, 309.92); % Fix glitches in data
vicon = RemoveTime(vicon, 310.03, 310.3); % Fix glitches in data

[data, vicon] = TrimSynced(data, vicon, 31.85+23+9, 53); % arbitrary joystick references
%[data, vicon] = TrimSynced(data, vicon, 31.85+40+137.8+11.6, 88.54-11.6); % circle test
%[data, vicon] = TrimSynced(data, vicon, 200);

% Aligns MTI and Vicon yaw with estimated yaw
data.yaw = data.yaw + 2*pi;
EstMTIyawOffset = data.yaw(1) - data.mti_yaw(1);
[data.mti_q, data.mti_dq, data.mti_omega_inertial] = QuaternionYawOffset(data.mti_q, data.mti_dq, EstMTIyawOffset, false);
mti_eul = quat2eul(data.mti_q);
data.mti_roll = mti_eul(:,3);
data.mti_pitch = mti_eul(:,2);
data.mti_yaw = unwrap(mti_eul(:,1));
if (sign(data.yaw(1)) ~= sign(data.mti_yaw(1)))
    data.mti_yaw = data.mti_yaw + 2*pi*sign(data.yaw(1));
end

EstViconyawOffset = data.yaw(1) - vicon.yaw(1);
[vicon.q, vicon.dq, vicon.omega_inertial] = QuaternionYawOffset(vicon.q, vicon.dq, EstViconyawOffset, false);
vicon_eul = quat2eul(vicon.q);
vicon.roll = vicon_eul(:,3);
vicon.pitch = vicon_eul(:,2);
vicon.yaw = unwrap(vicon_eul(:,1));
if (sign(data.yaw(1)) ~= sign(vicon.yaw(1)))
    vicon.yaw = vicon.yaw + 2*pi*sign(data.yaw(1));
end
vicon.position = (rotz(EstViconyawOffset)*vicon.position')';
vicon.velocity = (rotz(EstViconyawOffset)*vicon.velocity')';
vicon.velocity_heading = (rotz(EstViconyawOffset)*vicon.velocity_heading')';

% Align initial Vicon position with internal estimate
data.position = data.position - data.position(1,:);
vicon.position(:,1:2) = vicon.position(:,1:2) - (vicon.position(1,1:2) - data.position(1,:));

% Compute Euler references
eul_ref = quat2eul(data.q_ref, 'ZYX');
roll_ref = eul_ref(:,3);
pitch_ref = eul_ref(:,2);
yaw_ref = unwrap(eul_ref(:,1));
if (sign(data.yaw(1)) ~= sign(yaw_ref(1)))
    yaw_ref = yaw_ref + 2*pi*sign(data.yaw(1));
end


%% Velocity
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Velocity');
ax1 = subplot(2,1,1); plot(vicon.time, vicon.velocity_heading(:,1), data.time, data.velocity_heading(:,1), data.time, data.velocity_ref_heading(:,1)); ylabel('m/s'); title('dx'); legend('Vicon', 'VEKF', 'Reference');
ax2 = subplot(2,1,2); plot(vicon.time, vicon.velocity_heading(:,2), data.time, data.velocity_heading(:,2), data.time, data.velocity_ref_heading(:,2)); ylabel('m/s'); title('dy'); legend('Vicon', 'VEKF', 'Reference');
linkaxes([ax1,ax2],'x');
xlabel('Time [s]');

%% Position
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Position');
ax1 = subplot(2,1,1); plot(vicon.time, vicon.position(:,1), data.time, data.position(:,1)); ylabel('m'); title('x'); legend('Vicon', 'Estimated');
ax2 = subplot(2,1,2); plot(vicon.time, vicon.position(:,2), data.time, data.position(:,2)); ylabel('m'); title('y'); legend('Vicon', 'Estimated');
linkaxes([ax1,ax2],'x');
xlabel('Time [s]');

%% Orientation
fig = figure(3); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.roll), data.time, rad2deg(roll_ref)); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'QEKF', 'Reference');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(pitch_ref)); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'QEKF', 'Reference');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.yaw), data.time, rad2deg(yaw_ref)); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'QEKF', 'Reference');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Angular velocity body
fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity');
ax1 = subplot(3,1,1); plot(vicon.time, vicon.omega_body(:,1), data.time, data.mti_omega_body(:,1), data.time, data.omega_body(:,1), data.time, data.omega_ref_body(:,1)); ylabel('Angular velocity [rad/s]'); title('omega body x'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
ax2 = subplot(3,1,2); plot(vicon.time, vicon.omega_body(:,2), data.time, data.mti_omega_body(:,2), data.time, data.omega_body(:,2), data.time, data.omega_ref_body(:,2)); ylabel('Angular velocity [rad/s]'); title('omega body y'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
ax3 = subplot(3,1,3); plot(vicon.time, vicon.omega_body(:,3), data.time, data.mti_omega_body(:,3), data.time, data.omega_body(:,3), data.time, data.omega_ref_body(:,3)); ylabel('Angular velocity [rad/s]'); title('omega body z'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Torque
fig = figure(8); set(fig, 'NumberTitle', 'off', 'Name', 'Torque');
ax1 = subplot(3,1,1); plot(data.time, data.torque(:,1), data.time, data.torque_delivered(:,1)); ylabel('Nm'); title('Motor 0'); legend('Torque control', 'Torque delivered');
ax2 = subplot(3,1,2); plot(data.time, data.torque(:,2), data.time, data.torque_delivered(:,2)); ylabel('Nm'); title('Motor 1'); legend('Torque control', 'Torque delivered');
ax3 = subplot(3,1,3); plot(data.time, data.torque(:,3), data.time, data.torque_delivered(:,3)); ylabel('Nm'); title('Motor 2'); legend('Torque control', 'Torque delivered');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Position
fig = figure(10); set(fig, 'NumberTitle', 'off', 'Name', 'Position');
t = vicon.time;
x = vicon.position(:,2)';
y = -vicon.position(:,1)';
z = vicon.position(:,3)';
col = t / t(end) * 1000;  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col';col'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
axis equal;  
grid;
xlabel('Time [s]');


%% Store data
% out = [vicon.time, vicon.position(:,1:2), vicon.velocity(:,1:2), vicon.velocity_heading(:,1:2), rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw), vicon.q, vicon.omega_body];
% out = Downsample(out, 2); % downsample plot data
% headers = {'time', 'position_x', 'position_y', 'velocity_x', 'velocity_y', 'velocity_heading_x', 'velocity_heading_y', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z'};
% csvwrite_with_headers('velocity_circle_vicon.csv', out, headers);
% 
% out = [data.time, data.position, data.velocity, data.velocity_heading, data.velocity_ref_inertial, data.velocity_ref_heading, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), rad2deg(roll_ref), rad2deg(pitch_ref), rad2deg(yaw_ref), data.q, data.omega_body, data.omega_ref_body, data.S, data.torque, data.torque_delivered];
% out = Downsample(out, 2); % downsample plot data
% headers = {'time', 'position_x', 'position_y', 'velocity_x', 'velocity_y', 'velocity_heading_x', 'velocity_heading_y', 'velocity_ref_x', 'velocity_ref_y', 'velocity_heading_ref_x', 'velocity_heading_ref_y', 'roll', 'pitch', 'yaw', 'roll_ref', 'pitch_ref', 'yaw_ref', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z', 'omega_body_ref_x', 'omega_body_ref_y', 'omega_body_ref_z', 's0', 's1', 's2', 'tau0', 'tau1', 'tau2', 'tau0_delivered', 'tau1_delivered', 'tau2_delivered'};
% csvwrite_with_headers('velocity_circle_system.csv', out, headers);