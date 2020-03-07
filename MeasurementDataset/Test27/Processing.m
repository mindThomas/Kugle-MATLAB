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
[data, vicon] = TrimSynced(data, vicon, 13.55, 65);

% Aligns MTI and Vicon yaw with estimated yaw
EstMTIyawOffset = data.yaw(1) - data.mti_yaw(1);
[data.mti_q, data.mti_dq, data.mti_omega_inertial] = QuaternionYawOffset(data.mti_q, data.mti_dq, EstMTIyawOffset, true);
mti_eul = quat2eul(data.mti_q);
data.mti_roll = mti_eul(:,3);
data.mti_pitch = mti_eul(:,2);
data.mti_yaw = unwrap(mti_eul(:,1));

EstViconyawOffset = data.yaw(1) - vicon.yaw(1);
[vicon.q, vicon.dq, vicon.mti_omega_inertial] = QuaternionYawOffset(vicon.q, vicon.dq, EstViconyawOffset, true);
vicon_eul = quat2eul(vicon.q);
vicon.roll = vicon_eul(:,3);
vicon.pitch = vicon_eul(:,2);
vicon.yaw = unwrap(vicon_eul(:,1));
vicon.position = (rotz(EstViconyawOffset)*vicon.position')';
vicon.velocity = (rotz(EstViconyawOffset)*vicon.velocity')';

% Align initial Vicon position with internal estimate
data.position = data.position - data.position(1,:);
vicon.position(:,1:2) = vicon.position(:,1:2) - (vicon.position(1,1:2) - data.position(1,:));

% Compute Euler references
eul_ref = quat2eul(data.q_ref, 'ZYX');
roll_ref = eul_ref(:,3);
pitch_ref = eul_ref(:,2);
yaw_ref = eul_ref(:,1);

%% Velocity
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Velocity');
ax1 = subplot(2,1,1); plot(vicon.time, vicon.velocity(:,1), data.time, data.velocity(:,1), data.time, data.velocity_ref_inertial(:,1)); ylabel('m/s'); title('dx'); legend('Vicon', 'VEKF', 'Reference');
ax2 = subplot(2,1,2); plot(vicon.time, vicon.velocity(:,2), data.time, data.velocity(:,2), data.time, data.velocity_ref_inertial(:,2)); ylabel('m/s'); title('dy'); legend('Vicon', 'VEKF', 'Reference');
linkaxes([ax1,ax2],'x');
xlabel('Time [s]');

%% Position references
data.position_ref = repmat([0.18, 0.02], [length(data.position), 1]);

% Position
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Position');
ax1 = subplot(2,1,1); plot(vicon.time, vicon.position(:,1), data.time, data.position(:,1), data.time, data.position_ref(:,1)); ylabel('m'); title('x'); legend('Vicon', 'Estimated', 'Reference');
ax2 = subplot(2,1,2); plot(vicon.time, vicon.position(:,2), data.time, data.position(:,2), data.time, data.position_ref(:,2)); ylabel('m'); title('y'); legend('Vicon', 'Estimated', 'Reference');
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


%% Store data
out = [vicon.time, vicon.position(:,1:2), vicon.velocity(:,1:2), rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw), vicon.q, vicon.omega_body];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'position_x', 'position_y', 'velocity_x', 'velocity_y', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z'};
csvwrite_with_headers('station_keeping_nonaggressive_static_vicon.csv', out, headers);

out = [data.time, data.position, data.velocity, data.velocity_ref_inertial, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), rad2deg(roll_ref), rad2deg(pitch_ref), rad2deg(yaw_ref), data.q, data.omega_body, data.omega_ref_body, data.S, data.torque, data.torque_delivered];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'position_x', 'position_y', 'velocity_x', 'velocity_y', 'velocity_ref_x', 'velocity_ref_y', 'roll', 'pitch', 'yaw', 'roll_ref', 'pitch_ref', 'yaw_ref', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z', 'omega_body_ref_x', 'omega_body_ref_y', 'omega_body_ref_z', 's0', 's1', 's2', 'tau0', 'tau1', 'tau2', 'tau0_delivered', 'tau1_delivered', 'tau2_delivered'};
csvwrite_with_headers('station_keeping_nonaggressive_static_system.csv', out, headers);