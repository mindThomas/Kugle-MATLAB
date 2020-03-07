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
[data, vicon] = TrimSynced(data, vicon, 20+3.432-2, 22);

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

eul_ref = quat2eul(data.q_ref, 'ZYX');
roll_ref = eul_ref(:,3);
pitch_ref = eul_ref(:,2);
yaw_ref = eul_ref(:,1);
    
%% Velocity with disturbance marks
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Velocity comparison');
ax1 = subplot(2,1,1); plot(vicon.time, vicon.velocity(:,1), data.time, data.velocity(:,1)); ylabel('m/s'); title('dx'); legend('Vicon', 'VEKF');
ax2 = subplot(2,1,2); plot(vicon.time, vicon.velocity(:,2), data.time, data.velocity(:,2)); ylabel('m/s'); title('dy'); legend('Vicon', 'VEKF');
linkaxes([ax1,ax2],'x');
xlabel('Time [s]');

%% Orientation
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.roll), data.time, rad2deg(roll_ref)); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'QEKF', 'Reference');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(pitch_ref)); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'QEKF', 'Reference');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.yaw), data.time, rad2deg(yaw_ref)); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'QEKF', 'Reference');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Position
fig = figure(3); set(fig, 'NumberTitle', 'off', 'Name', 'Position');
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


%% Angular velocity body
fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity');
ax1 = subplot(3,1,1); plot(vicon.time, vicon.omega_body(:,1), data.time, data.mti_omega_body(:,1), data.time, data.omega_body(:,1)); ylabel('Angular velocity [rad/s]'); title('omega body x'); legend('Vicon', 'MTI', 'QEKF');
ax2 = subplot(3,1,2); plot(vicon.time, vicon.omega_body(:,2), data.time, data.mti_omega_body(:,2), data.time, data.omega_body(:,2)); ylabel('Angular velocity [rad/s]'); title('omega body y'); legend('Vicon', 'MTI', 'QEKF');
ax3 = subplot(3,1,3); plot(vicon.time, vicon.omega_body(:,3), data.time, data.mti_omega_body(:,3), data.time, data.omega_body(:,3)); ylabel('Angular velocity [rad/s]'); title('omega body z'); legend('Vicon', 'MTI', 'QEKF');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Quaternion
fig = figure(5); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion');
ax1 = subplot(4,1,1); plot(vicon.time, vicon.q(:,1), data.time, data.q(:,1)); ylabel('q0'); title('q0'); legend('Vicon', 'QEKF');
ax2 = subplot(4,1,2); plot(vicon.time, vicon.q(:,2), data.time, data.q(:,2)); ylabel('q1'); title('q1'); legend('Vicon', 'QEKF');
ax3 = subplot(4,1,3); plot(vicon.time, vicon.q(:,3), data.time, data.q(:,3)); ylabel('q2'); title('q2'); legend('Vicon', 'QEKF');
ax4 = subplot(4,1,4); plot(vicon.time, vicon.q(:,4), data.time, data.q(:,4)); ylabel('q3'); title('q3'); legend('Vicon', 'QEKF');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');

%% Orientation derivative
fig = figure(6); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion derivative');
ax1 = subplot(4,1,1); plot(vicon.time, vicon.dq(:,1), data.time, data.mti_dq(:,1), data.time, data.dq(:,1)); ylabel('dq0'); title('dq0'); legend('Vicon', 'MTI', 'QEKF');
ax2 = subplot(4,1,2); plot(vicon.time, vicon.dq(:,2), data.time, data.mti_dq(:,2), data.time, data.dq(:,2)); ylabel('dq1'); title('dq1'); legend('Vicon', 'MTI', 'QEKF');
ax3 = subplot(4,1,3); plot(vicon.time, vicon.dq(:,3), data.time, data.mti_dq(:,3), data.time, data.dq(:,3)); ylabel('dq2'); title('dq2'); legend('Vicon', 'MTI', 'QEKF');
ax4 = subplot(4,1,4); plot(vicon.time, vicon.dq(:,4), data.time, data.mti_dq(:,4), data.time, data.dq(:,4)); ylabel('dq3'); title('dq3'); legend('Vicon', 'MTI', 'QEKF');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');


%% Sliding variable
fig = figure(7); set(fig, 'NumberTitle', 'off', 'Name', 'Sliding variable');
ax1 = subplot(3,1,1); plot(data.time, data.S(:,1));
ax2 = subplot(3,1,2); plot(data.time, data.S(:,2));
ax3 = subplot(3,1,3); plot(data.time, data.S(:,3));
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
out = [vicon.time, vicon.velocity(:,1:2), rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw), vicon.q];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'velocity_x', 'velocity_y', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3'};
csvwrite_with_headers('step_test_vicon.csv', out, headers);

out = [data.time, data.velocity, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), rad2deg(roll_ref), rad2deg(pitch_ref), rad2deg(yaw_ref), data.q, data.S, data.torque];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'velocity_x', 'velocity_y', 'roll', 'pitch', 'yaw', 'roll_ref', 'pitch_ref', 'yaw_ref', 'q0', 'q1', 'q2', 'q3', 's0', 's1', 's2', 'tau0', 'tau1', 'tau2'};
csvwrite_with_headers('step_test_system.csv', out, headers);