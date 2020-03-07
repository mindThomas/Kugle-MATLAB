Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
          q(2) q(1)  -q(4) q(3);
          q(3) q(4)  q(1)  -q(2);
          q(4) -q(3) q(2)  q(1)];           
Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
             p(2) p(1) p(4) -p(3);
             p(3) -p(4) p(1) p(2);
             p(4) p(3) -p(2) p(1)];  

devec = [0,1,0,0;0,0,1,0;0,0,0,1]; % 'v' in notes
vec = [0,0,0;1,0,0;0,1,0;0,0,1]; % '^' in notes
I_conj = diag([1,-1,-1,-1]);

addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\DataProcessing\functions');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Model\generated');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Parameters');
Constants_Kugle;
Parameters_General;
Parameters_Controllers;

DumpFolder = [pwd '\'];
data = LoadDump(DumpFolder, '');
vicon = LoadVicon(DumpFolder, '');
[data, vicon] = DumpViconTimeSynchronization(data, vicon);
[data, vicon] = TrimSynced(data, vicon, 15);

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

%% Orientation
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.mti_roll), data.time, rad2deg(data.roll)); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'MTI', 'QEKF');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.mti_pitch), data.time, rad2deg(data.pitch)); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'MTI', 'QEKF');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.mti_yaw), data.time, rad2deg(data.yaw)); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'MTI', 'QEKF');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Bias
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Bias');
ax1 = subplot(3,1,1); plot(data.time, data.gyro_bias(:,1)); ylabel('Angular velocity [rad/s]'); title('X'); legend('MTI Gyro', 'Estimated bias');
ax2 = subplot(3,1,2); plot(data.time, data.gyro_bias(:,2)); ylabel('Angular velocity [rad/s]'); title('Y'); legend('MTI Gyro', 'Estimated bias');
ax3 = subplot(3,1,3); plot(data.time, data.gyro_bias(:,3)); ylabel('Angular velocity [rad/s]'); title('Z'); legend('MTI Gyro', 'Estimated bias');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Store data
out = [data.time, data.gyro_bias(:,1), data.gyro_bias(:,2)];
out = Downsample(out, 10); % downsample plot data
headers = {'time', 'bias_x', 'bias_y'};
csvwrite_with_headers('bias.csv', out, headers);