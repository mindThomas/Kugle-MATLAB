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
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Estimators\Madgwick');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Estimators\Madgwick\quaternion_library');
Constants_Kugle;
Parameters_General;
Parameters_Controllers; % for sample rate

DumpFolder = [pwd '\'];
data = LoadDump(DumpFolder, '');
vicon = LoadVicon(DumpFolder, '');
[data, vicon] = DumpViconTimeSynchronization(data, vicon);
[data, vicon] = TrimSynced(data, vicon, 92, 90); % 92 sync, 90 length

% Aligns MTI and Vicon yaw with estimated yaw
EstMTIyawOffset = data.yaw(1) - data.mti_yaw(1);
[data.mti_q, data.mti_dq, data.mti_omega_inertial] = QuaternionYawOffset(data.mti_q, data.mti_dq, EstMTIyawOffset, true);
mti_eul = quat2eul(data.mti_q);
data.mti_roll = mti_eul(:,3);
data.mti_pitch = mti_eul(:,2);
data.mti_yaw = unwrap(mti_eul(:,1));

EstViconyawOffset = data.yaw(1) - vicon.yaw(1);
[vicon.q, vicon.dq, vicon.omega_inertial] = QuaternionYawOffset(vicon.q, vicon.dq, EstViconyawOffset, true);
vicon_eul = quat2eul(vicon.q);
vicon.roll = vicon_eul(:,3);
vicon.pitch = vicon_eul(:,2);
vicon.yaw = unwrap(vicon_eul(:,1));
vicon.position = (rotz(EstViconyawOffset)*vicon.position')';
vicon.velocity = (rotz(EstViconyawOffset)*vicon.velocity')';


%% Process sensor data through Madgwick algorithm
AHRS = MadgwickAHRS('SamplePeriod', Ts, 'Beta', 0.001);
% AHRS = MahonyAHRS('SamplePeriod', Ts, 'Kp', 0.5);

AHRS.Quaternion = vicon.q(1,:); % initialize filter

madgwick.q = zeros(length(data.time), 4);
madgwick.dq = zeros(length(data.time), 4);
madgwick.omega_body = zeros(length(data.time), 3);
for t = 1:length(data.time)    
    AHRS.UpdateIMU(data.gyroscope(t,:), data.accelerometer(t,:));	% gyroscope units must be radians
    madgwick.q(t,:) = AHRS.Quaternion;
    madgwick.dq(t,:) = AHRS.QuaternionDerivative;
    madgwick.omega_body(t,:) = ( 2 * devec * Phi(madgwick.q(t,:)')' * madgwick.dq(t,:)' )';
end
madgwick_eul = quat2eul(madgwick.q);
madgwick.roll = madgwick_eul(:,3);
madgwick.pitch = madgwick_eul(:,2);
madgwick.yaw = unwrap(madgwick_eul(:,1));

%% Orientation
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.mti_roll), data.time, rad2deg(data.roll), data.time, rad2deg(madgwick.roll)); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.mti_pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(madgwick.pitch)); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.mti_yaw), data.time, rad2deg(data.yaw), data.time, rad2deg(madgwick.yaw)); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Angular velocity body
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity');
ax1 = subplot(3,1,1); plot(vicon.time, vicon.omega_body(:,1), data.time, data.mti_omega_body(:,1), data.time, data.omega_body(:,1), data.time, madgwick.omega_body(:,1)); ylabel('Angular velocity [rad/s]'); title('omega body x'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
ax2 = subplot(3,1,2); plot(vicon.time, vicon.omega_body(:,2), data.time, data.mti_omega_body(:,2), data.time, data.omega_body(:,2), data.time, madgwick.omega_body(:,2)); ylabel('Angular velocity [rad/s]'); title('omega body y'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
ax3 = subplot(3,1,3); plot(vicon.time, vicon.omega_body(:,3), data.time, data.mti_omega_body(:,3), data.time, data.omega_body(:,3), data.time, madgwick.omega_body(:,3)); ylabel('Angular velocity [rad/s]'); title('omega body z'); legend('Vicon', 'MTI', 'QEKF', 'Madgwick');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Quaternion
fig = figure(3); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion');
ax1 = subplot(4,1,1); plot(vicon.time, vicon.q(:,1), data.time, data.mti_q(:,1), data.time, data.q(:,1)); ylabel('q0'); title('q0'); legend('Vicon', 'MTI', 'QEKF');
ax2 = subplot(4,1,2); plot(vicon.time, vicon.q(:,2), data.time, data.mti_q(:,2), data.time, data.q(:,2)); ylabel('q1'); title('q1'); legend('Vicon', 'MTI', 'QEKF');
ax3 = subplot(4,1,3); plot(vicon.time, vicon.q(:,3), data.time, data.mti_q(:,3), data.time, data.q(:,3)); ylabel('q2'); title('q2'); legend('Vicon', 'MTI', 'QEKF');
ax4 = subplot(4,1,4); plot(vicon.time, vicon.q(:,4), data.time, data.mti_q(:,4), data.time, data.q(:,4)); ylabel('q3'); title('q3'); legend('Vicon', 'MTI', 'QEKF');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');

%% Orientation derivative
fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion derivative');
ax1 = subplot(4,1,1); plot(vicon.time, vicon.dq(:,1), data.time, data.mti_dq(:,1), data.time, data.dq(:,1)); ylabel('dq0'); title('dq0'); legend('Vicon', 'MTI', 'QEKF');
ax2 = subplot(4,1,2); plot(vicon.time, vicon.dq(:,2), data.time, data.mti_dq(:,2), data.time, data.dq(:,2)); ylabel('dq1'); title('dq1'); legend('Vicon', 'MTI', 'QEKF');
ax3 = subplot(4,1,3); plot(vicon.time, vicon.dq(:,3), data.time, data.mti_dq(:,3), data.time, data.dq(:,3)); ylabel('dq2'); title('dq2'); legend('Vicon', 'MTI', 'QEKF');
ax4 = subplot(4,1,4); plot(vicon.time, vicon.dq(:,4), data.time, data.mti_dq(:,4), data.time, data.dq(:,4)); ylabel('dq3'); title('dq3'); legend('Vicon', 'MTI', 'QEKF');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');


%% Orientation error
mti_eul = [data.mti_roll, data.mti_pitch, data.mti_yaw];
qekf_eul = [data.roll, data.pitch, data.yaw];
vicon_eul = [vicon.roll, vicon.pitch, vicon.yaw];
vicon_eul2 = interp1(vicon.time, vicon_eul, data.time, 'linear', 'extrap');
eul_error_mti = qekf_eul - mti_eul;
eul_error_vicon = qekf_eul - vicon_eul2;
fig = figure(5); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation estimation error');
ax1 = subplot(3,1,1); plot(data.time, rad2deg(eul_error_mti(:,1)), data.time, rad2deg(eul_error_vicon(:,1))); ylabel('Angle error [deg]'); title('Roll'); legend('QEKF - MTI', 'QEKF - Vicon');
ax2 = subplot(3,1,2); plot(data.time, rad2deg(eul_error_mti(:,2)), data.time, rad2deg(eul_error_vicon(:,2))); ylabel('Angle error [deg]'); title('Pitch'); legend('QEKF - MTI', 'QEKF - Vicon');
ax3 = subplot(3,1,3); plot(data.time, rad2deg(eul_error_mti(:,3)), data.time, rad2deg(eul_error_vicon(:,3))); ylabel('Angle error [deg]'); title('Yaw'); legend('QEKF - MTI', 'QEKF - Vicon');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Angular velocity error
vicon_omega_body2 = interp1(vicon.time, vicon.omega_body, data.time, 'linear', 'extrap');
omega_body_error_mti = data.omega_body - data.mti_omega_body;
omega_body_error_vicon = data.omega_body - vicon_omega_body2;
fig = figure(6); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity estimation error');
ax1 = subplot(3,1,1); plot(data.time, omega_body_error_mti(:,1), data.time, omega_body_error_vicon(:,1)); ylabel('Angular velocity error [deg]'); title('Roll'); legend('QEKF - MTI', 'QEKF - Vicon');
ax2 = subplot(3,1,2); plot(data.time, omega_body_error_mti(:,2), data.time, omega_body_error_vicon(:,2)); ylabel('Angular velocity error [deg]'); title('Pitch'); legend('QEKF - MTI', 'QEKF - Vicon');
ax3 = subplot(3,1,3); plot(data.time, omega_body_error_mti(:,3), data.time, omega_body_error_vicon(:,3)); ylabel('Angular velocity error [deg]'); title('Yaw'); legend('QEKF - MTI', 'QEKF - Vicon');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Quaternion estimation error covariance
cov_diag = [data.Cov_q(:,1), data.Cov_q(:,6), data.Cov_q(:,11), data.Cov_q(:,16)];
fig = figure(5); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion estimation error covariance');
ax1 = subplot(4,1,1); plot(data.time, cov_diag(:,1)); ylabel('Covariance'); title('q0');
ax2 = subplot(4,1,2); plot(data.time, cov_diag(:,2)); ylabel('Covariance'); title('q1');
ax3 = subplot(4,1,3); plot(data.time, cov_diag(:,3)); ylabel('Covariance'); title('q2');
ax4 = subplot(4,1,4); plot(data.time, cov_diag(:,4)); ylabel('Covariance'); title('q3');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');

return;

%% Angular velocity output
% out = [data.time, data.q, data.dq, data.omega_body, data.mti_q, data.mti_dq, data.mti_omega_body];
% out = Downsample(out, 2); % downsample plot data
% headers = {'time', 'qekf_q0', 'qekf_q1', 'qekf_q2', 'qekf_q3', 'qekf_dq0', 'qekf_dq1', 'qekf_dq2', 'qekf_dq3', 'qekf_omega_body_x', 'qekf_omega_body_z', 'qekf_omega_body_y', 'mti_q0', 'mti_q1', 'mti_q2', 'mti_q3', 'mti_dq0', 'mti_dq1', 'mti_dq2', 'mti_dq3', 'mti_omega_body_x', 'mti_omega_body_y', 'mti_omega_body_z'};
% csvwrite_with_headers('angular_velocity_zoom3.csv', out, headers);

%% Angle comparison output
out = [data.time, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), data.q, data.dq, data.omega_body, rad2deg(data.mti_roll), rad2deg(data.mti_pitch), rad2deg(data.mti_yaw), data.mti_q, data.mti_dq, data.mti_omega_body, rad2deg(madgwick.roll), rad2deg(madgwick.pitch), rad2deg(madgwick.yaw), madgwick.q, madgwick.dq, madgwick.omega_body, rad2deg(eul_error_mti), omega_body_error_mti, rad2deg(eul_error_vicon), omega_body_error_vicon];
out = Downsample(out, 3); % downsample plot data
headers = {'time', ...
    'qekf_roll', 'qekf_pitch', 'qekf_yaw', ...
    'qekf_q0', 'qekf_q1', 'qekf_q2', 'qekf_q3', ...
    'qekf_dq0', 'qekf_dq1', 'qekf_dq2', 'qekf_dq3', ...
    'qekf_omega_body_x', 'qekf_omega_body_y', 'qekf_omega_body_z', ...
    'mti_roll', 'mti_pitch', 'mti_yaw', ...
    'mti_q0', 'mti_q1', 'mti_q2', 'mti_q3', ...
    'mti_dq0', 'mti_dq1', 'mti_dq2', 'mti_dq3', ...
    'mti_omega_body_x', 'mti_omega_body_y', 'mti_omega_body_z', ...
    'madgwick_roll', 'madgwick_pitch', 'madgwick_yaw', ...
    'madgwick_q0', 'madgwick_q1', 'madgwick_q2', 'madgwick_q3', ...
    'madgwick_dq0', 'madgwick_dq1', 'madgwick_dq2', 'madgwick_dq3', ...
    'madgwick_omega_body_x', 'madgwick_omega_body_y', 'madgwick_omega_body_z', ...
    'roll_error_mti', 'pitch_error_mti', 'yaw_error_mti', ...
    'omega_body_error_mti_x', 'omega_body_error_mti_y', 'omega_body_error_mti_z', ...
    'roll_error_vicon', 'pitch_error_vicon', 'yaw_error_vicon', ...
    'omega_body_error_vicon_x', 'omega_body_error_vicon_y', 'omega_body_error_vicon_z' ...
};
csvwrite_with_headers('system.csv', out, headers);

out = [vicon.time, rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw), vicon.q, vicon.dq, vicon.omega_body];
out = Downsample(out, 3); % downsample plot data
headers = {'time', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3', 'dq0', 'dq1', 'dq2', 'dq3', 'omega_body_x', 'omega_body_y', 'omega_body_z'};
csvwrite_with_headers('vicon.csv', out, headers);