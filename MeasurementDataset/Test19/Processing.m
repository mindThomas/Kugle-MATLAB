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
[data, vicon] = TrimSynced(data, vicon, 93.93, 52.45+5);

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
    
%% Orientation
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.roll), data.time, rad2deg(roll_ref)); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'QEKF', 'Reference');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(pitch_ref)); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'QEKF', 'Reference');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.yaw), data.time, rad2deg(yaw_ref)); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'QEKF', 'Reference');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Orientation error
eul_ref = [data.roll_ref, data.pitch_ref, data.yaw_ref];
qekf_eul = [data.roll, data.pitch, data.yaw];
vicon_eul = [vicon.roll, vicon.pitch, vicon.yaw];
vicon_eul2 = interp1(vicon.time, vicon_eul, data.time, 'linear', 'extrap');
eul_error_qekf = qekf_eul - eul_ref;
eul_error_vicon = vicon_eul2 - eul_ref;
fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation tracking error');
ax1 = subplot(3,1,1); plot(data.time, rad2deg(eul_error_qekf(:,1)), data.time, rad2deg(eul_error_vicon(:,1))); ylabel('Angle error [deg]'); title('Roll'); legend('Ref - QEK', 'Ref - Vicon');
ax2 = subplot(3,1,2); plot(data.time, rad2deg(eul_error_qekf(:,2)), data.time, rad2deg(eul_error_vicon(:,2))); ylabel('Angle error [deg]'); title('Pitch'); legend('Ref - QEK', 'Ref - Vicon');
ax3 = subplot(3,1,3); plot(data.time, rad2deg(eul_error_qekf(:,3)), data.time, rad2deg(eul_error_vicon(:,3))); ylabel('Angle error [deg]'); title('Yaw'); legend('Ref - QEK', 'Ref - Vicon');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Angular velocity body
fig = figure(3); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity');
ax1 = subplot(3,1,1); plot(vicon.time, vicon.omega_body(:,1), data.time, data.mti_omega_body(:,1), data.time, data.omega_body(:,1), data.time, data.omega_ref_body(:,1)); ylabel('Angular velocity [rad/s]'); title('omega body x'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
ax2 = subplot(3,1,2); plot(vicon.time, vicon.omega_body(:,2), data.time, data.mti_omega_body(:,2), data.time, data.omega_body(:,2), data.time, data.omega_ref_body(:,2)); ylabel('Angular velocity [rad/s]'); title('omega body y'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
ax3 = subplot(3,1,3); plot(vicon.time, vicon.omega_body(:,3), data.time, data.mti_omega_body(:,3), data.time, data.omega_body(:,3), data.time, data.omega_ref_body(:,3)); ylabel('Angular velocity [rad/s]'); title('omega body z'); legend('Vicon', 'MTI', 'QEKF', 'Reference');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');

%% Angular velocity error
omega_body_error_qekf = data.omega_body - data.omega_ref_body;
vicon_omega_body = interp1(vicon.time, vicon.omega_body, data.time, 'linear', 'extrap');
omega_body_error_vicon = vicon_omega_body - data.omega_ref_body;
fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity error');
ax1 = subplot(3,1,1); plot(data.time, omega_body_error_qekf(:,1), data.time, omega_body_error_vicon(:,1)); ylabel('Angular velocity [rad/s]'); title('omega body x'); legend('Ref - QEK', 'Ref - Vicon');
ax2 = subplot(3,1,2); plot(data.time, omega_body_error_qekf(:,2), data.time, omega_body_error_vicon(:,2)); ylabel('Angular velocity [rad/s]'); title('omega body y'); legend('Ref - QEK', 'Ref - Vicon');
ax3 = subplot(3,1,3); plot(data.time, omega_body_error_qekf(:,3), data.time, omega_body_error_vicon(:,3)); ylabel('Angular velocity [rad/s]'); title('omega body z'); legend('Ref - QEK', 'Ref - Vicon');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%% Store data
out = [vicon.time, vicon.velocity(:,1:2), rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw), vicon.q, vicon.omega_body];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'velocity_x', 'velocity_y', 'roll', 'pitch', 'yaw', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z'};
csvwrite_with_headers('inclination_rotation_test_aggressive_vicon.csv', out, headers);

out = [data.time, data.velocity, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), rad2deg(roll_ref), rad2deg(pitch_ref), rad2deg(yaw_ref), data.q, data.omega_body, data.omega_ref_body, data.S, data.torque, data.torque_delivered, rad2deg(eul_error_qekf), rad2deg(eul_error_vicon), omega_body_error_qekf, omega_body_error_vicon];
out = Downsample(out, 2); % downsample plot data
headers = {'time', 'velocity_x', 'velocity_y', 'roll', 'pitch', 'yaw', 'roll_ref', 'pitch_ref', 'yaw_ref', 'q0', 'q1', 'q2', 'q3', 'omega_body_x', 'omega_body_y', 'omega_body_z', 'omega_body_ref_x', 'omega_body_ref_y', 'omega_body_ref_z', 's0', 's1', 's2', 'tau0', 'tau1', 'tau2', 'tau0_delivered', 'tau1_delivered', 'tau2_delivered' ...
           'roll_error_qekf', 'pitch_error_qekf', 'yaw_error_qekf', 'roll_error_vicon', 'pitch_error_vicon', 'yaw_error_vicon', ...
           'omega_error_x_qekf', 'omega_error_y_qekf', 'omega_error_z_qekf', ...
           'omega_error_x_vicon', 'omega_error_y_vicon', 'omega_error_z_vicon' ...
};
csvwrite_with_headers('inclination_rotation_test_aggressive_system.csv', out, headers);