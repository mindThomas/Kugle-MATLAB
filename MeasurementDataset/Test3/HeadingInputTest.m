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
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\Estimators\QEKF');
Constants_Kugle;
Parameters_General;
Parameters_Estimators;

DumpFolder = [pwd '\'];
data = LoadDump(DumpFolder, '');
vicon = LoadVicon(DumpFolder, '');
[data, vicon] = DumpViconTimeSynchronization(data, vicon);
[data, vicon] = TrimSynced(data, vicon, 20, 90+50); % 92 sync, 90 length  (yaw_bias)
%[data, vicon] = TrimSynced(data, vicon, 92, 90); % 92 sync, 90 length  (heading_input)

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


%% Run QEKF on measurements
dt = Ts;
YawBiasEstimationEnabled = true; % in C++ code this is only set to true when a heading input is available

X = [data.q(1,:),  data.omega_body(1,:),  data.gyro_bias(1,:)]';
P = diag(QEKF_P_init_diagonal);    
%P = diag([1E-5, 1E-5, 1E-5, 1E-7,  1E-7, 1E-7, 1E-7, 1E-7,   1E-5, 1E-5]);    
P(1:4,1:4) = reshape(data.Cov_q(1,:), [4,4])'; % load covariance from measurements (use same init state)
Cov_dq_init = reshape(data.Cov_dq(1,:), [4,4])';
T = 2*devec*Phi(X(1:4))';
Cov_omega_body_init = T * Cov_dq_init * T';
P(5:7,5:7) = Cov_omega_body_init;
P(8:10,8:10) = reshape(data.Cov_bias(1,:), [3,3])';

q_est = [];
dq_est = [];
omega_est = [];
gyro_bias_est = [];
omega_body_est = [];

q_est = [q_est; X(1:4)'];
dq_est = [dq_est; X(5:8)'];
omega_body_est = [omega_body_est; X(5:7)'];
gyro_bias_est = [gyro_bias_est; X(8:10)'];
accelerometer_angle = [0, 0];
%omega_body_est = [omega_body_est; (2*devec*Phi(X(1:4))'*X(5:8))'];
for (i = 2:length(data.time))
    HeadingValid = false;
    Heading = 0;    
    if (mod(i, ceil(0.05 / dt)) == 0 && data.time(i) < 40)                
        [val, idx] = min(abs(vicon.time - data.time(i))); % find matchin vicon sample
        Heading = vicon.yaw(idx);
        %HeadingValid = true;
    end    
    
    % Remember to call the function with "single(...)" as inputs, since the estimators are implemented with "float"
    [X, P] = QEKF(single(X), single(P), single(data.gyroscope(i,:)'), single(data.accelerometer(i,:)'), single(Heading), HeadingValid, single(dt), SensorDrivenQEKF, EstimateBias, YawBiasEstimationEnabled, NormalizeAccelerometer, single(cov_gyro), single(cov_acc), single(GyroscopeTrustFactor), single(sigma2_omega), single(sigma2_heading), single(sigma2_bias), AccelerometerVibration_DetectionEnabled, AccelerometerVibration_NormLPFtau, single(AccelerometerVibration_CovarianceVaryFactor), single(AccelerometerVibration_MaxVaryFactor), single(g));
    q_est = [q_est; X(1:4)'];
    %dq_est = [dq_est; X(5:8)']; omega_body_est = [omega_body_est; (2*devec*Phi(X(1:4))'*X(5:8))'];
    omega_body_est = [omega_body_est; X(5:7)'];
    gyro_bias_est = [gyro_bias_est; X(8:10)'];    
    %omega_body_est = [omega_body_est; (2*devec*Phi(X(1:4))'*X(5:8))'];    
           
    mu = 0.0001;
    accelerometer_roll = atan2(data.raw_accelerometer(i,2), sqrt(data.raw_accelerometer(i,3)^2 + mu*data.raw_accelerometer(i,1)^2));
    accelerometer_pitch = atan2(-data.raw_accelerometer(i,1), sqrt(data.raw_accelerometer(i,2)^2 + data.raw_accelerometer(i,3)^2));                   
    accelerometer_angle = [accelerometer_angle; [accelerometer_roll, accelerometer_pitch]];    
end
eul_est = quat2eul(q_est, 'ZYX');

%% Orientation
fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation');
ax1 = subplot(3,1,1); plot(vicon.time, rad2deg(vicon.roll), data.time, rad2deg(data.roll), data.time, rad2deg(eul_est(:,3))); ylabel('Angle [deg]'); title('Roll'); legend('Vicon', 'QEKF', 'QEKF Post-processed');
ax2 = subplot(3,1,2); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(eul_est(:,2))); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'QEKF', 'QEKF Post-processed');
ax3 = subplot(3,1,3); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.yaw),data.time, rad2deg(eul_est(:,1))); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'QEKF', 'QEKF Post-processed');
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


%% Orientation error
mti_eul = [data.mti_roll, data.mti_pitch, data.mti_yaw];
qekf_eul = [data.roll, data.pitch, data.yaw];
qekf2_eul = eul_est(:,[3,2,1]);
vicon_eul = [vicon.roll, vicon.pitch, vicon.yaw];
vicon_eul2 = interp1(vicon.time, vicon_eul, data.time, 'linear', 'extrap');
eul_error_mti = qekf_eul - mti_eul;
eul_error_vicon = qekf_eul - vicon_eul2;
eul_error_with_heading_mti = qekf2_eul - mti_eul;
eul_error_with_heading_vicon = qekf2_eul - vicon_eul2;
fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Orientation estimation error');
ax1 = subplot(3,1,1); plot(data.time, rad2deg(eul_error_mti(:,1)), data.time, rad2deg(eul_error_vicon(:,1)), data.time, rad2deg(eul_error_with_heading_mti(:,1)), data.time, rad2deg(eul_error_with_heading_vicon(:,1))); ylabel('Angle error [deg]'); title('Roll'); legend('QEKF - MTI', 'QEKF - Vicon', 'QEKF w/heading - MTI', 'QEKF w/heading - Vicon');
ax2 = subplot(3,1,2); plot(data.time, rad2deg(eul_error_mti(:,2)), data.time, rad2deg(eul_error_vicon(:,2)), data.time, rad2deg(eul_error_with_heading_mti(:,2)), data.time, rad2deg(eul_error_with_heading_vicon(:,2))); ylabel('Angle error [deg]'); title('Pitch'); legend('QEKF - MTI', 'QEKF - Vicon', 'QEKF w/heading - MTI', 'QEKF w/heading - Vicon');
ax3 = subplot(3,1,3); plot(data.time, rad2deg(eul_error_mti(:,3)), data.time, rad2deg(eul_error_vicon(:,3)), data.time, rad2deg(eul_error_with_heading_mti(:,3)), data.time, rad2deg(eul_error_with_heading_vicon(:,3))); ylabel('Angle error [deg]'); title('Yaw'); legend('QEKF - MTI', 'QEKF - Vicon', 'QEKF w/heading - MTI', 'QEKF w/heading - Vicon');
linkaxes([ax1,ax2,ax3],'x');
xlabel('Time [s]');


%
fig = figure(5); set(fig, 'NumberTitle', 'off', 'Name', 'Yaw Bias calibration');
ax1 = subplot(4,1,1); plot(vicon.time, rad2deg(vicon.pitch), data.time, rad2deg(data.pitch), data.time, rad2deg(eul_est(:,2))); ylabel('Angle [deg]'); title('Pitch'); legend('Vicon', 'QEKF', 'QEKF w/heading');
ax2 = subplot(4,1,2); plot(vicon.time, rad2deg(vicon.yaw), data.time, rad2deg(data.yaw), data.time, rad2deg(eul_est(:,1))); ylabel('Angle [deg]'); title('Yaw'); legend('Vicon', 'QEKF', 'QEKF w/heading');
ax3 = subplot(4,1,3); plot(data.time, gyro_bias_est(:,1), data.time, gyro_bias_est(:,2), data.time, gyro_bias_est(:,3)); ylabel('Angular velocity bias [rad/s]'); title('Gyro bias'); legend('X','Y','Z');
ax4 = subplot(4,1,4); plot(data.time, rad2deg(eul_error_vicon(:,3)), data.time, rad2deg(eul_error_with_heading_vicon(:,3))); ylabel('Angle error [deg]'); title('Yaw estimation error'); legend('QEKF - Vicon', 'QEKF w/heading - Vicon');
linkaxes([ax1,ax2,ax3,ax4],'x');
xlabel('Time [s]');


%% Yaw bias output - TrimSynced(data, vicon, 20, 90+50); % 92 sync, 90 length
% out = [data.time, rad2deg(eul_est(:,3)), rad2deg(eul_est(:,2)), rad2deg(eul_est(:,1)), gyro_bias_est, rad2deg(eul_error_with_heading_vicon(:,3))];
% out = Downsample(out, 10); % downsample plot data
% headers = {'time', 'roll', 'pitch', 'yaw', 'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z', 'yaw_estimation_error'};
% csvwrite_with_headers('yaw_bias_system.csv', out, headers);
% 
% out = [vicon.time, rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw)];
% out = Downsample(out, 10); % downsample plot data
% headers = {'time', 'roll', 'pitch', 'yaw'};
% csvwrite_with_headers('yaw_bias_vicon.csv', out, headers);

%% Heading input data output
out = [data.time, rad2deg(data.roll), rad2deg(data.pitch), rad2deg(data.yaw), rad2deg(data.mti_roll), rad2deg(data.mti_pitch), rad2deg(data.mti_yaw), rad2deg(eul_est(:,3)), rad2deg(eul_est(:,2)), rad2deg(eul_est(:,1)), rad2deg(eul_error_mti(:,3)), rad2deg(eul_error_with_heading_mti(:,3)), rad2deg(eul_error_vicon(:,3)), rad2deg(eul_error_with_heading_vicon(:,3)), gyro_bias_est(:,3)];
out = Downsample(out, 10); % downsample plot data
headers = {'time', ...
    'qekf_roll', 'qekf_pitch', 'qekf_yaw', ...
    'mti_roll', 'mti_pitch', 'mti_yaw', ...
    'qekf_with_heading_roll', 'qekf_with_heading_pitch', 'qekf_with_heading_yaw', ...
    'yaw_error_mti', 'yaw_error_with_heading_mti', ...    
    'yaw_error_vicon', 'yaw_error_with_heading_vicon', ...
    'yaw_gyro_bias'
};
csvwrite_with_headers('heading_input_system.csv', out, headers);

out = [vicon.time, rad2deg(vicon.roll), rad2deg(vicon.pitch), rad2deg(vicon.yaw)];
out = Downsample(out, 10); % downsample plot data
headers = {'time', 'roll', 'pitch', 'yaw'};
csvwrite_with_headers('heading_input_vicon.csv', out, headers);