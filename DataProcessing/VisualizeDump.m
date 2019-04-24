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

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, '../Model/generated'));
addpath(fullfile(scriptDir, '../Parameters'));
Constants_Kugle;

DumpFolder = '~/kugle_dump/';
data = LoadDump(DumpFolder, '');

%close all;

PlotRawSensors = true;
PlotRawSensorsComparison = false;
PlotAccelerometerNorm = false;
PlotBasicSensorAngle = false;
PlotQuaternionEstimate = true;
PlotQuaternionError = true;
PlotEulerAnglesEstimate = true;
PlotQuaternionDerivativeEstimate = false;
PlotAngularVelocityEstimate = true;
PlotPositionAndVelocityEstimate = true;
PlotMotorVelocityComparison = false;
PlotWheelSlipDetection = true;
PlotCOM = false;
PlotMotorTorque = true;
PlotSlidingManifold = true;
PlotMotorCurrents = false;
PlotVelocityControllerInfo = true;

%data.accelerometer = (R*data.accelerometer')';
%data.accelerometer = (data.raw_accelerometer - acc_bias) .* acc_scale;

if (PlotRawSensors)
    fig = figure(1); set(fig, 'NumberTitle', 'off', 'Name', 'Raw sensors');
    ax1 = subplot(3,1,1); plot(data.time, data.accelerometer(:,1), data.time, data.accelerometer(:,2), data.time, data.accelerometer(:,3)); legend('X', 'Y', 'Z'); title('Accelerometer'); ylabel('m/s^2');
    ax2 = subplot(3,1,2); plot(data.time, data.gyroscope(:,1), data.time, data.gyroscope(:,2), data.time, data.gyroscope(:,3)); legend('X', 'Y', 'Z'); title('Gyroscope'); ylabel('rad/s');
    ax3 = subplot(3,1,3); plot(data.time, data.magnetometer(:,1), data.time, data.magnetometer(:,2), data.time, data.magnetometer(:,3)); legend('X', 'Y', 'Z'); title('Magnetometer'); ; ylabel('mT');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');
end

if (PlotRawSensorsComparison)
    fig = figure(10); set(fig, 'NumberTitle', 'off', 'Name', 'Raw sensors comparison');
    ax11 = subplot(3,2,1); plot(data.time, data.accelerometer(:,1), data.time, data.raw_accelerometer(:,1), data.time, data.mti_accelerometer(:,1)); title('Accelerometer X'); ylabel('m/s^2'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200');
    ax12 = subplot(3,2,2); plot(data.time, data.gyroscope(:,1), data.time, data.raw_gyroscope(:,1), data.time, data.mti_gyroscope(:,1)); title('Gyroscope X'); ylabel('rad/s'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200'); ylim([-0.2, 0.2]);
    ax21 = subplot(3,2,3); plot(data.time, data.accelerometer(:,2), data.time, data.raw_accelerometer(:,2), data.time, data.mti_accelerometer(:,2)); title('Accelerometer Y'); ylabel('m/s^2'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200');
    ax22 = subplot(3,2,4); plot(data.time, data.gyroscope(:,2), data.time, data.raw_gyroscope(:,2), data.time, data.mti_gyroscope(:,2)); title('Gyroscope Y'); ylabel('rad/s'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200'); ylim([-0.2, 0.2]);
    ax31 = subplot(3,2,5); plot(data.time, data.accelerometer(:,3), data.time, data.raw_accelerometer(:,3), data.time, data.mti_accelerometer(:,3)); title('Accelerometer Z'); ylabel('m/s^2'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200');
    ax32 = subplot(3,2,6); plot(data.time, data.gyroscope(:,3), data.time, data.raw_gyroscope(:,3), data.time, data.mti_gyroscope(:,3)); title('Gyroscope Z'); ylabel('rad/s'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200'); ylim([-0.2, 0.2]);
    linkaxes([ax11,ax21,ax31],'x');
    linkaxes([ax12,ax22,ax32],'x');
    xlabel('Time [s]');
end

if (PlotAccelerometerNorm)   
    fig = figure(11); set(fig, 'NumberTitle', 'off', 'Name', 'Accelerometer norm');
    plot(data.time, sqrt(sum(data.accelerometer.^2,2)), data.time, sqrt(sum(data.raw_accelerometer.^2,2)), data.time, sqrt(sum(data.mti_accelerometer.^2,2))); title('Accelerometer Norm'); ylabel('m/s^2'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200');   
    xlabel('Time [s]');
end

if (PlotBasicSensorAngle)
    accelerometer_angle = [];
    for (i = 1:length(data.time))
        roll = atan2(data.accelerometer(i,2), data.accelerometer(i,3));
        pitch = atan2(-data.accelerometer(i,1), data.accelerometer(i,3));
        accelerometer_angle = [accelerometer_angle; [roll, pitch]];
    end  

    fig = figure(12); set(fig, 'NumberTitle', 'off', 'Name', 'Basic Sensor angle');
    ax1 = subplot(2,1,1);
    plot(data.time, rad2deg(accelerometer_angle(:,1)), data.time, rad2deg(data.roll), data.time, rad2deg(data.mti_roll)); title('Roll'); ylabel('deg'); legend('Accelerometer', 'Estimate', 'MTI');   
    ax2 = subplot(2,1,2);
    plot(data.time, rad2deg(accelerometer_angle(:,2)), data.time, rad2deg(data.pitch), data.time, rad2deg(data.mti_pitch)); title('Pitch'); ylabel('deg'); legend('Accelerometer', 'Estimate', 'MTI');   
    linkaxes([ax1,ax2],'x');
    xlabel('Time [s]');
end

%%
if (PlotQuaternionEstimate)
    fig = figure(2); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion estimate');
    ax1 = subplot(4,1,1); plot(data.time, data.q(:,1), data.time, data.q_ref(:,1), data.time, data.mti_q(:,1)); title('q.w'); legend('Estimate', 'Reference', 'MTI');
    ax2 = subplot(4,1,2); plot(data.time, data.q(:,2), data.time, data.q_ref(:,2), data.time, data.mti_q(:,2)); title('q.x'); legend('Estimate', 'Reference', 'MTI');
    ax3 = subplot(4,1,3); plot(data.time, data.q(:,3), data.time, data.q_ref(:,3), data.time, data.mti_q(:,3)); title('q.y'); legend('Estimate', 'Reference', 'MTI');
    ax4 = subplot(4,1,4); plot(data.time, data.q(:,4), data.time, data.q_ref(:,4), data.time, data.mti_q(:,4)); title('q.z'); legend('Estimate', 'Reference', 'MTI');
    linkaxes([ax1,ax2,ax3,ax4],'x');
    xlabel('Time [s]');    
end

if (PlotEulerAnglesEstimate)
    fig = figure(3); set(fig, 'NumberTitle', 'off', 'Name', 'Euler angles estimate');
    ax1 = subplot(3,1,1); plot(data.time, rad2deg(data.roll), data.time, rad2deg(data.roll_ref), data.time, rad2deg(data.mti_roll)); title('Roll');  legend('Estimate', 'Reference', 'MTI'); ylabel('deg');
    ax2 = subplot(3,1,2); plot(data.time, rad2deg(data.pitch), data.time, rad2deg(data.pitch_ref), data.time, rad2deg(data.mti_pitch)); title('Pitch');  legend('Estimate', 'Reference', 'MTI'); ylabel('deg');
    ax3 = subplot(3,1,3); plot(data.time, rad2deg(data.yaw), data.time, rad2deg(data.yaw_ref), data.time, rad2deg(data.mti_yaw), data.time, rad2deg(data.heading)); title('Yaw');  legend('Estimate', 'Reference', 'MTI', 'Heading'); ylabel('deg');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');   
end

if (PlotQuaternionDerivativeEstimate)
    fig = figure(4); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion derivative estimate');
    ax1 = subplot(4,1,1); plot(data.time, data.dq(:,1), data.time, data.mti_dq(:,1)); title('dq.w'); legend('Estimate', 'MTI');
    ax2 = subplot(4,1,2); plot(data.time, data.dq(:,2), data.time, data.mti_dq(:,2)); title('dq.x'); legend('Estimate', 'MTI');
    ax3 = subplot(4,1,3); plot(data.time, data.dq(:,3), data.time, data.mti_dq(:,3)); title('dq.y'); legend('Estimate', 'MTI');
    ax4 = subplot(4,1,4); plot(data.time, data.dq(:,4), data.time, data.mti_dq(:,4)); title('dq.z'); legend('Estimate', 'MTI');
    linkaxes([ax1,ax2,ax3,ax4],'x');
    xlabel('Time [s]');    
end

if (PlotAngularVelocityEstimate)
    fig = figure(5); set(fig, 'NumberTitle', 'off', 'Name', 'Angular velocity estimate');
    ax1 = subplot(3,1,1); plot(data.time, data.gyroscope(:,1), data.time, data.omega_body(:,1), data.time, data.mti_omega_body(:,1), data.time, data.omega_ref_body(:,1)); title('omega.x'); legend('Gyroscope', 'Estimate', 'MTI', 'Reference'); ylabel('rad/s');
    ax2 = subplot(3,1,2); plot(data.time, data.gyroscope(:,2), data.time, data.omega_body(:,2), data.time, data.mti_omega_body(:,2), data.time, data.omega_ref_body(:,2)); title('omega.y'); legend('Gyroscope', 'Estimate', 'MTI', 'Reference'); ylabel('rad/s');
    ax3 = subplot(3,1,3); plot(data.time, data.gyroscope(:,3), data.time, data.omega_body(:,3), data.time, data.mti_omega_body(:,3), data.time, data.omega_ref_body(:,3)); title('omega.z'); legend('Gyroscope', 'Estimate', 'MTI', 'Reference'); ylabel('rad/s');
    linkaxes([ax1,ax2,ax3],'x');
end


% Compute motor velocities by differentiation
dpsi = [zeros(1,3); diff(data.encoder_angle) ./ diff(data.time)];
dpsi(1,:) = dpsi(2,:);
kinematics_velocity = ForwardKinematics(dpsi(:,1)',dpsi(:,2)',dpsi(:,3)', data.dq(:,1)',data.dq(:,2)',data.dq(:,3)',data.dq(:,4)',  data.q(:,1)',data.q(:,2)',data.q(:,3)',data.q(:,4)',  rk,rw)';
kinematics_CoR_velocity = [];
dpsi_inverse_kinematics = [];
ball_velocity = [];
for (i = 1:length(data.time))
    vel_CoR_to_ball_correction = devec * (Phi(data.dq(i,:)')*Gamma(data.q(i,:)')' + Phi(data.q(i,:)')*Gamma(data.dq(i,:)')') * [0,0,0,CoR]';
    kinematics_CoR_velocity = [kinematics_CoR_velocity; kinematics_velocity(i,:) + vel_CoR_to_ball_correction(1:2)'];    
    dpsi_IK = InverseKinematics(data.dq(i,1), data.dq(i,2), data.dq(i,3), data.dq(i,4), data.velocity(i,1), data.velocity(i,2), data.q(i,1), data.q(i,2), data.q(i,3), data.q(i,4), rk, rw);
    dpsi_inverse_kinematics = [dpsi_inverse_kinematics; dpsi_IK'];
end  
    
if (PlotPositionAndVelocityEstimate)
    fig = figure(6); set(fig, 'NumberTitle', 'off', 'Name', 'Position & velocity estimate');
    ax1 = subplot(3,1,1); plot(data.time, data.position(:,1), data.time, data.position(:,2)); legend('X', 'Y'); title('Position'); ylabel('m');
    ax2 = subplot(3,1,2); plot(data.time, data.velocity(:,1), data.time, kinematics_velocity(:,1), data.time, kinematics_CoR_velocity(:,1), data.time, data.velocity_ref_inertial(:,1)); legend('Estimate', 'Kinematics Ball', 'Kinematics CoR', 'Reference'); title('Velocity X'); ylabel('m/s');
    ax3 = subplot(3,1,3); plot(data.time, data.velocity(:,2), data.time, kinematics_velocity(:,2), data.time, kinematics_CoR_velocity(:,2), data.time, data.velocity_ref_inertial(:,2)); legend('Estimate', 'Kinematics Ball', 'Kinematics CoR', 'Reference'); title('Velocity Y'); ylabel('m/s');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');    
end

if (PlotMotorVelocityComparison)
    fig = figure(7); set(fig, 'NumberTitle', 'off', 'Name', 'Motor velocity comparison');
    ax1 = subplot(3,1,1); plot(data.time, dpsi(:,1), data.time, dpsi_inverse_kinematics(:,1)); legend('Encoder', 'Inverse Kinematics'); title('Motor 1'); ylabel('rad/s');
    ax2 = subplot(3,1,2); plot(data.time, dpsi(:,2), data.time, dpsi_inverse_kinematics(:,2)); legend('Encoder', 'Inverse Kinematics'); title('Motor 2'); ylabel('rad/s');
    ax3 = subplot(3,1,3); plot(data.time, dpsi(:,3), data.time, dpsi_inverse_kinematics(:,3)); legend('Encoder', 'Inverse Kinematics'); title('Motor 3'); ylabel('rad/s');    
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');    
end    
    
if (PlotWheelSlipDetection)    
    ddpsi = diff(dpsi) ./ diff(data.time);
    fig = figure(27); set(fig, 'NumberTitle', 'off', 'Name', 'Motor acceleration');
    ax1 = subplot(3,1,1); plot(data.time(2:end), ddpsi(:,1)); title('Motor 1'); ylabel('rad/s^2');
    ax2 = subplot(3,1,2); plot(data.time(2:end), ddpsi(:,2)); title('Motor 2'); ylabel('rad/s^2');
    ax3 = subplot(3,1,3); plot(data.time(2:end), ddpsi(:,3)); title('Motor 3'); ylabel('rad/s^2');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');       
    
    SlipDetected = abs(ddpsi) > 500;
    fig = figure(28); set(fig, 'NumberTitle', 'off', 'Name', 'Wheel slip detection');
    ax1 = subplot(4,1,1); stairs(data.time, data.wheel_slip_detected(:,1)); hold on; stairs(data.time(2:end), SlipDetected(:,1)); plot(data.time, data.WheelSlipRampGain); hold off; legend('Embedded', 'MATLAB', 'Ramp gain'); title('Motor 1'); ylabel('Slip detected'); hold off; ylim([0, 1.2]);
    ax2 = subplot(4,1,2); stairs(data.time, data.wheel_slip_detected(:,2)); hold on; stairs(data.time(2:end), SlipDetected(:,2)); plot(data.time, data.WheelSlipRampGain);  hold off; legend('Embedded', 'MATLAB', 'Ramp gain'); title('Motor 2'); ylabel('Slip detected'); hold off; ylim([0, 1.2]);
    ax3 = subplot(4,1,3); stairs(data.time, data.wheel_slip_detected(:,3)); hold on; stairs(data.time(2:end), SlipDetected(:,3)); plot(data.time, data.WheelSlipRampGain);  hold off; legend('Embedded', 'MATLAB', 'Ramp gain'); title('Motor 3'); ylabel('Slip detected'); hold off; ylim([0, 1.2]);
    ax4 = subplot(4,1,4);  plot(data.time, sqrt(sum(data.accelerometer.^2,2)), data.time, sqrt(sum(data.raw_accelerometer.^2,2)), data.time, sqrt(sum(data.mti_accelerometer.^2,2))); title('Accelerometer Norm'); ylabel('m/s^2'); legend('MPU9250 calibrated', 'MPU9250', 'MTI200');   
    linkaxes([ax1,ax2,ax3,ax4],'x');   
    xlabel('Time [s]');      
end

if (PlotCOM)
    fig = figure(8); set(fig, 'NumberTitle', 'off', 'Name', 'COM');
    ax1 = subplot(3,1,1); plot(data.time, data.COM(:,1)); ylabel('m');
    ax2 = subplot(3,1,2); plot(data.time, data.COM(:,2)); ylabel('m');
    ax3 = subplot(3,1,3); plot(data.time, data.COM(:,3)); ylabel('m');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');   
end


if (PlotQuaternionError)
    q_err = [];
    q_ref_corrected = [];
    for (i = 1:length(data.time))
        q_err_ = Phi(data.q_ref(i,:)')' * data.q(i,:)';
        if (q_err_(1) < 0)
            q_err_ = -q_err_;
        end
        q_err = [q_err; q_err_'];
        
        q_ref_ = data.q_ref(i,:)';
        q_ref_corrected_ = Gamma(q_err_)' * data.q(i,:)';
        q_ref_corrected = [q_ref_corrected; q_ref_corrected_'];
    end
    fig = figure(9); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion error');
    ax1 = subplot(4,1,1); plot(data.time, q_err(:,1)); title('q.w');
    ax2 = subplot(4,1,2); plot(data.time, q_err(:,2)); title('q.x');
    ax3 = subplot(4,1,3); plot(data.time, q_err(:,3)); title('q.y');
    ax4 = subplot(4,1,4); plot(data.time, q_err(:,4)); title('q.z');
    linkaxes([ax1,ax2,ax3,ax4],'x');
    xlabel('Time [s]');
    
    fig = figure(10); set(fig, 'NumberTitle', 'off', 'Name', 'Quaternion reference');
    ax1 = subplot(4,1,1); plot(data.time, data.q(:,1), data.time, data.q_ref(:,1), data.time, q_ref_corrected(:,1)); title('q.w'); legend('Estimate', 'Reference', 'Reference corrected');
    ax2 = subplot(4,1,2); plot(data.time, data.q(:,2), data.time, data.q_ref(:,2), data.time, q_ref_corrected(:,2)); title('q.x'); legend('Estimate', 'Reference', 'Reference corrected');
    ax3 = subplot(4,1,3); plot(data.time, data.q(:,3), data.time, data.q_ref(:,3), data.time, q_ref_corrected(:,3)); title('q.y'); legend('Estimate', 'Reference', 'Reference corrected');
    ax4 = subplot(4,1,4); plot(data.time, data.q(:,4), data.time, data.q_ref(:,4), data.time, q_ref_corrected(:,4)); title('q.z'); legend('Estimate', 'Reference', 'Reference corrected');
    linkaxes([ax1,ax2,ax3,ax4],'x');
    xlabel('Time [s]');    
end


%%
if (PlotMotorTorque)
    fig = figure(17); set(fig, 'NumberTitle', 'off', 'Name', 'Motor torque');
    ax1 = subplot(3,1,1); plot(data.time, data.torque(:,1), data.time, data.torque_delivered(:,1)); title('Motor 1'); legend('Setpoint', 'Delivered'); ylabel('Torque [Nm]');
    ax2 = subplot(3,1,2); plot(data.time, data.torque(:,2), data.time, data.torque_delivered(:,2)); title('Motor 2'); legend('Setpoint', 'Delivered'); ylabel('Torque [Nm]');
    ax3 = subplot(3,1,3); plot(data.time, data.torque(:,3), data.time, data.torque_delivered(:,3)); title('Motor 3'); legend('Setpoint', 'Delivered'); ylabel('Torque [Nm]');
    linkaxes([ax1,ax2,ax3],'x');
end

if (PlotSlidingManifold)
    fig = figure(18); set(fig, 'NumberTitle', 'off', 'Name', 'Sliding manifold');
    ax1 = subplot(3,1,1); plot(data.time, data.S(:,1)); title('Sx');
    ax2 = subplot(3,1,2); plot(data.time, data.S(:,2)); title('Sy');
    ax3 = subplot(3,1,3); plot(data.time, data.S(:,3)); title('Sz');
    linkaxes([ax1,ax2,ax3],'x');
end

if (PlotMotorCurrents)
    fig = figure(19); set(fig, 'NumberTitle', 'off', 'Name', 'Motor currents');
    current = ((data.torque_delivered / (13/3)) / 30.5e-3);
    ax1 = subplot(3,1,1); plot(data.time, current(:,1)); title('Motor 1'); legend('Delivered'); ylabel('Current [A]');
    ax2 = subplot(3,1,2); plot(data.time, current(:,2)); title('Motor 2'); legend('Delivered'); ylabel('Current [A]');
    ax3 = subplot(3,1,3); plot(data.time, current(:,3)); title('Motor 3'); legend('Delivered'); ylabel('Current [A]');
    linkaxes([ax1,ax2,ax3],'x');
end

if (PlotVelocityControllerInfo)
    fig = figure(20); set(fig, 'NumberTitle', 'off', 'Name', 'Velocity Controller Info');
    integral_ypr = quat2eul(data.q_tilt_integral, 'ZYX');
    ax1 = subplot(3,1,1); plot(data.time, rad2deg(data.roll), data.time, rad2deg(data.roll_ref), data.time, rad2deg(integral_ypr(:,3))); title('Roll');  legend('Estimate', 'Reference', 'Integral'); ylabel('deg');
    ax2 = subplot(3,1,2); plot(data.time, rad2deg(data.pitch), data.time, rad2deg(data.pitch_ref), data.time, rad2deg(integral_ypr(:,2))); title('Pitch');  legend('Estimate', 'Reference', 'Integral'); ylabel('deg');
    ax3 = subplot(3,1,3); plot(data.time, rad2deg(data.yaw), data.time, rad2deg(data.yaw_ref), data.time, rad2deg(integral_ypr(:,1))); title('Yaw');  legend('Estimate', 'Reference', 'Integral'); ylabel('deg');
    linkaxes([ax1,ax2,ax3],'x');
    xlabel('Time [s]');
end
