%% IMU settings
UseXsensIMU = false;

% MPU-9250 with internal LPF off
cov_gyro_mpu = [0.000496942754176,   0.000020107488666,   0.000003512802761;
                0.000020107488666,   0.000174919150389,  -0.000025989121108;
				0.000003512802761,  -0.000025989121108,   0.001396990425282];
cov_acc_mpu = [0.394508786413515E-03,   0.000603648730082E-03,  -0.023365964974750E-03;
               0.000603648730082E-03,   0.392207026988784E-03,  -0.003560872957017E-03;
              -0.023365964974750E-03,  -0.003560872957017E-03,   0.994086382318077E-03];

% MTI-200 with sample rate configured to 400 Hz but samples are only captured at 200 Hz (hence downsampled)
% Covariance has been estimated with samples sampled at 200 Hz
cov_gyro_mti = [0.767838569550055E-05,  -0.001550044758582E-05,   0.006121206040185E-05;
               -0.001550044758582E-05,   0.744576444194164E-05,  -0.003516953093983E-05;
                0.006121206040185E-05,  -0.003516953093983E-05,   0.795201715550991E-05];			
cov_acc_mti =  [0.143168418480867E-03,   0.025720201380381E-03,   0.013511303535437E-03;
				0.025720201380381E-03,   0.132103665088956E-03,   0.025679048752216E-03;
				0.013511303535437E-03,   0.025679048752216E-03,   0.141723092884984E-03];

%% Orientation estimation settings
UseMadgwick = false;
            
%% QEKF (Quaternion estimator) parameters
UseHeadingEstimateFromXsensIMU = true; % if the Xsens Quaternion estimate is not used, setting this flag to true will input the Xsens heading into the QEKF as a heading sensor input - Do not use this if a SLAM-based heading input is to be used
SensorDrivenQEKF = false;
NormalizeAccelerometer = true; % in C++ code this is hardcoded to true
EstimateBias = true; % estimate gyroscope bias as part of QEKF - it is not recommended to enable this when using the Xsens IMU since it has internal bias correction
sigma2_bias = 1E-10; % for MPU9250 use 1E-6 or 1E-7 works well, for MTI-200 use 1E-9 or disable bias estimation completely!
sigma2_omega = 10^(-6.5); % 10^(-6.5) = 3.16228e-07  --  for MPU9250 use 1E-5, for MTI-200 use 1E-2 due to the smaller gyroscope noise magnitude
sigma2_heading = (deg2rad(1) / 3) ^ 2; % 3*sigma == 1 degree = 3.3846e-05
GyroscopeTrustFactor = 2.0; % the higher value the more trust is put into the gyroscope measurements by increasing the accelerometer covariance
AccelerometerVibration_DetectionEnabled = false;
AccelerometerVibration_NormLPFtau = 0.5; % seconds
AccelerometerVibration_CovarianceVaryFactor = 2.0; % vary/scale the accelerometer covariance depending on exaggerated accelerations (above 'AccelerometerVibration_DetectionAmount') based on VaryFactor=exp(AccelerometerCovarianceVaryFactor*norm_difference)
AccelerometerVibration_MaxVaryFactor = 10000; % vary/scale the accelerometer covariance with maximum this value
% X = {q0, q1, q2, q3,   omega_b_x, omega_b_y, omega_b_z,   gyro_bias_x, gyro_bias_y, gyro_bias_z}
QEKF_P_init_diagonal = [1E-7, 1E-7, 1E-7, 1E-9,   1E-7, 1E-7, 1E-9,   1E-8, 1E-8, 1E-8]; % initialize q3 variance lower than others, since yaw can not be estimated so we are more certain on the initial value to let gyro integration (dead-reckoning) dominate the "yaw" estimate
Ts_heading = 1/10; % 10 Hz SLAM rate (for heading estimate)

%% VEKF (Velocity estimator) parameters
UseVelocityEstimator = true;
UseQdotInVelocityEstimator = true;
eta_encoder = 1.0; % tuning factor for encoder measurement trust - decrease value to trust the encoder measurements more
eta_accelerometer = 5;  % tuning factor for accelerometer trust - increase value to put less trust in accelerometer measurements
var_acc_bias = 1E-9;
var_acceleration = 1E-5; % smoothing factor of velocity estimate in terms of process variance on acceleration
% X = {dx, dy, ddx, ddy, acc_bias_x, acc_bias_y, acc_bias_z}
VelocityEstimator_P_init_diagonal = [1E-1,1E-1, 1E-2,1E-2, 1E-9,1E-9,1E-9]; 

%% Automatic parameters - do not touch!
if (UseXsensIMU)
    cov_gyro = cov_gyro_mti;
    cov_acc = cov_acc_mti;
else
    cov_gyro = cov_gyro_mpu;
    cov_acc = cov_acc_mpu;
end