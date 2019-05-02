% Quaternion math operators
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

beta = 0; % quaternion regularization coefficient = disabled for controllers etc. - only needed in ODE where it is manually set

% IMU location
IMU_height = 0.1; % distance in body frame from center of ball to center of IMU
IMU_misalignmentYPR = 0*deg2rad([0, -3, 0]); % yaw, pitch, roll
IMU_misalignment = eul2rotm(IMU_misalignmentYPR, 'ZYX');

% Sensor Noise parameters
EnableNoise = false;

% Load sensor covariance from estimator parameters
Parameters_Estimators;
  
% Specify noise variance for simulated sensor (can be the same as used in estimator)
sensor_sigma2_acc = 1.0e-03 * 0.4;
sensor_sigma2_gyro = 1.0e-03 * 0.9;     
sensor_sigma2_encoder = 0.5;
sensor_sigma2_heading = (deg2rad(1) / 3) ^ 2;

% Construct simulated sensor covariance matrices
sensor_cov_acc = cov_acc; %eye(3) * sensor_sigma2_acc;
sensor_cov_gyro = cov_gyro; %eye(3) * sensor_sigma2_gyro;

% Sensor Bias parameters
gyro_bias = 0*[-0.025, 0.015, -0.008]';  % rad/s

% Plant/Sim Initialization parameters
roll_init_deg = 0;
pitch_init_deg = 0;
yaw_init_deg = 0;
omeg_x_init = 0;
omeg_y_init = 0;
omeg_z_init = 0;
q_init = eul2quat(deg2rad([yaw_init_deg,pitch_init_deg,roll_init_deg]),'ZYX')';
dq_init = 1/2 * Gamma(q_init) * [0;omeg_x_init;omeg_y_init;omeg_z_init];
x_init = [4,-2, q_init', 0,0, dq_init']';

% Estimator initialization parameters
est_q_init = eul2quat(deg2rad([yaw_init_deg,pitch_init_deg,roll_init_deg]),'ZYX')';


%% Reference Test modes
ReferenceMode = 1; % 0 = constant, 1 = roll sine wave, 2 = roll chirp

% Constant reference mode
q_ref = eul2quat(deg2rad([0,0,0]), 'ZYX')';
omega_b_ref = [0,0,0]';

% Roll sine wave
f_sine = 0.5; % hertz
roll_sine_amplitude_deg = 3;

% Roll chip
f0_chirp = 0.5; % hertz
rate_chirp = 0.05; % hertz per second
chirp_amplitude_deg = 2;


%% MPC simulation parameters
MPC_TrajectoryType = 0; % 0=oval,  1=figure eight
MPC_EnableStaticObstacles = true;
MPC_RandomObstacles = 0;
MPC_DesiredVelocity = 1.0; % m/s
MPC_HeadingAngularVelocity = 0.0; % rad/s

%% MPC state init
% Load MPC parameters
Parameters_MPC;

% x = [q2,q3, x,y, dx,dy, s,ds, omega_ref_x,omega_ref_y]
MPC_x0 = [0,0,  0,0,  0.001,0.001,  0,0.001,  0,0]; % init with a small velocity - otherwise problem is not feasible ???
MPC_u0 = [0,0,0,0,0];

%% Automatic parameters - do not touch!
% Used to generate multivariate noise as: R*randn(3,1)
R_cov_acc = chol(sensor_cov_acc, 'lower');
R_cov_gyro = chol(sensor_cov_gyro, 'lower');