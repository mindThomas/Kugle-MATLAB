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


% IMU location
IMU_height = 0.1; % distance in body frame from center of ball to center of IMU
IMU_misalignmentYPR = 0*deg2rad([0, -3, 0]); % yaw, pitch, roll
IMU_misalignment = eul2rotm(IMU_misalignmentYPR, 'ZYX');

% Sensor Noise parameters
EnableNoise = false;
  
sigma2_acc = 1.0e-03 * 0.4;
sigma2_gyro = 1.0e-03 * 0.9;     
sigma2_encoder = 0.5;   
sigma2_heading = (deg2rad(10) / 3) ^ 2;

% Sensor Bias parameters
gyro_bias = [0*-0.025, 0*0.015, 0*-0.008]';  % rad/s

% Plant/Sim Initialization parameters
roll_init_deg = 10;
pitch_init_deg = 0;
yaw_init_deg = 0;
omeg_x_init = 0;
omeg_y_init = 0;
omeg_z_init = 0;
q_init = eul2quat(deg2rad([yaw_init_deg,pitch_init_deg,roll_init_deg]),'ZYX')';
dq_init = 1/2 * Gamma(q_init) * [0;omeg_x_init;omeg_y_init;omeg_z_init];
x_init = [0,0, q_init', 0,0, dq_init']';

% Estimator initialization parameters
est_q_init = eul2quat(deg2rad([yaw_init_deg,pitch_init_deg,roll_init_deg]),'ZYX')';
