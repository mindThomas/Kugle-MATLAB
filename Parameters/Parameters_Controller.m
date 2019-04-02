% Controller parameters
Ts = 1/200; % 200 Hz controller sample rate
Ts_heading = 1/10; % 10 Hz SLAM rate (for heading estimate)

% Sliding Mode controller selection and gains
SlidingManifold = 1; % 0=q_dot inertial,  1=q_dot body,  2=Body angular velocity,  3=Inertial angular velocity
SwitchingLaw = 2; % 1=discontinous,  2=continous
K = diag([6, 6, 6]);
eta = [5; 5; 6];
epsilon = [0.8; 0.8; 0.3];