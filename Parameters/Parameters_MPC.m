%% MPC sample rate
Ts_MPC = 1/10;

%% Horizon length
N = 20;

%% MPC constraints
minVelocity = 0.0; % m/s
maxVelocity = 3.0; % m/s
maxAngle = deg2rad(10); % rad
maxOmegaRef = deg2rad(30); % rad/s

%% Weights
WPathFollow = 50.0;
WVelocity = 10.0;
WSmoothness = 10.0;

% Horizon weight matrix (cost)
Wdiag = [WPathFollow, WPathFollow,  0.1,0.1,  0.1,0.1,  99999,WVelocity,  WSmoothness,WSmoothness,  99999.0,99999999.0]; % [ x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error;   domega_ref_x;domega_ref_y;   velocity_slack_variable;angle_slack_variable;proximity_slack_variable ]
Wmat = diag(Wdiag);

% Final state weight matrix (cost)
WNdiag = [WPathFollow, WPathFollow,  0.1,0.1,  0.1,0.1,  99999,WVelocity]; % [ x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error ]
WNmat = diag(WNdiag);

%% Window parameters
WindowWidth = 5.0;
WindowHeight = 5.0;
WindowOffset = [0, 0]; % Window offset relative to robot, hence positive X means window is moved in positive X direction in robot frame so robot center will move to a negative X position in the window
WindowOrientation = 0;
pathApproximationOrder = 8;
WindowOrientationSelection = 0; % 0=inertial frame, 1=robot yaw (heading), 2=velocity direction

%% MPC state init
% q_init = [1,0,0,0]';
% xInit = [q_init(2:3)',  2,1,  0,0]';
% uInit = [0,0]';
% zInit = [];
% bValues = [];
% 
% timeVec = (0:N)' * Ts_MPC;
% x_ref = 0;
% y_ref = 0;
% references = [timeVec, repmat([x_ref, y_ref], [N+1,1])];
% 
% 
% x_final = x_ref;
% y_final = y_ref;
% OData_element = [maxAngle, maxOmegaRef, x_final, y_final];
% OData = repmat(OData_element, [N+1,1]);