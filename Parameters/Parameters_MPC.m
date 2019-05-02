%% MPC sample rate
Ts_MPC = 1/10;

%% Horizon length
N = 20;

%% MPC constraints
minVelocity = 0.0; % m/s
maxVelocity = 0.5; % m/s
maxAngle = deg2rad(5); % rad
maxOmegaRef = deg2rad(30); % rad/s
maxdOmegaRef = deg2rad(30); % rad/s^2  (this setting is crucial when torque saturations come into play)

%% Weights
WPathFollow = 200.0;
WVelocity = 20.0;
WSmoothness = 100.0;
WOmega = 100;

% Horizon weight matrix (cost)
Wdiag = [WPathFollow, WPathFollow,  0.1,0.1,  WOmega,WOmega,  99999,WVelocity,  WSmoothness,WSmoothness,  99999.0,99999999.0]; % [ x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error;   domega_ref_x;domega_ref_y;   velocity_slack_variable;proximity_slack_variable ]
Wmat = diag(Wdiag);

% Final state weight matrix (cost)
WNdiag = [WPathFollow, WPathFollow,  0.1,0.1,  0.1,0.1,  99999,WVelocity]; % [ x_err;y_err; q2;q3;  omega_ref_x;omega_ref_y;  velocity_matching; velocity_error ]
WNmat = diag(WNdiag);

%% Window parameters
WindowWidth = 5.0;
WindowHeight = 5.0;
WindowOffset = [0, 0]; % Window offset relative to robot, hence positive X means window is moved in positive X direction in robot frame so robot center will move to a negative X position in the window
pathApproximationOrder = 8;
VelocityDefinedWindow = false; % will replace the parameters above if set to true
WindowOrientationSelection = 0; % 0=inertial frame, 1=robot yaw (heading), 2=velocity direction