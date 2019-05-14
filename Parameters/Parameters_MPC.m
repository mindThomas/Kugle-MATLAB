%% MPC sample rate
Ts_MPC = 1/10;

%% Horizon length
N = 20;

%% MPC constraints
minVelocity = 0.0; % m/s
maxVelocity = 10; % m/s
maxAngle = deg2rad(10); % rad
maxOmegaRef = deg2rad(15); % rad/s
maxdOmegaRef = deg2rad(30); % rad/s^2  (this setting is crucial when torque saturations come into play)

%% Weights
WPathFollow = 9999;  % weight on longitudinal tracking error - should be high to achieve accurate path following
WLateral = 25;  % weight on lateral tracking error
WVelocity = 80;  % weight on error between desired velocity and actual longitudinal velocity
WProgress = 0;  % punishment of being far away from the end of the fitted path
WSmoothness = 100;  % punishment of changes in angular velocity reference (hence weight on d_omega_ref)
WOmega = 20;  % weight on angular velocity reference (omega_ref)
WAngle = 10;  % weight on angle reference
WObstacles = 20;  % weight on the exponential obstacle proximity, defined by an offset and scale value
ProximityOffset = 0.3;  % exponential obstacle offset
ProximityScale = 10;  % exponential obstacle scale

% Horizon weight matrix (cost)
Wdiag = [WPathFollow, WLateral,  WAngle,WAngle,  WOmega,WOmega,  WVelocity,WProgress,  WSmoothness,WSmoothness,  WObstacles,  99999,9999,9999]; % [ lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   domega_ref_x;domega_ref_y;   obstacle_proximity;   velocity_slack;angle_slack;proximity_slack ]
Wmat = diag(Wdiag);

% Final state weight matrix (cost)
WNdiag = [WPathFollow, 10*WLateral,  WAngle,WAngle,  WOmega,WOmega,  WVelocity,WProgress,  WObstacles]; % [ lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   obstacle_proximity ]
WNmat = diag(WNdiag);

%% Window parameters
WindowWidth = 5.0;
WindowHeight = 5.0;
WindowOffset = [0, 0]; % Window offset relative to robot, hence positive X means window is moved in positive X direction in robot frame so robot center will move to a negative X position in the window
pathApproximationOrder = 8;
VelocityDefinedWindow = false; % will replace the parameters above if set to true
WindowOrientationSelection = 0; % 0=inertial frame, 1=robot yaw (heading), 2=velocity direction