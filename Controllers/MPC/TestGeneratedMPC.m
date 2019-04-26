scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'generated'));
addpath(fullfile(scriptDir, '../../Parameters'));
Parameters_MPC;

%% Test MPC with MATLAB interface
% Test parameters
xyInit = [0, 0]';
xyFinal = [0.2, 0.2]';

desiredVelocity = 1.0;
% Create velocity profile
maxAcceleration = 1.0;
maxVelChangePrStep = maxAcceleration*Ts_MPC;
velocityProfile = zeros(N+1,1);
velocityProfile(1:N/2+1) = maxVelChangePrStep*(0:(N/2));
velocityProfile((N/2+2):end) = velocityProfile(N/2)-maxVelChangePrStep*(0:(N/2-1));
velocityProfile = min(velocityProfile, desiredVelocity);

% Define MPC initialization state
% x   = [q2,q3,      x,y,              dx,dy,      s,ds,   omega_ref_x,omega_ref_y]
xInit = [0,0,  xyInit(1),xyInit(2),  0.001,0.001,  0,0.001,  0,0]'; % init with a small velocity - otherwise problem is not feasible ???
uInit = [0,0,0,0,0]';

% Generate test trajectory
trajectoryStart = 0;
trajectoryLength = sqrt(xyFinal'*xyFinal);
cx = zeros(1,10);
cy = zeros(1,10);
cx(end) = xyInit(1);
cy(end) = xyInit(2);
cx(end-1) = (xyFinal(1) - xyInit(1)) / trajectoryLength;
cy(end-1) = (xyFinal(2) - xyInit(2)) / trajectoryLength;

% Generate test obstacles
obs_vec = [99, 99, 0.1]; % [x, y, r]
obstacles = repmat(obs_vec, [1, 5]); % no obstacles

% Set online data
od0 = [desiredVelocity, maxVelocity, maxAngle, maxOmegaRef, trajectoryLength, trajectoryStart, cx, cy, obstacles];
odInit = repmat(od0, [N+1,1]);

%odInit(end,1) = 0;
%odInit(end,2) = 0;
%odInit(:,2) = velocityProfile;
odInit(end,1) = 0;
%odInit(end-1,1) = 0;
%odInit(end-2,1) = 0;
%odInit((N/2+2):end,2) = 0;

% Run MPC
clear acado_input;
acado_input.od = odInit; % Online data
acado_input.W = Wmat;
acado_input.WN = WNmat;
acado_input.x = repmat(xInit', [N+1,1]);
acado_input.x0 = xInit';
acado_input.y = zeros(N, size(Wmat, 1));
acado_input.yN = zeros(1, size(WNmat, 1));
acado_input.u = repmat(uInit', [N,1]); % Set initial inputs to zero

tic;
acado_output = kugle_mpc_obstacles(acado_input);
toc;

acado_output.info

% Extract horizon values for plotting
xy_horizon = acado_output.x(:,3:4); 
s_horizon = acado_output.x(:,7);
xy_ref_horizon = [polyval(cx, s_horizon), polyval(cy, s_horizon)]; 

% Generate points for visualization 
s_eval = linspace(0, trajectoryLength, 100);
x_poly = polyval(cx, s_eval);
y_poly = polyval(cy, s_eval);       

% Visualize horizon/prediction
figure(1);   
plot(-y_poly, x_poly, 'b--');
hold on;  
plot(-xy_horizon(:,2), xy_horizon(:,1), 'r*');  
axis equal;
WindowWidth = 1;
WindowHeight = 1;
WindowOffset = [0, 0];
xlim([-WindowWidth/2 + WindowOffset(2), WindowWidth/2 + WindowOffset(2)]);
ylim([-WindowHeight/2 + WindowOffset(1), WindowHeight/2 + WindowOffset(1)]);              
plot(-xy_ref_horizon(:,2), xy_ref_horizon(:,1), 'go');    
legend('Window trajectory (desired)', 'MPC trajectory', 'MPC evaluation points'); 
hold off;
drawnow;