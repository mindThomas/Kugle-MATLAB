clear all;

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'functions/polynomial'));
addpath(fullfile(scriptDir, 'functions/trajectory'));
addpath(fullfile(scriptDir, 'functions/visualization'));
addpath(fullfile(scriptDir, 'generated'));
addpath(fullfile(scriptDir, '../../Parameters'));
addpath(fullfile(scriptDir, '../../Model/generated')); % for SteadyStateAcceleration used when simulating ballbot
load(fullfile(scriptDir, '../../Linearization/generated/SteadyStateAccelerationConstants.mat'));
Constants_Kugle;
Parameters_MPC;
Parameters_Simulation;

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

%% Show visualization while running ?
PlotEnabled = true;
IncludeLegendInPlot = false; % including the legend will slow down the plotting
vis = TrajectoryVisualizer(PlotEnabled);

%% Generate trajectory
TrajectoryPoints = GenerateTestTrajectory; % GenerateTestTrajectory or GenerateTestTrajectory_FigureEight

%% Set initial point and settings
vis.GlobalTrajectoryPoints = TrajectoryPoints;
vis.ChangeLimits(-7, -2, 7, 2);
vis.x = x_init(1);
vis.y = x_init(2);
vis.yaw = deg2rad( 0 );
if (PlotEnabled)
    vis.Draw;
end

% Simulation and configuration parameters
UseNonlinearSteadyStateModelForSimulation = true; % whether to use the steady state acceleration model for simulation

% Simulated obstacle (defined in inertial frame)
ObstacleAvoidanceEnabled = MPC_EnableStaticObstacles;
Obstacle_Avoidance_Clearance = 0.05;
RandomizedObstacles = MPC_RandomObstacles; % enable random obstacles by setting this >0

% Define some static obstacles that are always present
%             X     Y     R   
Obstacles = [ 4,   1.9,  0.2;
             5.5,  1.6,  0.8;
             3.10, 2.2,  0.3;
             5.7, -1.93, 1.0;
             6.6,  0.3,  0.4;
            -5.3,  1.8,  0.4;
            -6.1, -1.0, 0.6];

if (RandomizedObstacles > 0)
    for (n = 1:RandomizedObstacles)
        obs_x = vis.x_min + (vis.x_max-vis.x_min)*rand(1,1);
        obs_y = vis.y_min + (vis.y_max-vis.y_min)*rand(1,1);
        obs_r = 0.2 + 0.8 * rand(1,1); % between 0.2-1.0 meter radius
        Obstacles = [Obstacles;
                     [obs_x, obs_y, obs_r]];
    end
end

% If obstacle avoidance should not be tested, remove all obstacles
if (ObstacleAvoidanceEnabled == false)
    Obstacles = repmat([100, 100, 1], [5,1]); % needs at least 5 obstacles, here just set to far away
end  
         
% Initial robot stats = [q2,q3, x,y, dx,dy, omega_ref_x,omega_ref_y]
RobotStates = [eul2quat([vis.yaw,0,0], 'ZYX'),  vis.x,vis.y,  0,0]';

% MPC settings
order = pathApproximationOrder; % polynomial approximation order
velocity = MPC_DesiredVelocity; % desired velocity - note that the final constraint of zero velocity will reduce the maximum drivable velocity (since it has to be able to decelerate within the horizon)
ts = Ts_MPC;
tEnd = N*ts;
timeVec = (0:N)' * ts;

SpaceBehindRobot = 0.2;
if (VelocityDefinedWindow)        
    % WindowHeight/2 + WindowOffsetX = N*ts*velocity
    WindowHeight = N*ts*velocity + SpaceBehindRobot;
    if (WindowHeight < 2*SpaceBehindRobot)
        WindowHeight = 2*SpaceBehindRobot;
    end
    WindowOffset = [WindowHeight/2-SpaceBehindRobot, 0];
end

% MPC initialization
% x = [q2,q3, x,y, dx,dy, s,ds, omega_ref_x,omega_ref_y]
x0 = [0,0,  0,0,  0.001,0.001,  0,0.001,  0,0]; % init with a small velocity - otherwise problem is not feasible ???
xInit = [timeVec, repmat(x0, [(N+1),1])];
u0 = [0,0,0,0,0];
uInit = [timeVec(1:end-1), repmat(u0, [N,1])];

od0 = [velocity, maxVelocity, maxAngle, maxOmegaRef, 99, 0, [zeros(1,8),1000,0], [zeros(1,8),0,0], repmat([99,99,0.01], [1,5])];
PreviousHorizonLength = 0;
omeg_ref_input_old = [0,0]';

clear acado_input;

MPClog = [];
Stateslog = [];

for (i = 1:400)
    %% Load states from state "estimate"
    RobotX = RobotStates(5);
    RobotY = RobotStates(6);
    
    q = RobotStates(1:4);
    p = Phi(q)*Gamma(q)'*[0,1,0,0]';
    RobotYaw = atan2(p(3),p(2));   
    q_heading = [cos(RobotYaw/2);0;0;sin(RobotYaw/2)];
    q_tilt = Phi(q_heading)' * q; % tilt in heading frame   
    if (q_tilt(1) < 0)
        q_tilt = -q_tilt; % invert q_tilt to take shortest rotation
    end
    R_heading = [cos(RobotYaw), sin(RobotYaw);
                 -sin(RobotYaw), cos(RobotYaw)];
    
    Stateslog = [Stateslog; RobotStates'];
             
    vis.x = RobotX;
    vis.y = RobotY;    
    vis.yaw = RobotYaw;
    if (PlotEnabled)
        vis.Draw;
        figure(vis.fig);
        subplot(1,2,1);
        hold on;
        % plot obstacles
        for (j = 1:size(Obstacles,1))
            d = 2 * Obstacles(j,3);
            px = Obstacles(j,1) - Obstacles(j,3);
            py = Obstacles(j,2) - Obstacles(j,3);
            rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[0 0 1]);            
        end
        hold off;   
        xlim([min(TrajectoryPoints(:,1))*1.1, max(TrajectoryPoints(:,1))*1.1]);
        ylim([min(TrajectoryPoints(:,2))*1.5, max(TrajectoryPoints(:,2))*1.5]);
    end
    
    
    % Recompute window size      
    RobotVelocity = sqrt(sum(RobotStates(7:8).^2));
    SpaceBehindRobot_Dynamic = (N*ts*velocity - SpaceBehindRobot) * min(max((velocity - RobotVelocity), 0), 1) + SpaceBehindRobot;    
    if (VelocityDefinedWindow)        
        % WindowHeight/2 + WindowOffsetX = N*ts*velocity
        WindowHeight = N*ts*velocity + SpaceBehindRobot_Dynamic;
        if (WindowHeight < 2*SpaceBehindRobot_Dynamic)
            WindowHeight = 2*SpaceBehindRobot_Dynamic;
        end
        WindowOffset = [WindowHeight/2-SpaceBehindRobot_Dynamic, 0];
    end    

    %ExtractionLength = 1.3*N*ts*RobotVelocity; % use current velocity to determine the length of the reference trajectory use
    ExtractionLength = 1.3*PreviousHorizonLength;
    if (ExtractionLength < 2.0)
        ExtractionLength = 2.0;
    end
    
    tic
    %[WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractWindowTrajectory(TrajectoryPoints, [RobotX, RobotY], RobotYaw, [RobotStates(7),RobotStates(8)], 1.5*N*ts*velocity, WindowWidth, WindowHeight, WindowOffset, OrientationParameter);    
    [WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractDistanceTrajectory(TrajectoryPoints, [RobotX, RobotY], RobotYaw, [RobotStates(7),RobotStates(8)], ExtractionLength, WindowOrientationSelection);    
    R_window = [cos(WindowOrientation), sin(WindowOrientation);
             -sin(WindowOrientation), cos(WindowOrientation)];         
    tWindow = toc;
    
    tic;
    % Compute reference path polynomial
    [ReferencePoints, coeff_trajectory_x, coeff_trajectory_y, windowTrajectoryLength, minDistancePoint] = FitReferencePathPolynomial(WindowTrajectory, order, velocity, ts, N+1);

    % Compute coordinate on reference path closest to the robot
    x_min = EvaluatePolynomial(coeff_trajectory_x, minDistancePoint);
    y_min = EvaluatePolynomial(coeff_trajectory_y, minDistancePoint);        

    % Generate points for visualization 
    s_eval = linspace(0, windowTrajectoryLength, 100);
    x_poly = polyval(coeff_trajectory_x, s_eval);
    y_poly = polyval(coeff_trajectory_y, s_eval);      
    
    tExtract = toc;

    % Compute Tilt direction
    tiltDirection = R_window * [0,1,0,0;0,0,1,0] * Phi(q)*Gamma(q)'*[0,0,0,1]';
    
    % Visualize step  
    if (PlotEnabled)
        figure(vis.fig);
        subplot(1,2,2);
        plot(-WindowTrajectory(:,2), WindowTrajectory(:,1), 'k*'); % plot rotated
        Vel = R_window * RobotStates(7:8);
        PlotRobotWithTiltAndVelocity([0,0], deg2rad(90)+RobotYaw-WindowOrientation, 0.05, [-Vel(2),Vel(1)], [-tiltDirection(2),tiltDirection(1)]);
        hold on;        
        line('XData',[0,-y_min], 'YData',[0,x_min], 'Color',[0 1 0],'LineWidth',0.5);
        plot(-y_poly, x_poly, 'b--');   
        
        % plot obstacle
        for (j = 1:size(Obstacles,1))
            d = 2 * Obstacles(j,3);
            px = -(Obstacles(j,2) - RobotY) - Obstacles(j,3);
            py = (Obstacles(j,1) - RobotX) - Obstacles(j,3);
            rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[0 0 1]);            
        end       
        
        hold off;
        axis equal;
        xlim([-WindowWidth/2 + WindowOffset(2), WindowWidth/2 + WindowOffset(2)]);
        ylim([-WindowHeight/2 + WindowOffset(1), WindowHeight/2 + WindowOffset(1)]);       
        if (IncludeLegendInPlot)
            legend('Window trajectory (desired)', 'Robot', 'Velocity vector', 'Tilt direction', 'Shortest distance', 'Fitted trajectory');                   
        end
    end
    
    %R_windowheading = [cos(WindowOrientation-RobotYaw), sin(WindowOrientation-RobotYaw);
    %     -sin(WindowOrientation-RobotYaw), cos(WindowOrientation-RobotYaw)];     
    
    %q_tiltOrientation = [1; R_windowheading * q_tilt(2:3); 0];         
    
    q_tilt_inertial = Gamma(q_heading)' * q; % tilt in inertial frame
    if (q_tilt_inertial(1) < 0) % negate quaternion
        q_tilt_inertial = -q_tilt_inertial;
    end
    q_tilt_window = R_window * q_tilt_inertial(2:3);
    
    % Set current states as initial value to MPC and set reference trajectory
    % Convert quaternion from inertial frame into heading frame type of quaternion
    % Note that first column, xInit(:,1), is time
    % xInit = [t, q2,q3, x,y, dx,dy, s,ds, omega_ref_x,omega_ref_y]
    xInit(1,2) = q_tilt_window(1) + rand(1,1)/1000; % q2    (add some small noise, as the problem other seem to become infeasible sometimes?)
    xInit(1,3) = q_tilt_window(2) + rand(1,1)/1000; % q3
    xInit(:,4) = xInit(:,4) - xInit(1,4); % make xInit(1,4) = 0   (centering the robot in the prediction window)
    xInit(:,5) = xInit(:,5) - xInit(1,5); % make xInit(1,5) = 0   (centering the robot in the prediction window)    
    xInit(1,6:7) = R_window * RobotStates(7:8); % OBS. Inertial frame velocity gets rotated into heading frame/window frame        
    xInit(:,8) = xInit(:,8) - xInit(1,8); % make xInit(1,8) = 0
    
    tic;
    % Find solution with MPC (optimizer)    
    coeff_x = [zeros(1,10-length(coeff_trajectory_x)), coeff_trajectory_x'];
    coeff_y = [zeros(1,10-length(coeff_trajectory_y)), coeff_trajectory_y'];
    %od0 = [velocity, maxAngle, maxOmegaRef, windowTrajectoryLength, minDistancePoint, coeff_x, coeff_y, 0, 0];                              
    %acado_input.od = repmat(od0, [N+1,1]); % Online data                   

    NearestObstacles = GetNearestObstacles([RobotX, RobotY], Obstacles, 5);

    % prepare obstacle list and convert to MPC frame
    obstacle1 = NearestObstacles(1,:) - [RobotX, RobotY, 0];
    obstacle2 = NearestObstacles(2,:) - [RobotX, RobotY, 0];
    obstacle3 = NearestObstacles(3,:) - [RobotX, RobotY, 0];
    obstacle4 = NearestObstacles(4,:) - [RobotX, RobotY, 0];
    obstacle5 = NearestObstacles(5,:) - [RobotX, RobotY, 0];

    % Prepare online data matrix
    od0 = [velocity, maxVelocity, maxAngle, maxOmegaRef, maxdOmegaRef, windowTrajectoryLength, minDistancePoint, coeff_x, coeff_y, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5];                              
    acado_input.od = repmat(od0, [N+1,1]); % Online data  

    % Assemble acado_input struct
    acado_input.W = Wmat;
    acado_input.WN = WNmat; 
    %acado_input.Wlx = Slx;
    %acado_input.Wlu = Slu;
    acado_input.x = xInit(1:(N+1),2:end);
    acado_input.x0 = xInit(1,2:end);
    %acado_input.y = [zeros(N, 3), uInit(1:N,2:3)];
    acado_input.y = zeros(N, 12);
    acado_input.yN = zeros(1, 8);
    acado_input.u = uInit(1:N,2:end);        

    % Run MPC
    acado_output = kugle_mpc_obstacles(acado_input);

    % Extract MPC predictions and controls
    out.STATES = [timeVec, acado_output.x];
    out.CONTROLS = [timeVec(1:end-1), acado_output.u];
    out.CONVERGENCE_ACHIEVED = (acado_output.info.status == 0);

    MPClog = [MPClog; out];
    MPCtrajectory = out.STATES(:,4:5);
    MPCsRef = out.STATES(:,8);
    MPCreferencePoints = [polyval(coeff_trajectory_x, MPCsRef), polyval(coeff_trajectory_y, MPCsRef)];    

    % Plot tube constraints
    if (PlotEnabled)
        figure(vis.fig);
        subplot(1,2,2);
        hold on;       
        sInit = xInit(:,8);
        MPCinitReferencePoints = [polyval(coeff_trajectory_x, sInit), polyval(coeff_trajectory_y, sInit)];    
        dcoeff_x = ComputeDerivativePolynomialCoefficients(coeff_trajectory_x);
        dcoeff_y = ComputeDerivativePolynomialCoefficients(coeff_trajectory_y);                
        dx_eval = EvaluatePolynomial(dcoeff_x, sInit);
        dy_eval = EvaluatePolynomial(dcoeff_y, sInit); 
        plot(-MPCinitReferencePoints(:,2), MPCinitReferencePoints(:,1), 'mo');   
        hold off;
    end
    
    % Set the initialization parameters for next iteration to be the current MPC solution     
    %if (out.CONVERGENCE_ACHIEVED)
        %expectedPosChange = out.STATES(2,4:5) - out.STATES(1,4:5);
        %expectedSChange = out.STATES(2,8) - out.STATES(1,8);

        % Update initialization states and controls to be used in next iteration
        xInit = [timeVec, [out.STATES(2:end,2:end); out.STATES(end,2:end)]]; % start from step 1 (not 0) in current MPC solution and repeat final state in the end
        uInit = [timeVec(1:length(out.CONTROLS)), [out.CONTROLS(2:end,2:end); zeros(1,size(out.CONTROLS,2)-1)]]; % start from step 1 (not 0) in current MPC solution and insert zeros in the end
        % This update/propagation could have been computed more exact instead of just repeating the last state and setting the control to zero.
        % However due to the terminal constraints, the control will be zero at the end and at least the angle and angular velocity states will also be zero.
        % So it would just be the position that would  have to be propagated, which is therefore carried out above.
                
        % Propagate position initialization        
        %xInit(:,4:5) = xInit(:,4:5) - expectedPosChange; % correct initial guess of position with expected position movement
        %xInit(:,8) = xInit(:,8) - expectedSChange; % correct initial guess of position with expected position movement
        %xInit(:,9) = xInit(:,9);
        
        %xInit(:,8:9) = out.STATES(:,8:9);
    %else
    %    xInit = [timeVec, repmat(out.STATES(2,2:end), [N+1, 1])];
    %    uInit = [timeVec(1:length(out.CONTROLS)), zeros(length(out.CONTROLS), size(out.CONTROLS,2)-1)];
    %end       
    
    tMPC = toc;
    
    % Visualize MPC prediction
    if (PlotEnabled)
        figure(vis.fig);
        subplot(1,2,2);
        hold on;          
        plot(-MPCtrajectory(:,2), MPCtrajectory(:,1), 'r*');          
        plot(-MPCreferencePoints(:,2), MPCreferencePoints(:,1), 'go');    
        %legend('Window trajectory (desired)', 'Robot', 'Velocity vector', 'Tilt direction', 'Shortest distance', 'Fitted trajectory', 'MPC trajectory', 'MPC evaluation points',  'Location', 'southside', 'Orientation','vertical'); 
        hold off;
        drawnow;
        F(i) = getframe(vis.fig);
    end        
    
    if (out.CONVERGENCE_ACHIEVED)
        PreviousHorizonLength = out.STATES(end,8);
    end
    
    % Control output is given in Window frame
    % We thus need to convert the window frame angular velocity to body frame
    omeg_ref_input = omeg_ref_input_old;
    if (out.CONVERGENCE_ACHIEVED)
        %omeg_ref_input = omeg_ref_input + out.CONTROLS(1,2:3)' * ts;
        domeg_ref_input = out.CONTROLS(1,2:3)';
        omeg_ref_input = out.STATES(2,10:11)'; % omega_ref in window frame
    end
    omeg_ref_input_old = omeg_ref_input;
    
    % First we convert W_omega_ref_input to I_omega_ref_input
    I_omega_ref_input = R_window' * omeg_ref_input;    
    yawdot_ref = deg2rad(0); % 30 deg/s
    
    % Add process noise
    % 3*sigma = 0.1*maxOmegaRef
    sigma_u = 0.1/3 * maxOmegaRef;        
    u_noise = randn(2,1) * sigma_u;
    %B_omega_ref_input = B_omega_ref_input + u_noise;        

    % Next we convert to body frame, I_omega_ref_input to B_omega_ref_input
    B_omega_ref_input = devec * Phi(q)' * Gamma(q) * vec * [I_omega_ref_input;yawdot_ref];          
    
    % Simulation model modifications
    %COM(2) = -sin(deg2rad(4))*0.35; % simulate a shifted/offset COM
    
    % Simulate "ts" forward in time using the first control as input
    % control input variance (for simulation)    
    if (UseNonlinearSteadyStateModelForSimulation)
        [t1,x1] = ode45(@(t,x) EvaluateNonlinearMPCmodel(t,x,B_omega_ref_input,constants,COM), [0 ts], RobotStates);
    else
        [t1,x1] = ode45(@(t,x) EvaluateLinearMPCmodel(t,x,B_omega_ref_input,AccelerationConstant_q3_to_ddx,AccelerationConstant_q2_to_ddy), [0 ts], RobotStates);
    end
    RobotStates = x1(end,:)';
    YPR = quat2eul(RobotStates(1:4)');
    
    fprintf('Step: %.0f\n', i);
    fprintf('Time window extract: %2.4f\n', tWindow);
    fprintf('Time polynomial fitting: %2.4f\n', tExtract);
    fprintf('Time MPC: %2.4f\n', tMPC);
    if (out.CONVERGENCE_ACHIEVED)
        fprintf('MPC converged: true\n');
    else
        fprintf('MPC converged: false\n');             
        pause;
    end
    fprintf('Current velocity: %2.2f (velocity slack: %2.2f)\n', sqrt(sum(RobotStates(7:8).^2)), out.CONTROLS(1,5));    
    fprintf('Current max angle: %2.2f deg (angle slack: %2.2f deg)\n', rad2deg(max(YPR(2),YPR(3))), rad2deg(out.CONTROLS(1,6)));
    fprintf('Path distance s[N]=%2.2f vs path length=%2.2f\n', out.STATES(end,8)+minDistancePoint, windowTrajectoryLength);
    acado_output.info
    fprintf('---------------\n');
end


if (PlotEnabled)
    outputVideo = VideoWriter(fullfile('MPCwithSimTest.avi'));
    outputVideo.FrameRate = 1/ts;
    open(outputVideo);
    for (i = 1:length(F))
        writeVideo(outputVideo, F(i).cdata);
    end
    close(outputVideo);
end

%% Visualize resulting trajectory
figure(4);
plot(TrajectoryPoints(:,1), TrajectoryPoints(:,2), 'k-');
hold on;
plot(Stateslog(:,5),Stateslog(:,6), 'r-');
% plot obstacle
for (j = 1:size(Obstacles,1))
    d = 2 * Obstacles(j,3);
    px = Obstacles(j,1) - Obstacles(j,3);
    py = Obstacles(j,2) - Obstacles(j,3);
    rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[0 0 1]);            
end   
hold off;
axis equal;
xlim([min(TrajectoryPoints(:,1))*1.1, max(TrajectoryPoints(:,1))*1.1]);
ylim([min(TrajectoryPoints(:,2))*1.5, max(TrajectoryPoints(:,2))*1.5]);
figure(5);
VelocityLog = sqrt(Stateslog(:,7).^2 + Stateslog(:,8).^2);
plot(VelocityLog);
title('Velocity');
figure(6);
TiltLog = quat2eul(Stateslog(:,1:4),'ZYX');
MaxTilt = max(TiltLog(:,2),TiltLog(:,3));
subplot(2,1,1);
plot(rad2deg(TiltLog(:,3)));
title('Roll');
ylabel('Degrees');
subplot(2,1,2);
plot(rad2deg(TiltLog(:,2)));
title('Pitch');
ylabel('Degrees');