% Generate MEX files for optimization of functions used during simulation
% of MPC (e.g., trajectory fitting functions)
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'functions/polynomial'));
addpath(fullfile(scriptDir, 'functions/trajectory'));
addpath(fullfile(scriptDir, 'functions/visualization'));
cd(fullfile(scriptDir, 'generated'));

% Optimize 'FitReferencePathPolynomial'
% function [WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractWindowTrajectory(TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, WindowWidth, WindowHeight, WindowOffset, OrientationSelection)
TrajectoryPoints = coder.typeof(double(0), [Inf,2], [1,0]); % define matrix of variable size: m x 2
RobotPos = coder.typeof(double(0),[1 2],0);
RobotYaw = coder.typeof(double(0));
Velocity = coder.typeof(double(0),[1 2],0);
ExtractDist = coder.typeof(double(0));
WindowWidth = coder.typeof(double(0));
WindowHeight = coder.typeof(double(0));
WindowOffset = coder.typeof(double(0),[1 2],0);
OrientationSelection = coder.typeof(double(0));
codegen ExtractWindowTrajectory -nargout 3 -args { TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, WindowWidth, WindowHeight, WindowOffset, OrientationSelection };

% Optimize 'ExtractDistanceTrajectory'
% [WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractDistanceTrajectory(TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, OrientationSelection)
TrajectoryPoints = coder.typeof(double(0), [Inf,2], [1,0]); % define matrix of variable size: m x 2
RobotPos = coder.typeof(double(0),[1 2],0);
RobotYaw = coder.typeof(double(0));
Velocity = coder.typeof(double(0),[1 2],0);
ExtractDist = coder.typeof(double(0));
OrientationSelection = coder.typeof(double(0));
codegen ExtractDistanceTrajectory -nargout 3 -args { TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, OrientationSelection };

% Optimize 'FitReferencePathPolynomial'
% function [TrajectoryPoints, coeff_xs, coeff_ys, windowTrajectoryLength, minDistancePoint] = FitReferencePathPolynomial(WindowTrajectoryPoints, approximation_order, velocity, ts, N)
WindowTrajectoryPoints = coder.typeof(double(0), [Inf,2], [1,0]); % define matrix of variable size: m x 2
approximation_order = coder.typeof(double(0));
velocity = coder.typeof(double(0));
ts = coder.typeof(double(0));
N = coder.typeof(double(0));
codegen FitReferencePathPolynomial -nargout 5 -args { WindowTrajectoryPoints, approximation_order, velocity, ts, N };