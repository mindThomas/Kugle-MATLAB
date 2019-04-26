clear all;

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'generated'));
addpath(fullfile(scriptDir, '../../Parameters'));
addpath(fullfile(scriptDir, '../../Model/generated')); % for SteadyStateAcceleration used when simulating ballbot
load(fullfile(scriptDir, '../../Linearization/generated/SteadyStateAccelerationConstants.mat'));
Constants_Kugle;
Parameters_MPC;

%% Test definition
x0 = [eul2quat(deg2rad([0,0,0]),'ZYX'),  0,0,  0,0];

tspan1 = [0 5];   omeg_ref1 = [0, 0.01];
tspan2 = [5 15];  omeg_ref2 = [0.005, -0.01]';
tspan3 = [15 20]; omeg_ref3 = [-0.01, 0.01]';

%% Nonlinear simulation
[t1,x1] = ode45(@(t,x) EvaluateNonlinearMPCmodel(t,x,omeg_ref1,constants,COM), tspan1, x0);
[t2,x2] = ode45(@(t,x) EvaluateNonlinearMPCmodel(t,x,omeg_ref2,constants,COM), tspan2, x1(end,:));
[t3,x3] = ode45(@(t,x) EvaluateNonlinearMPCmodel(t,x,omeg_ref3,constants,COM), tspan3, x2(end,:));

t_nonlinear = [t1;t2;t3];
x_nonlinear = [x1;x2;x3];

%% Linear simulation
[t1,x1] = ode45(@(t,x) EvaluateLinearMPCmodel(t,x,omeg_ref1,AccelerationConstant_q2_to_ddx,AccelerationConstant_q1_to_ddy), tspan1, x0);
[t2,x2] = ode45(@(t,x) EvaluateLinearMPCmodel(t,x,omeg_ref2,AccelerationConstant_q2_to_ddx,AccelerationConstant_q1_to_ddy), tspan2, x1(end,:));
[t3,x3] = ode45(@(t,x) EvaluateLinearMPCmodel(t,x,omeg_ref3,AccelerationConstant_q2_to_ddx,AccelerationConstant_q1_to_ddy), tspan3, x2(end,:));

t_linear = [t1;t2;t3];
x_linear = [x1;x2;x3];

%% Recalculate corresponding acceleration for both cases for comparison
% Based on the nonlinear simulated quaternion
acceleration_nonlinear = SteadyStateAcceleration(COM_X,COM_Y,COM_Z,Jk,Jw,Mb,Mk,x_nonlinear(:,7),x_nonlinear(:,8),g,x_nonlinear(:,1),x_nonlinear(:,2),x_nonlinear(:,3),x_nonlinear(:,4),rk,rw);
acceleration_nonlinear = reshape(acceleration_nonlinear, [length(x_nonlinear), 2]);

acceleration_linear = [AccelerationConstant_q2_to_ddx*x_nonlinear(:,3), AccelerationConstant_q1_to_ddy*x_nonlinear(:,2)];

%% Visualize nonlinear simulation
t = t_nonlinear;
x = x_nonlinear;
q = x(:,1:4);
eul = rad2deg(quat2eul(q, 'ZYX'));
X = x(:,5);
Y = x(:,6);
dX = x(:,7);
dY = x(:,8);

figure(1);
subplot(3,1,1);
plot(t, X, t, Y);
legend('X', 'Y');
ylabel('Position [m]');
subplot(3,1,2);
plot(t, dX, t, dY);
legend('dX', 'dY');
ylabel('Velocity [m/s]');
subplot(3,1,3);
plot(t, eul(:,2), t, eul(:,3));
legend('Pitch', 'Roll');
ylabel('Angle [deg]');
xlabel('Time [s]');

%% Compare nonlinear and linear simulation
figure(2);
subplot(2,1,1);
plot(t_nonlinear, x_nonlinear(:,7), t_linear, x_linear(:,7));
title('dX');
legend('Nonlinear', 'Linear');
subplot(2,1,2);
plot(t_nonlinear, x_nonlinear(:,8), t_linear, x_linear(:,8));
title('dY');
legend('Nonlinear', 'Linear');

acceleration_nonlinear_discrete = diff(x_nonlinear(:,7:8)) ./ diff(t_nonlinear);

figure(3);
subplot(2,1,1);
plot(t_nonlinear, acceleration_nonlinear(:,1), t_nonlinear, acceleration_linear(:,1));
%plot(t_nonlinear, acceleration_nonlinear(:,1), t_linear, acceleration_linear(:,1), (t_linear(1:end-1)+t_linear(2:end))/2, acceleration_nonlinear_discrete(:,1), '*');
title('ddX');
legend('Nonlinear', 'Linear');
subplot(2,1,2);
plot(t_nonlinear, acceleration_nonlinear(:,2), t_nonlinear, acceleration_linear(:,2));
%plot(t_nonlinear, acceleration_nonlinear(:,2), t_linear, acceleration_linear(:,2), (t_linear(1:end-1)+t_linear(2:end))/2, acceleration_nonlinear_discrete(:,2), '*');
title('ddY');
legend('Nonlinear', 'Linear');