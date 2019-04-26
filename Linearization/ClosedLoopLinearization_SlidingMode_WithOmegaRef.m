%% This script linearizes the model by adding a small pertubation, thus computing the partial derivatives numerically
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
addpath(fullfile(scriptDir, '../Model'));
addpath(fullfile(scriptDir, '../Model/generated'));
addpath(fullfile(scriptDir, '../Controllers/SlidingMode'));
Constants_Kugle % load model parameters
Parameters_Controllers % load controller parameters

% Pertubation delta for numerical partial derivation
delta = 0.000001;

%% Quaternion operators
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

%% Linearization point
q0 = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
qref0 = q0;
omeg_b0 = [0, 0, 0]';
dq0 = 1/2 * Phi(q0) * vec * omeg_b0;
xy0 = [0, 0]';
dxy0 = [0, 0]';
tau0 = [0, 0, 0]';

q_ref = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
omega_ref = [0, 0, 0]';

X0 = [xy0;q0;dxy0;dq0;qref0];

%% Linearization with Sliding mode controller
A = zeros(16,16);
B = zeros(16,3);

tau0 = SlidingModeControl(X0(1:12),X0(13:16),omega_ref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon);
dX0(1:12) = EvaluateCompleteODE(X0(1:12), tau0, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
dX0(13:16) = 1/2 * Phi(X0(13:16)) * vec * omega_ref; % dq_ref

delta = 0.000001;
for (j = 1:16)
    deltaX = zeros(16,1);
    deltaX(j) = delta;    
    X1 = X0 + deltaX;    
    tau1 = SlidingModeControl(X1(1:12),X1(13:16),omega_ref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon); 
    dX1(1:12) = EvaluateCompleteODE(X1(1:12), tau1, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    dX1(13:16) = 1/2 * Phi(X1(13:16)) * vec * omega_ref; % dq_ref
    A(:,j) = (dX1-dX0) / delta;
end

delta = 0.000001;
for (j = 1:3)
    deltaomegaref = zeros(3,1);
    deltaomegaref(j) = delta;    
    tau1 = SlidingModeControl(X0(1:12),X0(13:16),omega_ref+deltaomegaref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon);   
    dX1(1:12) = EvaluateCompleteODE(X0(1:12), tau1, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    dX1(13:16) = 1/2 * Phi(X0(13:16)) * vec * (omega_ref+deltaomegaref); % dq_ref
    B(:,j) = (dX1 - dX0) / delta;
end

A = round(A, -floor(log10(delta))-2);
B = round(B, -floor(log10(delta))-2);

colNames = {'x','y','q1','q2','q3','q4','dx','dy','dq1','dq2','dq3','dq4','qr0','qr1','qr2','qr3'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4','dqr0','dqr1','dqr2','dqr3'};
A_withLabels = array2table(A,'RowNames',rowNames,'VariableNames',colNames)

colNames = {'omeg_ref_x','omeg_ref_y','omeg_ref_z'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4','dqr0','dqr1','dqr2','dqr3'};
B_withLabels = array2table(B,'RowNames',rowNames,'VariableNames',colNames)

save(fullfile(scriptDir, 'generated/ClosedLoopModelMatrices_SlidingMode_WithOmegaRef'), 'A', 'B', 'A_withLabels', 'B_withLabels');

%% Extract the steady state acceleration relationship between qref and linear acceleration
AccelerationConstant_q2_to_ddx = A(7,5) + A(7,15)
AccelerationConstant_q1_to_ddy = A(8,4) + A(8,14)

save(fullfile(scriptDir, 'generated/SteadyStateAccelerationConstants.mat'), 'AccelerationConstant_q2_to_ddx', 'AccelerationConstant_q1_to_ddy');