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
omeg_b0 = [0, 0, 0]';
dq0 = 1/2 * Phi(q0) * vec * omeg_b0;
xy0 = [0, 0]';
dxy0 = [0, 0]';
tau0 = [0, 0, 0]';

q_ref = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
omega_ref = [0, 0, 0]';

X0 = [xy0;q0;dxy0;dq0];

%% Linearization with Sliding mode controller
A = zeros(12,12);
B = zeros(12,4);

tau0 = SlidingModeControl(X0,q_ref,omega_ref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon);
dX0 = EvaluateCompleteODE(X0, tau0, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);

delta = 0.000001;
for (j = 1:12)
    deltaX = zeros(12,1);
    deltaX(j) = delta;    
    X1 = X0 + deltaX;    
    tau1 = SlidingModeControl(X1,q_ref,omega_ref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon); 
    dX1 = EvaluateCompleteODE(X1, tau1, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    A(:,j) = (dX1-dX0) / delta;
end

delta = 0.000001;
for (j = 1:4)
    deltaqref = zeros(4,1);
    deltaqref(j) = delta;    
    tau1 = SlidingModeControl(X1,q_ref+deltaqref,omega_ref,COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,SlidingManifold,SwitchingLaw,K,eta,epsilon);   
    dX1 = EvaluateCompleteODE(X0, tau1, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    B(:,j) = (dX1 - dX0) / delta;
end

A = round(A, -floor(log10(delta))-2);
B = round(B, -floor(log10(delta))-2);

colNames = {'x','y','q1','q2','q3','q4','dx','dy','dq1','dq2','dq3','dq4'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4'};
A_withLabels = array2table(A,'RowNames',rowNames,'VariableNames',colNames)

colNames = {'qref1','qref2','qref3','qref4'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4'};
B_withLabels = array2table(B,'RowNames',rowNames,'VariableNames',colNames)

save('generated/ClosedLoopModelMatrices_SlidingMode', 'A', 'B', 'A_withLabels', 'B_withLabels');

%% Pole/zero analysis
C = [0,0,0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0];

sys = ss(A, B, C, zeros(2,4));
[Poles, Zeros] = pzmap(sys)
pzmap(sys)

%% Extract the steady state acceleration relationship between qref and linear acceleration
AccelerationConstant_q3_to_ddx = A(7,5) + B(7,3)
AccelerationConstant_q2_to_ddy = A(8,4) + B(8,2)