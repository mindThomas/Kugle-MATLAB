%% This script linearizes the model by adding a small pertubation, thus computing the partial derivatives numerically
addpath('../Parameters');
addpath('../Model');
addpath('../Model/generated');
Constants_Kugle

% Pertubation delta for numerical partial derivation
delta = 0.000001;

%% Linearization point
q0 = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
omeg_b0 = [0, 0, 0]';
dq0 = 1/2 * Phi(q0) * vec * omeg_b0;
xy0 = [0, 0]';
dxy0 = [0, 0]';
tau0 = [0, 0, 0]';

X0 = [xy0;q0;dxy0;dq0];

%% Linearization of non-linear model
A = zeros(12,12);
B = zeros(12,3);

dX0 = EvaluateCompleteODE(X0, tau0, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);

for (j = 1:12)
    deltaX = zeros(12,1);
    deltaX(j) = delta;    
    X1 = X0 + deltaX;
    dX1 = EvaluateCompleteODE(X1, tau0, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    A(:,j) = (dX1-dX0) / delta;
end

for (j = 1:3)
    deltatau = zeros(3,1);
    deltatau(j) = delta;    
    tau1 = tau0 + deltatau;
    dX1 = EvaluateCompleteODE(X0, tau1, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g);
    B(:,j) = (dX1 - dX0) / delta;
end

A = round(A, -floor(log10(delta))-2);
B = round(B, -floor(log10(delta))-2);

colNames = {'x','y','q1','q2','q3','q4','dx','dy','dq1','dq2','dq3','dq4'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4'};
A_withLabels = array2table(A,'RowNames',rowNames,'VariableNames',colNames)

colNames = {'tau1','tau2','tau3'};
rowNames = {'dx','dy','dq1','dq2','dq3','dq4','ddx','ddy','ddq1','ddq2','ddq3','ddq4'};
B_withLabels = array2table(B,'RowNames',rowNames,'VariableNames',colNames)

save('generated/LinearizedModelMatrices', 'A', 'B', 'A_withLabels', 'B_withLabels');

%% Pole/zero analysis
C = [0,0,0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,0,0,1,0,0,0,0];

sys = ss(A, B, C, zeros(2,3));
[Poles, Zeros] = pzmap(sys)