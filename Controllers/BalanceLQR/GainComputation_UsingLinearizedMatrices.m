scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Parameters'));
load(fullfile(scriptDir, '../../Linearization/generated/LinearizedModelMatrices.mat'));
Constants_Kugle;

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

%% Derive/compute linearized matrices (based on OneNote derivation)
A11 = zeros(3,3);
A12 = 1/2 * eye(3);
A21 = 2*devec * A(9:12, 3:6) * vec;
A22 = devec * A(9:12, 9:12) * vec;

Ae = [A11, A12;
      A21, A22];
Be = [zeros(3,3);
     2*devec * B(9:12,:)];

colNames = {'qerr1','qerr2','qerr3','omeg_err_x','omeg_err_y','omeg_err_z'};
rowNames = {'dqerr1','dqerr2','dqerr3','domeg_err_x','domeg_err_y','domeg_err_z'};
Ae_withLabels = array2table(Ae,'RowNames',rowNames,'VariableNames',colNames)

colNames = {'tau1','tau2','tau3'};
rowNames = {'dqerr1','dqerr2','dqerr3','domeg_err_x','domeg_err_y','domeg_err_z'};
Be_withLabels = array2table(Be,'RowNames',rowNames,'VariableNames',colNames)

%% Now create LQR
Q = diag([1000, 1000, ... % qe2, qe3  (roll, pitch)
          1, ...    % qe4  (yaw)
          0.1, 0.1, ... % omega_b_err_x, omega_b_err_y
          0.01, ...    % omega_b_err_z
         ]);
R = 0.05 * diag([1 1 1]); % torque outputs

[K, S, e] = lqr(Ae, Be, Q, R);
disp(regexprep( mat2str(K), {'\[', '\]', '\s+', '\;'}, {'', '', ',\t', ',\n'}))
disp(' ');

% Save gain
BalanceLQRgain = K;
save('generated/BalanceLQRgain', 'BalanceLQRgain');

%% Test gains
q = [1,0,0,0]';
dq = [0,0,0,0]';
xy = [0,0]';
dxy = [1,0]';

X = [xy;q;dxy;dq];

q_ref = eul2quat(deg2rad([0,0,1]),'ZYX')';
omega_ref = [0.1,0,0.3]';
tau = BalanceLQR(X,q_ref,omega_ref,  COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,  K)