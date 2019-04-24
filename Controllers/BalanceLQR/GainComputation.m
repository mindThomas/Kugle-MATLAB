scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Model/generated'));
addpath(fullfile(scriptDir, '../../Model/functions'));
addpath(fullfile(scriptDir, '../../Parameters'));
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

%% Symbols definition
syms tau1 tau2 tau3 real;
syms x y q1 q2 q3 q4 real;
syms dx dy dq1 dq2 dq3 dq4 real;
syms ddx ddy ddq1 ddq2 ddq3 ddq4 real;
chi = [x y q1 q2 q3 q4]';
dchi = [dx dy dq1 dq2 dq3 dq4]';
ddchi = [ddx ddy ddq1 ddq2 ddq3 ddq4]';
q = [q1 q2 q3 q4]';
dq = [dq1 dq2 dq3 dq4]';
ddq = [ddq1 ddq2 ddq3 ddq4]';
tau = [tau1 tau2 tau3]';

%% Calculate non-linear dynamics given constants (and without regularization, hence beta=0)
beta = 0;
M_tilde = mass(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,q1,q2,q3,q4,rk,rw);
C_tilde = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
G_tilde = gravity(COM_X,COM_Y,COM_Z,Mb,beta,g,q1,q2,q3,q4);
D_tilde = friction(Bvb,Bvk,Bvm,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
Q_tilde = input_forces(q1,q2,q3,q4,rk,rw);

% Substitute constants into matrix
M_tilde2 = SimplifyWithQuatConstraint(subs(M_tilde), q);
C_tilde2 = SimplifyWithQuatConstraint(subs(C_tilde), q);
G_tilde2 = SimplifyWithQuatConstraint(subs(G_tilde), q);
D_tilde2 = SimplifyWithQuatConstraint(subs(D_tilde), q);
Q_tilde2 = SimplifyWithQuatConstraint(subs(Q_tilde), q);

%% Non-linear ODE
M_tilde2_inv = inv(M_tilde2);

% Extract quaternion dynamics (q_dotdot)
fq = [zeros(4,2), eye(4)] * M_tilde2_inv * ( - C_tilde2*dchi - G_tilde2 - D_tilde2 );
gq = [zeros(4,2), eye(4)] * M_tilde2_inv * Q_tilde2;
ddq_ODE = fq + gq*tau;

%% Derive/compute linearized matrices (based on OneNote derivation)
A11 = zeros(3,3);
A12 = 1/2 * eye(3);
A21 = 2*devec * subs( jacobian(ddq_ODE, q), [q;dq;tau], [[1,0,0,0]';[0,0,0,0]';[0,0,0]']) * vec;
A22 = devec * subs( jacobian(ddq_ODE, dq), [q;dq;tau], [[1,0,0,0]';[0,0,0,0]';[0,0,0]']) * vec;

Ae = vpa([A11, A12;
         A21, A22]);
Be = [zeros(3,3);
     2*devec * vpa(eval(subs(gq, [q;dq], [[1,0,0,0]';[0,0,0,0]'])))];

dx0 = 0;
dy0 = 0;
Ae = eval(subs(Ae, [dx;dy], [dx0;dy0]));
Be = eval(subs(Be, [dx;dy], [dx0;dy0]));

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