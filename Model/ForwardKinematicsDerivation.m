% The following script is derived by hand (see paper notes)
clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../Parameters'));
Constants_Kugle; % load some constants (eg. alpha and gamma) but replace other with symbolic variables

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

% Physical constants
syms l rw rk real;

% States etc.
syms q1 q2 q3 q4 real;
q_ = [q1, q2, q3, q4]';

syms dq1 dq2 dq3 dq4 real;
dq_ = [dq1, dq2, dq3, dq4]';

%% motor mapping (inverse kinematics)
e1 = [1,0,0]';
e2 = [0,1,0]';
e3 = [0,0,1]';
R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1];
R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0];

W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1)
W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2)
W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3)

W = [W1;W2;W3];

J2 = simplify( W*Phi(q_)' * [1/rk * Gamma(q_) * [0,0,1,0]', -1/rk * Gamma(q_) * [0,1,0,0]', -2*eye(4)] );


%% forward kinematics
% from motor velocity and q, q_dot to translational velocity
W_tilde = W * [zeros(1,3); eye(3)];
syms dpsi1 dpsi2 dpsi3 real;
dpsi = [dpsi1, dpsi2, dpsi3]';
B_omeg_k_ = [0; inv(W_tilde) * dpsi];
K_omeg_k_ = SimplifyWithQuatConstraint(Phi(q_) * Gamma(q_)' * B_omeg_k_, q_) + SimplifyWithQuatConstraint(Phi(q_) * Gamma(q_)' * 2*Phi(q_)', q_) * dq_

trans_vel = rk * [0,0,1,0;0,-1,0,0] * K_omeg_k_
rot_vel = [0,0,0,1] * B_omeg_k_
matlabFunction(trans_vel, 'file', 'generated/ForwardKinematics', 'outputs', {'dxdy'}); % save forward kinematics

trans_vel_headingFrame = rk * [0,0,1,0;0,-1,0,0] * SimplifyWithQuatConstraint(Phi(q_)' * Gamma(q_) * K_omeg_k_, q_)
matlabFunction(trans_vel_headingFrame, 'file', 'generated/ForwardKinematicsInHeadingFrame', 'outputs', {'dxdy_h'}); % save forward kinematics in heading frame

matlabFunction([trans_vel;rot_vel], 'file', 'generated/ForwardKinematics_WithRot', 'outputs', {'dxdydyaw'}); % save forward kinematics

%% Forward kinematics for COM velocity
B_p_COM = [0, 0, l]'; % COM in Body frame
K_p_COM = devec * Phi(q_) * Gamma(q_)' * [0;B_p_COM];
K_dp_COM = devec * (Phi(dq_) * Gamma(q_)' + Phi(q_) * Gamma(dq_)') * [0;B_p_COM];

%I_p_COM = K_p_COM + [x;y;0];
I_dp_COM = K_dp_COM + [trans_vel;0];
 
trans_vel_com = [1,0,0; 0,1,0] * I_dp_COM
matlabFunction(trans_vel_com, 'file', 'generated/ForwardKinematics_COM', 'outputs', {'dxdy_com'}); % save forward kinematics

%% Forward kinematics for 2*L velocity
B_p_2L = [0, 0, 2*l]'; % COM in Body frame
K_p_2L = devec * Phi(q_) * Gamma(q_)' * [0;B_p_2L];
K_dp_2L = devec * (Phi(dq_) * Gamma(q_)' + Phi(q_) * Gamma(dq_)') * [0;B_p_2L];

I_dp_2L = K_dp_2L + [trans_vel;0];
 
trans_vel_2L = [1,0,0; 0,1,0] * I_dp_2L
matlabFunction(trans_vel_2L, 'file', 'generated/ForwardKinematics_2L', 'outputs', {'dxdy_2l'}); % save forward kinematics


%%
addpath(fullfile(pwd, 'generated'));
rk = 0.129;
rw = 0.05;

omeg = [0.1, 0.05, 0.2]';
q = eul2quat(deg2rad([0, 10, 5]), 'ZYX')';
dq = 1/2 * Phi(q) * [0;omeg];

dx = 0.4;
dy = 0.2;

dpsi = InverseKinematics(dq(1),dq(2),dq(3),dq(4),dx,dy,q(1),q(2),q(3),q(4),rk,rw)
dxdy = ForwardKinematics(dpsi(1),dpsi(2),dpsi(3),dq(1),dq(2),dq(3),dq(4),q(1),q(2),q(3),q(4),rk,rw)
