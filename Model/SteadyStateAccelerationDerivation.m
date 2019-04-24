clear all;
scriptDir = fileparts(mfilename('fullpath'));
load(fullfile(scriptDir, 'generated/SymbolicModel.mat'));

assumeAlso(g, 'real');
assumeAlso(Mk, 'real');
assumeAlso(rk, 'real');
assumeAlso(Jw, 'real');
assumeAlso(Mw, 'real');
assumeAlso(rw, 'real');
assumeAlso(Mb, 'real');
assumeAlso(l, 'real');
assumeAlso(Bvk, 'real');
assumeAlso(Bvb, 'real');
assumeAlso(Bvm, 'real');
assumeAlso(Jbx, 'real');
assumeAlso(Jby, 'real');
assumeAlso(Jbz, 'real');
assumeAlso(Jk, 'real');
assumeAlso(COM_X, 'real');
assumeAlso(COM_Y, 'real');
assumeAlso(COM_Z, 'real');
assumeAlso(COM_Z, 'positive');

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

Beta = @(q,COM)(Phi(q)*Phi(vec*COM)*I_conj + Gamma(q)'*Gamma(vec*COM)); % from new derivation with shifted COM

%%
syms x y q1 q2 q3 q4 real;
syms dx dy dq1 dq2 dq3 dq4 real;
syms ddx ddy ddq1 ddq2 ddq3 ddq4 real;
q = [q1 q2 q3 q4]';
dq = [dq1 dq2 dq3 dq4]';
chi = [x y q1 q2 q3 q4]';
dchi = [dx dy dq1 dq2 dq3 dq4]';
ddchi = [ddx ddy ddq1 ddq2 ddq3 ddq4]';

% %% Generate simplication symbols and replace
% invM = inv(M_tilde);
% invM = SimplifyWithQuatConstraint(invM, q_);
% 
% %%
% C = subs(C_tilde, beta, 0);
% G = subs(G_tilde, beta, 0);
% D = subs(D_tilde, beta, 0);
% Q = Q_tilde;
% 
% 
% %% Generate closed-form non-linear symbolic model on the form
% % ddchi = f(chi,dchi) + g(chi) u
% f_cont = invM * (-C*dchi - G - D);
% g_cont = invM * Q;
% 
% %%
% % In steady state the attitude will be a constant value and dq and ddq will be zero
% % We thus solve for ddq = 0 to find equillibrium torque
% % Lets also ignore friction for now
% f_cont_steady = subs(f_cont, [dq;Bvm;Bvk;Bvb], [[0,0,0,0]'; 0;0;0]);
% g_cont_steady = subs(g_cont, [dq;Bvm;Bvk;Bvb], [[0,0,0,0]'; 0;0;0]);

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

%% Steady state torque - based on derivation on paper notes
W_bar = inv(-2*devec*W');

MA = M(1:2,1:2);
MB = M(1:2,3:6);
MB_bar = SimplifyWithQuatConstraint(W_bar * devec * Phi(q)' * MB', q);

Cq_xy = C(3:6,1:2);
Cq_xy = subs(Cq_xy, dq, [0,0,0,0]'); % set dq = 0 since we are interested in a steady state
C_bar = SimplifyWithQuatConstraint(W_bar * devec * Phi(q)' * Cq_xy, q) * [dx;dy];

Gq = G(3:6);
G_bar = SimplifyWithQuatConstraint(W_bar * devec * Phi(q)' * Gq, q);

L = 1/rk * [0,0,1,0; 0,-1,0,0] * Gamma(q)';

%%
Z = SimplifyWithQuatConstraint(MA - L*Phi(q)*W'*MB_bar, q);
invZ = SimplifyWithQuatConstraint(inv(Z), q);

%% Derive acceleration and jacobian
Acceleration = invZ * L*Phi(q)*W' * (C_bar + G_bar);
disp('Finished calculating acceleration equation');
Acceleration = SimplifyWithQuatConstraint( Acceleration, q );
disp('Finished simplifying acceleration equation');

% Save model
matlabFunction(Acceleration, 'file', 'generated/SteadyStateAcceleration', 'outputs', {'acceleration'});
disp('Finished saving acceleration model');

%% Simplify model for Jacobian derivation as it will other grow too big by including the wheel inertia
% The only difference is that it affects the Covariance propagation at direction changes
%Acceleration_simple = subs(Acceleration, Jw, 0);
%dAcceleration_dq = jacobian(Acceleration_simple, q);
%disp('Finished calculating acceleration dq');
%dAcceleration_dq = SimplifyWithQuatConstraint( dAcceleration_dq, q );
%disp('Finished simplifying acceleration dq');

%dAcceleration_dCOM = jacobian(Acceleration_simple, [COM_X;COM_Y]);
%disp('Finished calculating acceleration dCOM');
%dAcceleration_dCOM = SimplifyWithQuatConstraint( dAcceleration_dCOM, q );
%disp('Finished simplifying acceleration dCOM');

%% Save Jacobians
%matlabFunction(dAcceleration_dq, 'file', 'generated/SteadyStateAcceleration_dq', 'outputs', {'dAcceleration_dq'}); % , 'Optimize', false
%disp('Finished saving acceleration dq');
%matlabFunction(dAcceleration_dCOM, 'file', 'generated/SteadyStateAcceleration_dCOM', 'outputs', {'dAcceleration_dCOM'});
%disp('Finished saving acceleration dCOM');