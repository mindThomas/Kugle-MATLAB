%% OBS. This model derivation script uses q1 to q4 as the quaternion elements, where the report uses q0 to q3.
% Note that the scalar element of the quaternion in this derivative is q1, where it is q0 in the report.

clear all;
addpath('../Parameters');
Constants_Kugle; % load some constants (eg. alpha and gamma defining the wheel contact points) but replace other with symbolic variables

% When simplifying consider the option "'IgnoreAnalyticConstraints', true"

%% Parameters
syms g positive % gravity
syms alpha gamma positive;
syms Mk rk Jw Mw rw Mb l positive;
syms COM_X COM_Y real;
syms COM_Z positive;
syms Bvk Bvb Bvm positive; % friction coefficients

assumeAlso(g, 'real');
assumeAlso(alpha, 'real');
assumeAlso(gamma, 'real');
assumeAlso(Mk, 'real');
assumeAlso(rk, 'real');
assumeAlso(Jw, 'real');
assumeAlso(Mw, 'real');
assumeAlso(rw, 'real');
assumeAlso(Mb, 'real');
assumeAlso(l, 'real');
assumeAlso(COM_Z, 'real');
assumeAlso(Bvk, 'real');
assumeAlso(Bvb, 'real');
assumeAlso(Bvm, 'real');

% Inertia matrices are assumed to be diagonal since the moment of inertia
% is defined along the principal axes of the body
%syms Jkx Jky Jkz real;
%Jk = diag([Jkx Jky Jkz]);
syms Jk positive
assumeAlso(Jk, 'real');
Jk_ = diag([Jk Jk Jk]); % for a sphere the inertia tensor is diagonal and each element are equal
syms Jbx Jby Jbz positive;
assumeAlso(Jbx, 'real');
assumeAlso(Jby, 'real');
assumeAlso(Jbz, 'real');
Jb = diag([Jbx Jby Jbz]);
%Jk = sym('Jk', [3,3], 'real');
%Jk = tril(Jk,0) + tril(Jk,-1).'; % make symmetric
%Jb = sym('Jb', [3,3], 'real');
%Jb = tril(Jb,0) + tril(Jb,-1).'; % make symmetric

%% State variables
syms x(t) dx(t) % ball position
syms y(t) dy(t) % ball position
q=sym([]);
for i=1:4
   syms(sprintf('q%d(t)', i)) %declare each element in the array as a single symbolic function
   qq = symfun(str2sym(sprintf('q%d(t)', i)), t); %declare each element to a symbolic "handle"
   eval(sprintf('assumeAlso(q%d(t), ''real'');', i));
   q = [q;qq]; %paste the symbolic "handle" into an array 
end
dq=sym([]);
for i=1:4
   syms(sprintf('dq%d(t)', i)) %declare each element in the array as a single symbolic function
   qq = symfun(str2sym(sprintf('dq%d(t)', i)), t); %declare each element to a symbolic "handle"
   eval(sprintf('assumeAlso(dq%d(t), ''real'');', i));
   dq = [dq;qq]; %paste the symbolic "handle" into an array 
end
ddq=sym([]);
for i=1:4
   syms(sprintf('ddq%d(t)', i)) %declare each element in the array as a single symbolic function
   qq = symfun(str2sym(sprintf('ddq%d(t)', i)), t); %declare each element to a symbolic "handle"
   eval(sprintf('assumeAlso(ddq%d(t), ''real'');', i));
   ddq = [ddq;qq]; %paste the symbolic "handle" into an array 
end
dx = diff(x);
ddx = diff(dx);
dy = diff(y);
ddy = diff(dy);
dq = diff(q);
ddq = diff(dq);
assumeAlso(x(t), 'real');
assumeAlso(dx(t), 'real');
assumeAlso(ddx(t), 'real');
assumeAlso(y(t), 'real');
assumeAlso(dy(t), 'real');
assumeAlso(ddy(t), 'real');
assumeAlso(dq(t), 'real');
assumeAlso(ddq(t), 'real');

% Generalized coordinates
chi = [x, y, q1, q2, q3, q4]';
dchi = [dx; dy; dq];
ddchi = [ddx; ddy; ddq];

%% Quaternion specifics
Jb_ = [1 zeros(1,3);
       zeros(3,1) Jb];    

% Quaternion to matrix operators
fq = @(q,i)[i==1, i==2, i==3, i==4]*q; % to extract element from quaternion that works also with symbolic definitions
Phi = @(q)[fq(q,1) -fq(q,2) -fq(q,3) -fq(q,4);     % for q o p = Phi(q) * p
          fq(q,2) fq(q,1)  -fq(q,4) fq(q,3);
          fq(q,3) fq(q,4)  fq(q,1)  -fq(q,2);
          fq(q,4) -fq(q,3) fq(q,2)  fq(q,1)];           
Gamma = @(p)[fq(p,1) -fq(p,2) -fq(p,3) -fq(p,4);   % for q o p = Gamma(p) * q
             fq(p,2) fq(p,1) fq(p,4) -fq(p,3);
             fq(p,3) -fq(p,4) fq(p,1) fq(p,2);
             fq(p,4) fq(p,3) -fq(p,2) fq(p,1)];           
Conj = @(q)[fq(q,1);-fq(q,2);-fq(q,3);-fq(q,4)]; 
ConjMat = diag([1,-1,-1,-1]);
qVectorize = @(v)[zeros(1,3); eye(3,3)]*v;
qDevectorize = @(q)[zeros(3,1), eye(3,3)]*q;
        
   
%% Kinematics
omeg_k = 1/rk * [-dy, dx, 0]'; % angular velocity of ball along inertial axes
omeg_b = qDevectorize(2 * Phi(q)' * dq); % angular velocity of body along body axes

%B_p_COM = [0, 0, l]'; % COM in Body frame
B_p_COM = [COM_X, COM_Y, COM_Z]'; % COM in Body frame
%K_p_COM = Phi(q) * Phi([0;B_p_COM]) * Conj(q);
K_p_COM = qDevectorize( Phi(q) * Gamma(q)' * qVectorize(B_p_COM) );

I_p_COM = K_p_COM + [x;y;0];
I_dp_COM = diff(I_p_COM);

g_vec = [0; 0; -g];

B_omeg_k = qDevectorize( Phi(q)' * Gamma(q) * qVectorize(omeg_k) ) - omeg_b;

for (i = 1:3)
    B_p_c = rk * [cos(alpha) * cos(gamma*(i-1));
                  cos(alpha) * sin(gamma*(i-1));
                  sin(alpha)];
    eval(sprintf('B_p_c%d = B_p_c;', i));
    
    B_d_m = [-sin(gamma*(i-1));
             cos(gamma*(i-1));
             0];
    eval(sprintf('B_d_m%d = B_d_m;', i));    
end
B_v_c1 = cross(B_omeg_k, B_p_c1);
B_v_c2 = cross(B_omeg_k, B_p_c2);
B_v_c3 = cross(B_omeg_k, B_p_c3);

dpsi1 = 1/rw * B_v_c1' * B_d_m1;
dpsi2 = 1/rw * B_v_c2' * B_d_m2;
dpsi3 = 1/rw * B_v_c3' * B_d_m3;

%% Energy definitions
% Kugle
Tk = simplify( 1/2*Mk*dx^2 + 1/2*Mk*dy^2 + 1/2*omeg_k'*Jk_*omeg_k );
Vk = 0;

% Body
Tb = simplify( 1/2 * omeg_b' * Jb * omeg_b + 1/2 * Mb * (I_dp_COM' * I_dp_COM) );
Vb = simplify( -Mb * g_vec' * K_p_COM );

% Omniwheel
Tw1 = simplify( 1/2 * Jw * dpsi1^2 );
Tw2 = simplify( 1/2 * Jw * dpsi2^2 );
Tw3 = simplify( 1/2 * Jw * dpsi3^2 );
Vw = 0; % no potential energy

%% Find Equations of motion
% Lagrangian
L = simplify( Tk + Tb + Tw1 + Tw2 + Tw3 - Vk - Vb - Vw );
%L = simplify( Tk + Tb - Vb );

syms x y q1 q2 q3 q4 real;
syms dx dy dq1 dq2 dq3 dq4 real;
syms ddx ddy ddq1 ddq2 ddq3 ddq4 real;
chi_ = [x y q1 q2 q3 q4]';
dchi_ = [dx dy dq1 dq2 dq3 dq4]';
ddchi_ = [ddx ddy ddq1 ddq2 ddq3 ddq4]';
L2 = subs(L, ddchi, ddchi_);
L2 = subs(L2, dchi, dchi_);
L2 = subs(L2, chi, chi_);
%eq2 = subs(eq2, tau, tau_in);
L2 = L2(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t

% L2 = subs(L2, q1^2 + q2^2 + q3^2 + q4^2, 1) % q1^2 + q2^2 + q3^2 + q4^2  ==  1
% L2 = subs(L2, q1^2 - q2^2 - q3^2 + q4^2, 1 - 2*q2^2 - 2*q3^2) % q1^2 - q2^2 - q3^2 + q4^2   ==   1 - 2*q2^2 - 2*q3^2
% L2 = subs(L2, q1^2 + q2^2, 1 - q3^2 - q4^2) % q1^2 + q2^2 ==  1 - q3^2 - q4^2
% 
% L2 = subs(L2, dq1^2 + dq2^2 + dq3^2 + dq4^2, 1) % q1^2 + q2^2 + q3^2 + q4^2  ==  1
% L2 = subs(L2, dq1^2 - dq2^2 - dq3^2 + dq4^2, 1 - 2*dq2^2 - 2*dq3^2) % q1^2 - q2^2 - q3^2 + q4^2   ==   1 - 2*q2^2 - 2*q3^2
% L2 = subs(L2, dq1^2 + dq2^2, 1 - dq3^2 - dq4^2) % q1^2 + q2^2 ==  1 - q3^2 - q4^2

%return;

%% Derive equations of motion
syms dx_(t) dy_(t) dq1_(t) dq2_(t) dq3_(t) dq4_(t)
assumeAlso(dx_(t), 'real');
assumeAlso(dy_(t), 'real');
assumeAlso(dq1_(t), 'real');
assumeAlso(dq2_(t), 'real');
assumeAlso(dq3_(t), 'real');
assumeAlso(dq4_(t), 'real');
dchi_ = [dx_, dy_, dq1_, dq2_, dq3_, dq4_]';
dLdchidot = simplify( subs(functionalDerivative(subs(L, dchi, dchi_), dchi_), dchi_, dchi) );
dLdchi = simplify( subs(functionalDerivative(subs(L, dchi, dchi_), chi), dchi_, dchi) );
ddt_dLdchidot = simplify( diff(dLdchidot) );

eq = ddt_dLdchidot - dLdchi == zeros(6,1);%Q % differential equation
dyneq = expand(ddt_dLdchidot - dLdchi);

% Solve for ODE functions
syms x y q1 q2 q3 q4 real;
syms dx dy dq1 dq2 dq3 dq4 real;
syms ddx ddy ddq1 ddq2 ddq3 ddq4 real;
chi_ = [x y q1 q2 q3 q4]';
dchi_ = [dx dy dq1 dq2 dq3 dq4]';
ddchi_ = [ddx ddy ddq1 ddq2 ddq3 ddq4]';
dyneq2 = subs(dyneq, ddchi, ddchi_);
dyneq2 = subs(dyneq2, dchi, dchi_);
dyneq2 = subs(dyneq2, chi, chi_);
%eq2 = subs(eq2, tau, tau_in);
dyneq2 = dyneq2(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t

%% Input mapping
syms tau1 tau2 tau3 real
tau = [tau1, tau2, tau3]';
dpsi = [dpsi1, dpsi2, dpsi3]'; % motor axes in order matching inputs (torque)
dpsi = subs(dpsi, dchi, dchi_);
dpsi = subs(dpsi, chi, chi_);
dpsi = dpsi(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t
J = simplify( jacobian(dpsi, dchi_) );
matlabFunction(simplify(dpsi), 'file', 'generated/InverseKinematics', 'outputs', {'dpsi'}); % save inverse kinematics

%TransVelToTau = subs(J, [q1 q2 q3 q4], [1 0 0 0]) * [dx dy 0 0 0 0]';
%matlabFunction(TransVelToTau, 'file', 'generated/TransVelToTau', 'outputs', {'tau'}); % function for generating test torque inputs based on desired translational velocity

%TransVelToTau = J * [dx dy 0 0 0 0]';
%matlabFunction(TransVelToTau, 'file', 'generated/TransVelToTau', 'outputs', {'tau'}); % function for generating test torque inputs based on desired translational velocity

Q = simplify( J' ); % generalized input forces

%% Friction forces
% Viscous friction for Ball-ground contact
D_vk = Bvk * [dx dy 0 0 0 0]';

% Viscous friction in all motors
D_vm = J' * Bvm * dpsi; % (Kvm * dpsi) gives 3 friction torques affecting the motors, then J' maps it to generalized coordinates

% Viscous friction on body velocity (air friction)
omeg_b2 = subs(omeg_b, dchi, dchi_);
omeg_b2 = subs(omeg_b2, chi, chi_);
omeg_b2 = omeg_b2(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t
J_omeg_b = simplify( jacobian(omeg_b2, dchi_) );
D_vb = J_omeg_b' * Bvb * omeg_b2;

D = simplify( D_vk + D_vm + D_vb);


%% Find Mass matrix
np = 6;
M = sym([]);
for (m = 1:np)
    for (n = 1:np)
        z = zeros(1,np);
        z(m) = 1;
        tmp = coeffs(z*collect(dyneq2, ddchi_), ddchi_(n), 'all');
        if (length(tmp) < 2)
            tmp = zeros(1,2);
        end       
        M(m,n) = [1 0]*tmp'; % take end element
    end
end

M


%% Find Coriolis matrix
dyneq3 = simplify( dyneq2 - M*ddchi_ ); % remaining equation part after Mass matrix has been split

C = sym([]);
for (m = 1:np)
    for (n = 1:np)
        C(m,n) = 0;
    end
end

for (i = 1:2)
    for (m = 1:np)
        for (n = 1:np)
            z = zeros(1,np);
            z(m) = 1;
            tmp = coeffs(z*collect(dyneq3, dchi_), dchi_(n), 'all');
            if (length(tmp) == 2)
                c = [1 0]*tmp';
            elseif (length(tmp) == 3)
                c = [1 0 0]*tmp' * dchi_(n);
            else
                c = 0;
            end      

            C(m,n) = C(m,n) + c;
            dyneq3 = dyneq3 - z'*c*dchi_(n);
        end
    end

    dyneq3 = simplify(dyneq3);
    C = simplify(C);
end

C


%% Find Gravity matrix
dyneq4 = simplify( dyneq2 - M*ddchi_ - C*dchi_ ); % remaining equation part after Coriolis matrix has been split

G = dyneq4


%% See if we can ressemble system equations (check if the system is correct)
%simplify( dyneq2 == M*ddchi_ + C*dchi_ + G + D)

%% Insert Quaternion constraint
syms x y q1 q2 q3 q4 real;
syms dx dy dq1 dq2 dq3 dq4 real;
syms ddx ddy ddq1 ddq2 ddq3 ddq4 real;
q_ = [q1 q2 q3 q4]';
dq_ = [dq1 dq2 dq3 dq4]';
ddq_ = [ddq1 ddq2 ddq3 ddq4]';

% We would now have a system on the form
%   M*ddchi + C*dchi + G + D = 0_6
% However we need to add a Lagrange multiplier to fulfil the quaternion constraint
%   M*ddchi + C*dchi + G + D - lambda*2*[0;0;q] = 0_6
% We want to remove the lagrange multipliers, and we can do that by multiplying with the conjugate quaternion and removing the scalar equation:
    H = [zeros(3,1) eye(3)] * Phi(q_)';
% But since we have the two top elements (x and y) that we still want to go through we append the identity to the top left corner
    H_ = [eye(2) zeros(2,4); zeros(3,2) H];
% Since multiplying the lagrange multiplier part with this will give zero
%   H_*lambda*2*[0;0;q] = 0_5
% If we multply both sides of the differential equation with this matrix we get
%   H_*M*ddchi + H_*C*dchi + H_*G + H_*D - H_*lambda*2*[0;0;q] = H_*0_6
%   H_*M*ddchi + H_*C*dchi + H_*G + H_*D = 0_5
% Or with generalized input forces/torques (and friction) on the right hand side
%   H_*M*ddchi + H_*C*dchi + H_*G + H_*D - H_*lambda*2*[0;0;q] = H_*Q*tau
%   H_*M*ddchi + H_*C*dchi + H_*G + H_*D = H_*Q*tau

% We then add the quaternion constraint as a differential equation
%   -q'*ddq = r(q,dq)
% Where r(q,dq) is defined as
syms beta;
r = dq_'*dq_ + 2*beta*q_'*dq_ + 1/2*beta^2*(q_'*q_ - 1);

% We therefore augment our system of differential equations to become
%   M_tilde*ddchi + C_tilde*dchi + G_tilde + D_tilde = Q_hat
% M_tilde = [H_*M; zeros(1,2), -q_'];
% C_tilde = [H_*C; zeros(1,2), -dq_' - 2*beta*q_'];
% G_tilde = [H_*G; -(1/2)*beta^2*(q_'*q_-1)];
% Q_tilde = [H_*Q; 0];
% D_tilde = [H_*D; 0];

TT = diag([0 0 1 1 1 1]);

M = SimplifyWithQuatConstraint(M, q_);
C = SimplifyWithQuatConstraint(C, q_);
G = SimplifyWithQuatConstraint(G, q_);
D = SimplifyWithQuatConstraint(D, q_);
Q = SimplifyWithQuatConstraint(Q, q_);

M_tilde = [H_*M; chi_'*TT]; % OBS! This can be simplified a lot more since Matlab can not figure out how to reduce/simplify this properly due to the quaternion constraint not being enforced: q1^2+q2^2+q3^2+q4^2 = 1
C_tilde = [H_*C; dchi_'*TT + beta*chi_'*TT];
G_tilde = [H_*G; 1/2*beta*chi_'*TT*chi_];
D_tilde = [H_*D; -1/2*beta];
Q_tilde = [H_*Q; zeros(1,3)];

% OBS. Its' important not to Quaternion simplify G_tilde, since this includes one of
% the norm-preserving regularization terms (hence if beta is intended to be used)

%% Save model and generated symbolic function files (used in simulation, MATLAB coder etc.)
save('SymbolicModel.mat');
matlabFunction(M_tilde, 'file', 'generated/mass', 'outputs', {'M'});
matlabFunction(C_tilde, 'file', 'generated/coriolis', 'outputs', {'C'});
matlabFunction(G_tilde, 'file', 'generated/gravity', 'outputs', {'G'});
matlabFunction(Q_tilde, 'file', 'generated/input_forces', 'outputs', {'Q'});
matlabFunction(D_tilde, 'file', 'generated/friction', 'outputs', {'D'});