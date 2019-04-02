clear all;
addpath('../Parameters');

% Parameters
syms g positive % gravity
syms Jk Mk rk Jw Mw rw Jb Mb l positive;
syms Bvk Bvb Bvm positive; % friction coefficients

% State variables
syms phi(t) dphi(t) % ball position
syms theta(t) dtheta(t) % ball position
dphi = diff(phi);
ddphi = diff(dphi);
dtheta = diff(theta);
ddtheta = diff(dtheta);
assumeAlso(phi(t), 'real');
assumeAlso(dphi(t), 'real');
assumeAlso(ddphi(t), 'real');
assumeAlso(theta(t), 'real');
assumeAlso(dtheta(t), 'real');
assumeAlso(ddtheta(t), 'real');

% Generalized coordinates
chi = [phi theta]';
dchi = [dphi; dtheta];
ddchi = [ddphi; ddtheta];
   
% Rolling constraint
xk = phi * rk;
dxk = diff(xk);

%% Kinematics
gVec = [0; -g];
bPc = [0; rk]; % Contact point in body frame centered in ball
bPm = [0; l]; % CoM of body in body frame centered in ball
iPk = [xk; 0]; % Center of ball in inertial frame
iPm = iPk + l*[sin(theta); cos(theta)]; % Center of body mass in inertial frame
diPm = diff(iPm);

% Omniwheel velocity
dpsi = rk/rw * (dphi - dtheta);  % in ETH report they state this as  "rk/rw * (dphi - dtheta) - dtheta",  but I disagree

%% Energy definitions
% Kugle
Tk = simplify( 1/2*Mk*dxk^2 + 1/2*Jk*dphi^2 );
Vk = 0;

% Body
Tb = simplify( 1/2 * Jb * dtheta^2 + 1/2 * Mb * diPm' * diPm );
Vb = simplify( -Mb * gVec' * iPm );

% Omniwheel
Tw = simplify( 1/2 * Jw * dpsi^2 );
Vw = 0; % no potential energy

%% Find Equations of motion
% Lagrangian
L = simplify( Tk + Tb + Tw - Vk - Vb - Vw );
%L = simplify( Tk + Tb - Vb );

syms phi theta real;
syms dphi dtheta real;
syms ddphi ddtheta real;
chi_ = [phi theta]';
dchi_ = [dphi dtheta]';
ddchi_ = [ddphi ddtheta]';
L2 = subs(L, ddchi, ddchi_);
L2 = subs(L2, dchi, dchi_);
L2 = subs(L2, chi, chi_);
L2 = L2(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t

%% Derive equations of motion
syms dphi_(t) dtheta_(t)
assumeAlso(dphi_(t), 'real');
assumeAlso(dtheta_(t), 'real');
dchi_ = [dphi_, dtheta_]';
dLdchidot = simplify( subs(functionalDerivative(subs(L, dchi, dchi_), dchi_), dchi_, dchi) );
dLdchi = simplify( subs(functionalDerivative(subs(L, dchi, dchi_), chi), dchi_, dchi) );
ddt_dLdchidot = simplify( diff(dLdchidot) );

eq = ddt_dLdchidot - dLdchi == zeros(2,1);%Q % differential equation
dyneq = expand(ddt_dLdchidot - dLdchi);

% Solve for ODE functions
syms theta phi real;
syms dtheta dphi real;
syms ddtheta ddphi real;
chi_ = [phi theta]';
dchi_ = [dphi dtheta]';
ddchi_ = [ddphi ddtheta]';
dyneq2 = subs(dyneq, ddchi, ddchi_);
dyneq2 = subs(dyneq2, dchi, dchi_);
dyneq2 = subs(dyneq2, chi, chi_);
%eq2 = subs(eq2, tau, tau_in);
dyneq2 = dyneq2(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t

%% Input mapping
syms tau real
dpsi = subs(dpsi, dchi, dchi_);
dpsi = subs(dpsi, chi, chi_);
dpsi = dpsi(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t
J = simplify( jacobian(dpsi, dchi_) );
matlabFunction(simplify(dpsi), 'file', 'generated/PlanarInverseKinematics', 'outputs', {'dpsi'}); % save inverse kinematics

Q = simplify( J' * tau ); % generalized input forces

%% Friction forces
% Viscous friction for Ball-ground contact
dxk = subs(dxk, dchi, dchi_);
dxk = subs(dxk, chi, chi_);
dxk = dxk(0); % there should be no time dependence, so we evaluate at t=0 just to make MATLAB happy since it is independent of t
D_vk = Bvk * [dxk, 0]';

% Viscous friction in all motors
D_vm = J' * Bvm * dpsi; % (Kvm * dpsi) gives 3 friction torques affecting the motors, then J' maps it to generalized coordinates

% Viscous friction on body velocity (air friction)
D_vb = Bvb * [0, dtheta]';

D = simplify( D_vk + D_vm + D_vb);


%% Find Mass matrix
np = 2;
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


%% Save matrices
matlabFunction(M, 'file', 'generated/planar_mass', 'outputs', {'M'});
matlabFunction(C, 'file', 'generated/planar_coriolis', 'outputs', {'C'});
matlabFunction(G, 'file', 'generated/planar_gravity', 'outputs', {'G'});
matlabFunction(Q, 'file', 'generated/planar_input_forces', 'outputs', {'Q'});
matlabFunction(D, 'file', 'generated/planar_friction', 'outputs', {'D'});

%% Linearize system
% Normal method of linearizing system, now that the system is small enough
% such that we can find the symbolic inverse of the mass matrix
syms phi theta real;
syms dphi dtheta real;
syms ddphi ddtheta real;
chi_ = [phi, theta]';
dchi_ = [dphi, dtheta]';

x = [chi_; dchi_];
dx = [dchi_; inv(M)*(Q - C*chi_ - G - D)];

A = simplify( jacobian(dx, x) )
disp('Saving A matrix'); 
matlabFunction(A, 'file', 'generated/planar_A_matrix', 'outputs', {'A'}); 

B = simplify( jacobian(dx, tau) )
disp('Saving B matrix'); 
matlabFunction(B, 'file', 'generated/planar_B_matrix', 'outputs', {'B'}); 

%% See if we can ressemble system equations
simplify( dyneq2 == M*ddchi_ + C*dchi_ + G )