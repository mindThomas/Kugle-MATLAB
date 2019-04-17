% Compute balance LQR control action
% Remember to add the folder with the generated model files:
%   addpath('../../Model/generated');
% Note that 'omega_b_ref' is angular velocity given in body frame
function [tau, q_err] = BalanceLQR(X,q_ref,omega_b_ref,  COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,  K)

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
    
    beta = 0; % regularization of the model is only necessary for the simulated ODE, hence beta=0

    x = X(1);
    y = X(2);
    q1 = X(3);
    q2 = X(4);
    q3 = X(5);
    q4 = X(6);

    dx = X(7);
    dy = X(8);
    dq1 = X(9);
    dq2 = X(10);
    dq3 = X(11);
    dq4 = X(12);

    q = X(3:6);
    dq = X(9:12);
    dxy = X(7:8);
    dchi = reshape(X(7:12), 6, 1);
    
	
	%% Compute error state vector
    % Quaternion error based on reference input
	q_err = Phi(q_ref)' * q; % quaternion error in body frame        

    if (q_err(1) < 0)
       q_err = -q_err; % inverse quaternion gives the shorter way to the same rotation
    end

    % Recompute q_ref based on q_err (makes a difference if q_err was negated)
    q_ref = Gamma(q_err)' * q;
    dq_ref = 1/2 * Phi(q_ref) * [0;omega_b_ref]; % body angular velocity    

    domega_ref = [0,0,0]'; % assumed constant or slowly varying omega ref

	omega_b = 2*devec * Phi(q)' * dq;
	omega_b_err = omega_b - omega_b_ref;
    
	Xe = [q_err(2:4); % extract vector part of quaternion error
		  omega_b_err];
    
	
    %% Compute steady state torque
    % Define steady-state states
	qr1 = q_ref(1);
	qr2 = q_ref(2);
	qr3 = q_ref(3);
	qr4 = q_ref(4);
	dqr1 = dq_ref(1);
	dqr2 = dq_ref(2);
	dqr3 = dq_ref(3);
	dqr4 = dq_ref(4);
	dxr = dx;
	dyr = dy;
	
	% Compute steady-state nominal dynamics
	M_ss = mass(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,qr1,qr2,qr3,qr4,rk,rw);
	C_ss = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dqr1,dqr2,dqr3,dqr4,dxr,dyr,qr1,qr2,qr3,qr4,rk,rw);
	G_ss = gravity(COM_X,COM_Y,COM_Z,Mb,beta,g,qr1,qr2,qr3,qr4);
	D_ss = friction(Bvb,Bvk,Bvm,beta,dqr1,dqr2,dqr3,dqr4,dxr,dyr,qr1,qr2,qr3,qr4,rk,rw);
	Q_ss = input_forces(qr1,qr2,qr3,qr4,rk,rw);

    fq_ss = [zeros(4,2), eye(4)] * (M_ss \ (-C_ss*dchi - G_ss - D_ss)); % M_ss \ b = inv(M_ss)*b
    gq_ss = [zeros(4,2), eye(4)] * (M_ss \ Q_ss);
	
	% Compute the steady state torque
	tau_ss = -inv(devec*Phi(q_ref)'*gq_ss) * (devec*Phi(q_ref)'*fq_ss);
	
	
    %% Compute LQR control output
	tau_lqr = -K*Xe;
	
	
    %% Control output
    tau = tau_ss + tau_lqr;