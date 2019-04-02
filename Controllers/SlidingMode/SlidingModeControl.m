% Compute sliding mode control action
% Remember to add the folder with the generated model files:
%   addpath('../Model/generated');
% SlidingManifold  =>  0=q_dot inertial,  1=q_dot body,  2=Body angular velocity,  3=Inertial angular velocity
% SwitchingLaw  =>  1=discontinous,  2=continous
function [tau, S, q_err] = SlidingModeControl(X,q_ref,omega_ref,  COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g,  SlidingManifold,SwitchingLaw,K,eta,epsilon)

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
    
    sat = @(x) min(max(x, -1), 1); % saturation function definition

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

    M = mass(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,q1,q2,q3,q4,rk,rw);
    C = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
    G = gravity(COM_X,COM_Y,COM_Z,Mb,beta,g,q1,q2,q3,q4);
    D = friction(Bvb,Bvk,Bvm,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
    Q = input_forces(q1,q2,q3,q4,rk,rw);

    fq = [zeros(4,2), eye(4)] * (M \ (-C*dchi - G - D)); % M \ b = inv(M)*b
    gq = [zeros(4,2), eye(4)] * (M \ Q);

    if (SlidingManifold == 0 || SlidingManifold == 3)
        QuaternionErrorFrame = 0; % inertial frame
    else
        QuaternionErrorFrame = 1; % body frame
    end
    
    %% Quaternion error based on reference input
    if (QuaternionErrorFrame == 0) % inertial frame
        q_err = Gamma(q_ref)' * q; % quaternion error in inertial frame    
    elseif (QuaternionErrorFrame == 1) % body frame
        q_err = Phi(q_ref)' * q; % quaternion error in body frame    
    end

    if (q_err(1) < 0)
       q_err = -q_err; % inverse quaternion gives the shorter way to the same rotation
    end

    %% Recompute q_ref based on q_err (makes a difference if q_err was negated)
    if (QuaternionErrorFrame == 0) % inertial frame
        q_ref = Phi(q_err)' * q;
        dq_ref = 1/2 * Gamma(q_ref) * [0;omega_ref]; % inertial angular velocity
    elseif (QuaternionErrorFrame == 1) % body frame
        q_ref = Gamma(q_err)' * q;
        dq_ref = 1/2 * Phi(q_ref) * [0;omega_ref]; % body angular velocity
    end

    domega_ref = [0,0,0]'; % assumed constant or slowly varying omega ref

    %% Sliding manifold with q_dot and inertial error
    if (SlidingManifold == 0)        
        InputInv = inv(devec*Gamma(q_ref)'*gq);
        tau_eq = InputInv * ( -devec*Gamma(q_ref)'*fq - devec*2*Gamma(dq_ref)'*dq - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq );
        dq_err = Gamma(dq_ref)'*q + Gamma(q_ref)'*dq;
        S = devec*dq_err + K*devec*q_err;
    end

    if (SlidingManifold == 1)
        InputInv = inv(devec*Phi(q_ref)'*gq);
        tau_eq = InputInv * ( -devec*Phi(q_ref)'*fq - devec*2*Phi(dq_ref)'*dq - K*devec*Phi(dq_ref)'*q - K*devec*Phi(q_ref)'*dq );
        dq_err = Phi(dq_ref)'*q + Phi(q_ref)'*dq;
        S = devec*dq_err + K*devec*q_err;
    end

    %% Sliding manifold with omega    
    if (SlidingManifold == 2)
        % body angular velocity
        InputInv = inv(2 * devec*Phi(q)' * gq);
        tau_eq = InputInv * (-2*devec*Phi(dq)'*dq - 2*devec*Phi(q)'*fq + domega_ref - K*devec*Phi(dq_ref)'*q - K*devec*Phi(q_ref)'*dq);
        omeg = 2*devec*Phi(q)'*dq; % body angular velocity
        S = omeg - omega_ref + K*devec*q_err;
    end

    if (SlidingManifold == 3)
        % inertial angular velocity
        InputInv = inv(2 * devec*Gamma(q)' * gq);
        tau_eq = InputInv * (-2*devec*Gamma(dq)'*dq - 2*devec*Gamma(q)'*fq + domega_ref - K*devec*Gamma(dq_ref)'*q - K*devec*Gamma(q_ref)'*dq);
        omeg = 2*devec*Gamma(q)'*dq; % inertial angular velocity
        S = omeg - omega_ref + K*devec*q_err;
    end
    
    %% Discontinous switching law
    if (SwitchingLaw == 1)
        sgnS = sign(S);
        sgnS = sgnS + (sgnS==0); % force S >= 0 to cause positive input
        u = -eta .* sgnS;
        tau_switching = InputInv * u;
    end

    %% Continous switching law
    if (SwitchingLaw == 2)
        satS = sat(S./epsilon);
        u = -eta .* satS;
        tau_switching = InputInv * u;
    end

    %% Control output
    tau = tau_eq + tau_switching;