function dX = EvaluateNonlinearMPCmodel(t, X, u, constants, COM)
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
    
    % Extract constants
    Jk = constants(1);
    Mk = constants(2);
    rk = constants(3);
    Mb = constants(4);
    Jbx = constants(5);
    Jby = constants(6);
    Jbz = constants(7);
    Jw = constants(8);
    rw = constants(9);
    Bvk = constants(10);
    Bvm = constants(11);
    Bvb = constants(12);    
    g = constants(13);   	    
    COM_X = COM(1);
    COM_Y = COM(2);
    COM_Z = COM(3);
    
    % Extract states
    q1 = X(1);
    q2 = X(2);
    q3 = X(3);
    q4 = X(4);
    x = X(5);
    y = X(6);
    dx = X(7);
    dy = X(8);
    
    q = [q1,q2,q3,q4]';
    
    % Extract inputs
    omeg_ref_x = u(1);
    omeg_ref_y = u(2);
    omeg_ref_z = 0;
    if (length(u) > 2)   
        omeg_ref_z = u(3);
    end
    omeg_ref = [omeg_ref_x;omeg_ref_y;omeg_ref_z];
    
    % Non-linear ODE/model equations    
    %dq = 1/2 * Gamma(q) * vec * omeg_ref; % inertial frame omega_ref
    dq = 1/2 * Phi(q) * vec * omeg_ref; % body frame omega_ref    
        
    acceleration = SteadyStateAcceleration(COM_X,COM_Y,COM_Z,Jk,Jw,Mb,Mk,dx,dy,g,q1,q2,q3,q4,rk,rw);    
    ddx = acceleration(1);
    ddy = acceleration(2);
    
    dX = [dq; dx; dy; ddx; ddy];