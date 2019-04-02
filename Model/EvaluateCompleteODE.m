% Evaluate the non-linear ODE by using the generated Euler-Lagrange
% matrices. To run the function remember to add the "generated" folder to
% the MATLAB path: addpath('generated');
% dX returns the derivative of the complete state vector:
% X = [x; y; q; dx; dy; dq]
function dX = EvaluateCompleteODE(X, tau, COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,Bvb,Bvk,Bvm,rk,rw,g)    
    x = X(1);
    y = X(2);    
    q = X(3:6); q1 = X(3); q2 = X(4); q3 = X(5); q4 = X(6);
    dx = X(7);
    dy = X(8);
    dq = X(9:12); dq1 = X(9); dq2 = X(10); dq3 = X(11); dq4 = X(12);
    
    chi = X(1:6);
    dchi = X(7:12);
    
    %% Calculate non-linear dynamics given constants (and without regularization, hence beta=0)
    beta = 0;
    M_tilde = mass(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jk,Jw,Mb,Mk,q1,q2,q3,q4,rk,rw);
    C_tilde = coriolis(COM_X,COM_Y,COM_Z,Jbx,Jby,Jbz,Jw,Mb,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
    G_tilde = gravity(COM_X,COM_Y,COM_Z,Mb,beta,g,q1,q2,q3,q4);
    D_tilde = friction(Bvb,Bvk,Bvm,beta,dq1,dq2,dq3,dq4,dx,dy,q1,q2,q3,q4,rk,rw);
    Q_tilde = input_forces(q1,q2,q3,q4,rk,rw);
    
    %% Compute Non-linear ODE
    f = M_tilde \ ( - C_tilde*dchi - G_tilde - D_tilde );
    g = M_tilde \ Q_tilde;
    ddchi = f + g*tau;
    
    dX = [dchi; ddchi];