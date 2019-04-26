function dX = LinearMPCmodel(t, X, u, AccelerationConstant_q3_to_ddx, AccelerationConstant_q2_to_ddy)   
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
    
    % Non-linear ODE/model equations with omega_ref given in body frame 
    dq2 = 1/2 * omeg_ref_x;
    dq3 = 1/2 * omeg_ref_y;
    
    % OBS. The linear MPC model does not include yaw turning
    dq = [0,dq2,dq3,0]';
            
    ddx = AccelerationConstant_q3_to_ddx*q3;
    ddy = AccelerationConstant_q2_to_ddy*q2;
    
    dX = [dq; dx; dy; ddx; ddy];