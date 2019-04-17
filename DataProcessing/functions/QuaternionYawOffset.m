function [qOut, dqOut, omega_inertial_out] = QuaternionYawOffset(q, dq, yawOffset, preMultiply)
    % preMultiply defines whether yaw should be added in inertial (true) or body (false) frame
    % Should usually be set to true
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
    
    q_offset = eul2quat([yawOffset,0,0],'ZYX')';
    
    omega_body = zeros(size(q,1), 3);      
    omega_inertial_out = zeros(size(q,1), 3);      
    qOut = zeros(size(q,1), 4);  
    dqOut = zeros(size(q,1), 4);      
    for (m = 1:size(q,1))
        omega_body = 2 * devec * Phi(q(m,:)')' * dq(m,:)';  
        if (preMultiply)
            qOut(m,:) = ( Gamma(q(m,:)') * q_offset )'; % add yaw offset in inertial frame (as pre multiplication)
        else            
            qOut(m,:) = ( Phi(q(m,:)') * q_offset )'; % add yaw offset in body frame (as post multiplication)
        end
        dqOut(m,:) = ( 1/2 * Phi(qOut(m,:)') * vec * omega_body )';
        omega_inertial_out(m,:) = 2 * devec * Gamma(qOut(m,:)')' * dqOut(m,:)';      
    end  
end