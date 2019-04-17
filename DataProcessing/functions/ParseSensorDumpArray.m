function out = ParseSensorDumpArray(array)
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

    if (size(array,2) ~= 33) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);    
    out.raw_accelerometer = array(:,2:4);
    out.raw_gyroscope = array(:,5:7);
    out.raw_magnetometer = array(:,8:10);
    out.encoder_angle = array(:,11:13);
    out.torque_delivered = array(:,14:16);    
    out.mti_q = array(:,17:20);
    out.mti_dq = array(:,21:24);
    out.mti_accelerometer = array(:,25:27);
    out.mti_gyroscope = array(:,28:30);
    out.mti_magnetometer = array(:,31:33);
    
    if (out.mti_q(1,1) < 0) % if scalar value is negative, then invert quaternion
        out.mti_q = -out.mti_q;
        out.mti_dq = -out.mti_dq;
    end
    
    eul_mti = quat2eul(out.mti_q, 'ZYX');
    out.mti_roll = eul_mti(:,3);
    out.mti_pitch = eul_mti(:,2);
    out.mti_yaw = unwrap(eul_mti(:,1));        
    
    out.mti_omega_body = zeros(size(array,1), 3);    
    for (m = 1:size(array,1))
        out.mti_omega_body(m,:) = (2*devec*Phi(out.mti_q(m,:)')'*out.mti_dq(m,:)')';
        out.mti_omega_inertial(m,:) = (2*devec*Gamma(out.mti_q(m,:)')'*out.mti_dq(m,:)')';
        x_vec = devec * Phi(out.mti_q(m,:)') * Gamma(out.mti_q(m,:)')' * [0,1,0,0]';
        out.mti_heading(m,:) = atan2(x_vec(2), x_vec(1));            
    end    
end