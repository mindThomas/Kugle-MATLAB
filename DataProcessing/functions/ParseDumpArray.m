function out = ParseDumpArray(array)
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
    
    if (size(array,2) ~= 50) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);
    out.accelerometer = array(:,2:4);
    out.gyroscope = array(:,5:7);
    out.magnetometer = array(:,8:10);        
    out.position = array(:,11:12);
    out.q = array(:,13:16);
    out.velocity = array(:,17:18);
    out.dq = array(:,19:22);    
    out.X = array(:,11:22);
    out.gyro_bias = array(:,23:25);
    out.COM = array(:,26:28);
    wheel_slip_detected_combined = array(:,29);
    out.q_ref = array(:,30:33);
    out.omega_ref_body = array(:,34:36);    
    out.velocity_ref_inertial = array(:,37:38);
    out.q_tilt_integral = array(:,39:42);
    out.WheelSlipRampGain = array(:,43);
    out.S = array(:,44:46);   
    out.compute_time = array(:,47);
    out.torque = array(:,48:50);   
    
    out.wheel_slip_detected(:,3) = floor(wheel_slip_detected_combined ./ 100);
    out.wheel_slip_detected(:,2) = floor((wheel_slip_detected_combined - 100*out.wheel_slip_detected(:,3)) ./ 10);
    out.wheel_slip_detected(:,1) = (wheel_slip_detected_combined - 100*out.wheel_slip_detected(:,3) - 10*out.wheel_slip_detected(:,2));
   
    % Initial processing for easy visualization
    eul = quat2eul(out.q, 'ZYX');
    out.roll = eul(:,3);
    out.pitch = eul(:,2);
    out.yaw = eul(:,1);
    
    eul_ref = quat2eul(out.q_ref, 'ZYX');
    out.roll_ref = eul_ref(:,3);
    out.pitch_ref = eul_ref(:,2);
    out.yaw_ref = eul_ref(:,1);
    
    out.omega_body = zeros(size(array,1), 3);    
    out.velocity_ref_heading = zeros(size(array,1), 2);     
    for (m = 1:size(array,1))
        out.omega_body(m,:) = (2*devec*Phi(out.q(m,:)')'*out.dq(m,:)')';
        out.omega_inertial(m,:) = (2*devec*Gamma(out.q(m,:)')'*out.dq(m,:)')';                
        x_vec = devec * Phi(out.q(m,:)') * Gamma(out.q(m,:)')' * [0,1,0,0]';
        out.heading(m,:) = atan2(x_vec(2), x_vec(1)); 
        R_heading = [cos(out.heading(m,:)), -sin(out.heading(m,:)); sin(out.heading(m,:)), cos(out.heading(m,:))];        
        out.velocity_ref_heading(m,:) = (R_heading' * out.velocity_ref_inertial(m,:)')';
        out.velocity_heading(m,:) = (R_heading' * out.velocity(m,:)')';
    end
end