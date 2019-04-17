function [model, estimate, control] = ProcessSimulation(file)
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
    
    load(file);
    
    model.time = sim.time;
    model.position = sim.position;
    model.velocity = sim.velocity;
    model.q = sim.q;
    model.dq = sim.dq;
    
    estimate.time = sim.time;
    estimate.position = sim.position_est;
    estimate.velocity = sim.velocity_est;
    estimate.q = sim.q_est;
    estimate.dq = sim.dq_est;
    
    control.time = sim.time;
    control.q_ref = sim.q_ref;
    control.omega_b_ref = sim.omega_b_ref;
    control.S = sim.S;
    control.torque = sim.tau;

    % Compute Euler angles
    eul = quat2eul(model.q, 'ZYX');
    model.roll = eul(:,3);
    model.pitch = eul(:,2);
    model.yaw = (eul(:,1));
        
    eul = quat2eul(estimate.q, 'ZYX');
    estimate.roll = eul(:,3);
    estimate.pitch = eul(:,2);
    estimate.yaw = (eul(:,1));
        
    eul = quat2eul(control.q_ref, 'ZYX');
    control.roll_ref = eul(:,3);
    control.pitch_ref = eul(:,2);
    control.yaw_ref = (eul(:,1));    
    
    % Compute angular velocity in body and inertial frame (based on numerical derivative of quaternion)
    model.omega_body = zeros(length(model.q), 3);
    model.omega_inertial = zeros(length(model.q), 3);
    for (i = 1:length(model.q))
        model.omega_body(i,:) = ( 2 * devec * Phi(model.q(i,:)')' * model.dq(i,:)' )';
        model.omega_inertial(i,:) = ( 2 * devec * Gamma(model.q(i,:)')' * model.dq(i,:)' )';
    end    
    
    estimate.omega_body = zeros(length(estimate.q), 3);
    estimate.omega_inertial = zeros(length(estimate.q), 3);
    for (i = 1:length(estimate.q))
        estimate.omega_body(i,:) = ( 2 * devec * Phi(estimate.q(i,:)')' * estimate.dq(i,:)' )';
        estimate.omega_inertial(i,:) = ( 2 * devec * Gamma(estimate.q(i,:)')' * estimate.dq(i,:)' )';
    end        
    
    % Compute heading velocity
    model.velocity_heading = zeros(length(model.q), 2);
    for (i = 1:length(model.yaw))
        model.velocity_heading(i,:) = ( rot2(model.yaw(i))'*model.velocity(i,:)' )';
    end
    
    % Compute heading velocity
    estimate.velocity_heading = zeros(length(estimate.q), 2);
    for (i = 1:length(estimate.yaw))
        estimate.velocity_heading(i,:) = ( rot2(estimate.yaw(i))'*estimate.velocity(i,:)' )';
    end    