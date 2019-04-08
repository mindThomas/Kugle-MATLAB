function q_ref_out = HeadingIndependentReference(q_ref, q)
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

    %% Derive tilt and heading from combined quaternion
    % Z unit vector of Body in Inertial frame
    I_e_Z = devec * Phi(q) * Gamma(q)' * [0;0;0;1];
    
    % Extract direction in which this Z vector is pointing (project down to XY-plane)    
    direction = I_e_Z(1:2); %direction = [eye(2), zeros(2,1)] * I_e_Z;
    % Tilt amount corresponds to sin^-1 of the length of this vector
    tilt = asin(norm(direction));        
    
    % normalize direction vector before forming tilt quaternion
    if (norm(direction) ~= 0)
        direction = direction / norm(direction);
    else
        direction = 0*direction;
    end

    % Tilt quaternion describes the current (heading independent) tilt of the robot
    q_tilt = [cos(tilt/2);
              sin(tilt/2) * -direction(2);
              sin(tilt/2) * direction(1);
              0];

    % Remove the tilt from the current quaternion to extract the heading part of the quaternion
    q_heading = Phi(q_tilt)' * q
        
    %% Derive tilt from quaternion reference
    % Z unit vector of Body in Inertial frame
    I_e_Z = devec * Phi(q_ref) * Gamma(q_ref)' * [0;0;0;1];
    
    % Extract direction in which this Z vector is pointing (project down to XY-plane)    
    direction = I_e_Z(1:2); %direction = [eye(2), zeros(2,1)] * I_e_Z;
    % Tilt amount corresponds to sin^-1 of the length of this vector
    tilt = asin(norm(direction));        
    
    % normalize direction vector before forming tilt quaternion
    if (norm(direction) ~= 0)
        direction = direction / norm(direction);
    else
        direction = 0*direction;
    end

    % Tilt quaternion describes the current (heading independent) tilt of the robot
    q_tilt_ref = [cos(tilt/2);
              sin(tilt/2) * -direction(2);
              sin(tilt/2) * direction(1);
              0];

    % Remove the tilt from the current quaternion to extract the heading part of the quaternion
    q_heading_ref = Phi(q_tilt_ref)' * q_ref;
    
    %% Calculate reference quaternion by multiplying with the desired reference
    % We multiply on the right side since the heading quaternion is given
    % around the Z-axis in the body frame - thus in the frame of the desired tilt angle
    %q_ref_out = Phi(q_tilt_ref) * q_heading;     % if desired reference is given in inertial heading frame
    q_ref_out = Phi(q_heading) * q_tilt_ref;      % if desired reference is given in body heading frame        
    
end