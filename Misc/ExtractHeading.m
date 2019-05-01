function heading = ExtractHeading(q)
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
    % X unit vector of Body in Inertial frame
    I_e_X = devec * Phi(q) * Gamma(q)' * [0;1;0;0];
    
    % Extract direction of the X-vector
    direction = I_e_X(1:2); %direction = [eye(2), zeros(2,1)] * I_e_X;
    % This direction vector points in the heading direction
    heading = atan2(direction(2), direction(1));   