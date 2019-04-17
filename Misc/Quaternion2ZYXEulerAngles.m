function rpy = QuaternionToZYXEulerAngles(q)   
    % Normalize if incorrect (due to small numerical errors)
    q = q / norm(q); 
    
    % Extract quaternion elements
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % Ensure asin stays within -1 to 1 since asin of values larger than 1 produces complex number
    aSinInput = -2*(qx*qz-qw*qy);
    aSinInput(aSinInput > 1) = 1;
    aSinInput(aSinInput < -1) = -1;
   	
    % Compute Euler ZYX angles
    roll = atan2( 2*(qy*qz+qw*qx), qw^2 - qx^2 - qy^2 + qz^2 );
    pitch = asin( aSinInput );
    yaw = atan2( 2*(qx*qy+qw*qz), qw^2 + qx^2 - qy^2 - qz^2 );
    
    rpy = [roll; pitch; yaw];