function q = ZYXEulerAngles2Quaternion(rpy)           
        c = cos(rpy/2);
        s = sin(rpy/2);
        
        % Compute quaternion elements
        qw = c(3)*c(2)*c(1)+s(3)*s(2)*s(1);
        qx = c(3)*c(2)*s(1)-s(3)*s(2)*c(1);
        qy = c(3)*s(2)*c(1)+s(3)*c(2)*s(1);
        qz = s(3)*c(2)*c(1)-c(3)*s(2)*s(1);
        
        % Assemble quaternion
        q = [qw; qx; qy; qz];
end