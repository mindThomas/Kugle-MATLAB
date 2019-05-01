function rotatedPoints = RotateTrajectory(trajectory, rotation)  %#codegen
    R = [cos(rotation), sin(rotation);
         -sin(rotation), cos(rotation)];
     
    rotatedPoints = (R * trajectory')';
end