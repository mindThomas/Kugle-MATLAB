function [TrajectoryPoints, VisualizationLimits] = GenerateTestTrajectory()
    %% Generate test trajectory
    TrajectoryPoints = [];
    for (i = 0:(100+50+100+50-1))
        if (i < 100) % straight x=10, y=-50...50
            p = [20, i-50];
        elseif (i < 100+50) % left turn 180 degree with radius, r
            r = 20;
            p = [20-r,50] + r*[cos(pi/50*(i-100)), sin(pi/50*(i-100))];
        elseif (i < 100+50+100) % straight x=-10, y=50...-50
            p = [-20, 50-(i-100-50)];
        elseif (i < 100+50+100+50) % left turn 180 degree with radius, r
            r = 20;
            p = [20-r,-50] + r*[cos(pi/50*(i-100-50-100+50)), sin(pi/50*(i-100-50-100+50))];        
        end
        TrajectoryPoints = [TrajectoryPoints; p];
    end

    TrajectoryPoints = [-TrajectoryPoints(:,2) TrajectoryPoints(:,1)]; % Rotate trajectory 90 degree
    TrajectoryPoints = TrajectoryPoints / 10; % downscale trajectory
    
    VisualizationLimits.x_min = -9;
    VisualizationLimits.y_min = -4;
    VisualizationLimits.x_max = 9;
    VisualizationLimits.y_max = 4;
end