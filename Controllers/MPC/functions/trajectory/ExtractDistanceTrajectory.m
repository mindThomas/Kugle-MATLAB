function [WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractDistanceTrajectory(TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, OrientationSelection)  %#codegen

% OrientationSelection = 0;
% RobotPos = [4.20, 0.8];
% Velocity = [1, 0.2];
    if (OrientationSelection == 0) % inertial frame
        WindowOrientation = 0;
    elseif (OrientationSelection == 1) % robot yaw (heading)
        WindowOrientation = RobotYaw;
    elseif (OrientationSelection == 2) % velocity direction
        WindowOrientation = atan2(Velocity(2),Velocity(1));
    else
        WindowOrientation = 0;
    end

    R_orientation = [cos(WindowOrientation), sin(WindowOrientation);
                    -sin(WindowOrientation), cos(WindowOrientation)];
    
    % Rotate trajectory into robot heading
    RobotCentricTrajectoryPoints = TrajectoryPoints - repmat([RobotPos(1), RobotPos(2)], [length(TrajectoryPoints),1]);
    RotatedCenteredTrajectory = RotateTrajectory(RobotCentricTrajectoryPoints, WindowOrientation);   
    
    % Find continuous sequence of points closest to robot
    Dist = RotatedCenteredTrajectory(:,1).^2 + RotatedCenteredTrajectory(:,2).^2;
    [y,ClosestIdx] = min(Dist);
    
    WindowTrajectory_tmp = [RotatedCenteredTrajectory(ClosestIdx:end,:); RotatedCenteredTrajectory(1:ClosestIdx-1,:)];
    
    %if (SeqJumpIdx1 > SeqJumpIdx0)
        
    %else
    %    WindowTrajectory = RotatedCenteredTrajectory(WindowIdxReordered(SeqJumpIdx1+1:SeqJumpIdx0),:);
    %end
    
    if (ExtractDist > 0) % only extract certain future distance of trajectory based on a crude distance approximation
        dWindowTrajectory = diff(WindowTrajectory_tmp);    
        ApproxDist = cumsum(sqrt(dWindowTrajectory(:,1).^2 + dWindowTrajectory(:,2).^2));
        %ExtractDist = N*ts*velocity;
        WindowTrajectory_Shortened = WindowTrajectory_tmp(find(ApproxDist < ExtractDist),:);
    
        WindowTrajectory = WindowTrajectory_Shortened;
    else
        WindowTrajectory = WindowTrajectory_tmp;
    end

    nTrajPoints = length(WindowTrajectory);
    
%     splitIdx = find(diff(WindowIdx) > 1); % correct if/when the trajectory start and end is in the window, which messes up the order due to indexing (recommended to make trajectory object with sequence id)
%     if (~isempty(splitIdx))  % correct (by reordering) trajectory index list
%         splitIdx = splitIdx(1);
%         WindowIdx = [WindowIdx(splitIdx+1:end); WindowIdx(1:splitIdx)];
%     end
%     WindowTrajectory = RotatedCenteredTrajectory(WindowIdx,:);       

    %SquaredDistanceToPointsInWindow = RotatedCenteredTrajectory(:,1).^2 + RotatedCenteredTrajectory(:,2).^2;
    %[y, ClosestPointWithinWindowIdx] = min(SquaredDistanceToPointsInWindow);    

    % Select continuous series of points which includes this closest point

    %WindowTrajectory = RotatedCenteredTrajectory(ClosestPointWithinWindowIdx:end);    
       
    
    
% figure(1);
% clf;
% ax1 = axes;
% plot(ax1, TrajectoryPoints(:,1), TrajectoryPoints(:,2), 'k-', 'MarkerSize', 10);
% hold(ax1,'on');   
% InertialWindowTrajectory = RotateTrajectory(WindowTrajectory, -WindowOrientation) + repmat([RobotPos(1),RobotPos(2)], [length(WindowTrajectory),1]);
% plot(InertialWindowTrajectory(:,1), InertialWindowTrajectory(:,2), 'k*', 'MarkerSize', 2);
% PlotAxRobotWithTiltAndVelocity(ax1, [RobotPos(1),RobotPos(2)], WindowOrientation, 0.05, [Velocity(1),Velocity(2)], [0,0]);
% %plot(InertialReferencePoints(:,1), InertialReferencePoints(:,2), 'g*', 'MarkerSize', 3);
% %plot(InertialMPCtrajectory(:,1), InertialMPCtrajectory(:,2), 'r*', 'MarkerSize', 3);  
% WindowCornersCentered = [-WindowHeight/2, -WindowWidth/2
%                           WindowHeight/2, -WindowWidth/2
%                           WindowHeight/2, WindowWidth/2
%                           -WindowHeight/2, WindowWidth/2
%                           -WindowHeight/2, -WindowWidth/2] + WindowOffset;                     
% WindowCorners = RotateTrajectory(WindowCornersCentered, -WindowOrientation) + repmat([RobotPos(1), RobotPos(2)], [5,1]);
% plot(WindowCorners(:,1), WindowCorners(:,2), 'k--'); 
% hold('off');   
% axis equal;
% xlim([min(TrajectoryPoints(:,1))*2, max(TrajectoryPoints(:,1))*2]);
% ylim([min(TrajectoryPoints(:,2))*2, max(TrajectoryPoints(:,2))*2]);
% 
% figure(2);
% clf;
% ax2 = axes;
% plot(ax2, -WindowTrajectory(:,2), WindowTrajectory(:,1), 'k*'); % plot rotated
% hold(ax2, 'on');
% Vel = R_orientation * [Velocity(1);Velocity(2)];       
% PlotAxRobotWithTiltAndVelocity(ax2, [0,0], deg2rad(90)+0, 0.05, [-Vel(2),Vel(1)], [0,0]);    
% % if (trajectoryLength > 0)
% %     s = (0:0.01:trajectoryLength)';
% %     trajectory_x = EvaluatePolynomial(coeff_trajectory_x, s);
% %     trajectory_y = EvaluatePolynomial(coeff_trajectory_y, s);
% %     plot(ax2, -trajectory_y, trajectory_x, 'b-');
% % end    
% %plot(ax2, -ReferencePoints(:,2), ReferencePoints(:,1), 'g*');
% %plot(ax2, -MPCtrajectory(:,2), MPCtrajectory(:,1), 'r*');  
% hold(ax2, 'off');
% axis(ax2, 'equal');
% xlim(ax2, [-WindowWidth/2 + WindowOffset(2), WindowWidth/2 + WindowOffset(2)]);
% ylim(ax2, [-WindowHeight/2 + WindowOffset(1), WindowHeight/2 + WindowOffset(1)]);

end