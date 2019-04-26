function [WindowTrajectory, nTrajPoints, WindowOrientation] = ExtractWindowTrajectory(TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, WindowWidth, WindowHeight, WindowOffset, OrientationSelection)

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
    
    % Only extract points within window centered at robot (-WindowWidth/2 to WindowWidth/2, -WindowHeight/2 to WindowHeight)
    WindowIdx = find(RotatedCenteredTrajectory(:,1) >= -WindowHeight/2 + WindowOffset(1) & ...
                     RotatedCenteredTrajectory(:,1) <= WindowHeight/2 + WindowOffset(1) & ...
                     RotatedCenteredTrajectory(:,2) >= -WindowWidth/2 + WindowOffset(2) & ...
                     RotatedCenteredTrajectory(:,2) <= WindowWidth/2 + WindowOffset(2));

    JumpIdx = find(diff(WindowIdx) > 1); % find indices where there is a jump in the sequence index
    StartIdx = find(WindowIdx == 1);
    EndIdx = find(WindowIdx == length(TrajectoryPoints));
    if (~isempty(StartIdx) && ~isempty(EndIdx)) % extracted window includes both start and end index - we need to merge these such that the part belonging to the end index comes first
        % Find jump index closest to the end index
        [y,idx] = min(abs(JumpIdx-repmat(EndIdx,[length(JumpIdx),1])));
        EndPortionJumpIdx = JumpIdx(idx)+1;
        %EndPortion = RotatedCenteredTrajectory(WindowIdx(EndPortionJumpIdx:EndIdx),:);
        
        % Find jump index closest to the start index
        [y,idx] = min(abs(JumpIdx-repmat(StartIdx,[length(JumpIdx),1])));
        StartPortionJumpIdx = JumpIdx(idx)+1;
        %StartPortion = RotatedCenteredTrajectory(WindowIdx(StartIdx:EndPortionJumpIdx-1),:);
        
        %WindowTrajectory = [EndPortion; StartPortion];
        WindowIdxReordered = [WindowIdx(EndPortionJumpIdx:EndIdx(1));WindowIdx(StartIdx(1):EndPortionJumpIdx-1)];
        WindowIdxReorderedWithNegative = [WindowIdx(EndPortionJumpIdx:EndIdx(1))-length(TrajectoryPoints);WindowIdx(StartIdx(1):EndPortionJumpIdx-1)];
    else
        WindowIdxReordered = WindowIdx;
        WindowIdxReorderedWithNegative = WindowIdx;
    end
    
    % Find continuous sequence of points closest to robot
    Dist = RotatedCenteredTrajectory(WindowIdxReordered,1).^2 + RotatedCenteredTrajectory(WindowIdxReordered,2).^2;
    [y,ClosestIdx] = min(Dist);
    JumpIdx = [1; find(diff(WindowIdxReorderedWithNegative) > 1)+1; length(WindowIdxReordered)]; % find indices where there is a jump in the sequence index
    
    SortedJumpIdx = sort([JumpIdx;ClosestIdx]);
    CenterIdx = find(SortedJumpIdx == ClosestIdx);
    
    if (CenterIdx > 1)
        %SeqJumpIdx0 = SortedJumpIdx(CenterIdx(1)-1); % include old points as long as they are connected        
        SeqJumpIdx0 = SortedJumpIdx(CenterIdx(1))-2; % only include future trajectory points (+ 2 previous/old points)
        if (SeqJumpIdx0 < 1)
            SeqJumpIdx0 = 1;
        end
    else
        SeqJumpIdx0 = SortedJumpIdx(1);
    end
    if (CenterIdx < length(SortedJumpIdx))
        SeqJumpIdx1 = SortedJumpIdx(CenterIdx(1)+1);
    else
        SeqJumpIdx1 = SortedJumpIdx(length(SortedJumpIdx));
    end
    
%     [y,idx] = min(abs(JumpIdx-ClosestIdx));
%     SeqJumpIdx0 = JumpIdx(idx);
%     JumpIdx(idx) = []; % remove first jump index    
%     [y,idx] = min(abs(JumpIdx-ClosestIdx)); % find second jump index
%     SeqJumpIdx1 = JumpIdx(idx);   
    
    %if (SeqJumpIdx1 > SeqJumpIdx0)
        WindowTrajectory_tmp = RotatedCenteredTrajectory(WindowIdxReordered(SeqJumpIdx0:SeqJumpIdx1),:);
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