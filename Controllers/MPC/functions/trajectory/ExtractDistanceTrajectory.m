function [WindowTrajectory, nTrajPoints, WindowOrientation, ClosestIdx] = ExtractDistanceTrajectory(TrajectoryPoints, RobotPos, RobotYaw, Velocity, ExtractDist, OrientationSelection, PreviousClosestIndex)  %#codegen

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
    if (PreviousClosestIndex > 0 && mod(ClosestIdx-PreviousClosestIndex, length(TrajectoryPoints)) > 50) % index jump detected
        % Instead of just taking the minimum, which will lead to problems with
        % overlapping trajectories, we find the minimum closest to the previous index
        [distSorted,idxSorted] = sort(Dist,'ascend');            
        idxSorted2 = idxSorted(distSorted < 2*distSorted(1));
        [y,idx] = min(idxSorted2-PreviousClosestIndex);
        ClosestIdx = idxSorted2(idx);
        %ClosestIdx = PreviousClosestIndex+10; % note that this jump value of 10 is dependent on the density of the trajectory
        %if (ClosestIdx > length(TrajectoryPoints))
        %    ClosestIdx = 1;% + mod(ClosestIdx, length(TrajectoryPoints));
        %end
    end
    
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
end