function NearestObstacles = GetNearestObstacles(RobotPos, obstacles, number_of_nearest_obstacles)
    
    obstacles_pos = obstacles(:,1:2);
    obstacles_radius = obstacles(:,3);
    obstacles_vec = obstacles_pos - repmat(RobotPos, [size(obstacles_pos,1), 1]);
    obstacles_dist = sqrt(obstacles_vec(:,1).^2 + obstacles_vec(:,2).^2);
    obstacles_proximity = obstacles_dist - obstacles_radius;

    % Determine closest obstacles by sorting the proximity list
    [values, idx] = sort(obstacles_proximity);    
    
    % Remove any obstacles which we are inside (e.g. due to random obstacles spawning on top of robot)
    %idx(find(obstacles_proximity(idx) < -1/4*obstacles_dist(idx))) = [];
    sortedIdx = [];
    for (i = 1:length(idx))
        if (obstacles_proximity(idx(i)) >= -1/4*obstacles_dist(idx(i)))
            sortedIdx = [sortedIdx; idx(i)];
        end
    end
    
    if (length(idx) > number_of_nearest_obstacles)
        NearestObstacles = obstacles(idx(1:number_of_nearest_obstacles),:);
    else
        NearestObstacles = obstacles(idx,:);
    end
end