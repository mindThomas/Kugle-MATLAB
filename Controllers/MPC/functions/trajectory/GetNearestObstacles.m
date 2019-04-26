function NearestObstacles = GetNearestObstacles(RobotPos, obstacles, number_of_nearest_obstacles)
    
    obstacles_pos = obstacles(:,1:2);
    obstacles_radius = obstacles(:,3);
    obstacles_vec = obstacles_pos - RobotPos;
    obstacles_dist = sqrt(obstacles_vec(:,1).^2 + obstacles_vec(:,2).^2);
    obstacles_proximity = obstacles_dist - obstacles_radius;

    [values, idx] = sort(obstacles_proximity);    
    idx(find(obstacles_proximity(idx) < -1/4*obstacles_dist(idx))) = [];
    
    if (length(idx) > number_of_nearest_obstacles)
        NearestObstacles = obstacles(idx(1:number_of_nearest_obstacles),:);
    else
        NearestObstacles = obstacles(idx,:);
    end
end