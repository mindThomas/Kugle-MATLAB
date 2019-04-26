function PlotRobotWithVelocity(center, yaw, radius, velocity)
    d = 2 * radius;
    px = center(1) - radius;
    py = center(2) - radius;
    rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[1 0 0]);   
    
    % Plot heading line    
    h = center + radius*[cos(yaw) sin(yaw)];
    line('XData',[center(1),h(1)], 'YData',[center(2),h(2)], 'Color',[1 0 0],'LineWidth',0.5);                
    
    % Plot velocity line
    if (norm(velocity) > 0)
        if (length(velocity) == 1) % assuming velocity to be given in heading direction
            c = [center(1) center(2)] + radius*[cos(yaw) sin(yaw)];
            h = c + 0.1*velocity*[cos(yaw) sin(yaw)];
            line('XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
        elseif (length(velocity) == 2) % 2d velocity given
            c = [center(1) center(2)] + radius * velocity/norm(velocity);
            h = c + 0.5*velocity;
            line('XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
        end
    end    
end