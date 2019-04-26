function PlotRobot(center, yaw, radius)
    d = 2 * radius;
    px = center(1) - radius;
    py = center(2) - radius;
    rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[1 0 0]);   
    
    % Plot heading line    
    h = center + radius*[cos(yaw) sin(yaw)];
    line('XData',[center(1),h(1)], 'YData',[center(2),h(2)], 'Color',[1 0 0],'LineWidth',0.5);                
end