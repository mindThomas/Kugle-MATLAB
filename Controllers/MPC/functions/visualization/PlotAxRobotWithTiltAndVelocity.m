function PlotAxRobotWithTiltAndVelocity(ax, center, yaw, radius, velocity, tiltDirection)
    Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
              q(2) q(1)  -q(4) q(3);
              q(3) q(4)  q(1)  -q(2);
              q(4) -q(3) q(2)  q(1)];           
    Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
                 p(2) p(1) p(4) -p(3);
                 p(3) -p(4) p(1) p(2);
                 p(4) p(3) -p(2) p(1)];  

    devec = [0,1,0,0;0,0,1,0;0,0,0,1]; % 'v' in notes
    vec = [0,0,0;1,0,0;0,1,0;0,0,1]; % '^' in notes
    I_conj = diag([1,-1,-1,-1]); 

    d = 2 * radius;
    px = center(1) - radius;
    py = center(2) - radius;
    rectangle(ax, 'Position',[px py d d],'Curvature',[1,1],'EdgeColor',[1 0 0]);   
    
    % Plot heading line    
    h = center + radius*[cos(yaw) sin(yaw)];
    line(ax, 'XData',[center(1),h(1)], 'YData',[center(2),h(2)], 'Color',[1 0 0],'LineWidth',0.5);                
    
    % Plot velocity line
    if (norm(velocity) > 0)
        if (length(velocity) == 1) % assuming velocity to be given in heading direction
            c = [center(1) center(2)] + radius*[cos(yaw) sin(yaw)];
            h = c + 0.1*velocity*[cos(yaw) sin(yaw)];
            line(ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
        elseif (length(velocity) == 2) % 2d velocity given
            c = [center(1) center(2)] + radius * velocity/norm(velocity);
            h = c + 0.5*velocity;
            line(ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
        end
    end  
    
    % Plot tilt line  
    if (norm(tiltDirection) > 0)
        c = [center(1) center(2)] + radius * tiltDirection/norm(tiltDirection);
        h = c + 0.5*tiltDirection;
        line(ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 1 0],'LineWidth',1.5); 
    end
end