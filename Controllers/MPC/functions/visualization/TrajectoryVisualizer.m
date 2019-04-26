classdef TrajectoryVisualizer < handle
    properties
        x
        y
        r % robot radius
        yaw
        velocity
        GlobalTrajectoryPoints
        MPCWindowTrajectoryPoints
        MPCPredictionTrajectoryPoints
        fig
        x_min
        x_max
        y_min
        y_max
    end
    properties (Access = private)
        ax           
    end
    
    methods
        function obj = TrajectoryVisualizer(visualize)                                            
            if (nargin == 0 || (nargin > 0 && visualize))
                obj.fig = figure(1);
                obj.ax = subplot(1,2,1);
                obj.ax.XLim = [-1 1];
                obj.ax.YLim = [-1 1];
                obj.ax.DataAspectRatio = [1 1 1]; % set square aspect ratio
                obj.ax.PlotBoxAspectRatio = [1 1 1];
            end
            
            obj.x_min = -1;
            obj.y_min = -1;
            obj.x_max = 1;
            obj.y_max = 1;
            
            obj.r = 0.1;
            obj.x = 0;
            obj.y = 0;
            obj.yaw = 0;
            obj.velocity = 0;
            
            if (nargin == 0 || (nargin > 0 && visualize))
                obj.Draw();
            end
        end
        
        function ChangeLimits(obj, x_min, y_min, x_max, y_max)
            obj.x_min = x_min;
            obj.y_min = y_min;
            obj.x_max = x_max;
            obj.y_max = y_max;
            obj.ax.XLim = [x_min x_max];
            obj.ax.YLim = [y_min y_max];
        end
            
        function Draw(obj)
            cla(obj.ax);            
            
            % Plot robot
            d = 2 * obj.r;
            px = obj.x - obj.r;
            py = obj.y - obj.r;
            rectangle(obj.ax, 'Position',[px py d d],'Curvature',[1,1],'EdgeColor',[1 0 0]);            
            
            % Plot velocity line
            if (norm(obj.velocity) > 0)
                if (length(obj.velocity) == 1) % assuming velocity to be given in heading direction
                    c = [obj.x obj.y] + obj.r*[cos(obj.yaw) sin(obj.yaw)];
                    h = c + 0.1*obj.velocity*[cos(obj.yaw) sin(obj.yaw)];
                    line(obj.ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
                elseif (length(obj.velocity) == 2) % 2d velocity given
                    c = [obj.x obj.y] + obj.r * obj.velocity/norm(obj.velocity);
                    h = c + 0.5*obj.velocity;
                    line(obj.ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[0 0 1],'LineWidth',1.5);
                end
            end
                        
            % Plot heading line
            c = [obj.x obj.y];
            h = c + obj.r*[cos(obj.yaw) sin(obj.yaw)];
            line(obj.ax, 'XData',[c(1),h(1)], 'YData',[c(2),h(2)], 'Color',[1 0 0],'LineWidth',0.5);
            
            % Plot overall trajectory
            if (size(obj.GlobalTrajectoryPoints,2) == 2)
                hold(obj.ax, 'on');
                plot(obj.ax, obj.GlobalTrajectoryPoints(:,1), obj.GlobalTrajectoryPoints(:,2), 'k-', 'MarkerSize', 10);
                hold(obj.ax, 'off');
            end
            
            % Plot MPC horizon trajectory
            if (size(obj.MPCWindowTrajectoryPoints,2) == 2)
                hold(obj.ax, 'on');
                plot(obj.ax, obj.MPCWindowTrajectoryPoints(:,1), obj.MPCWindowTrajectoryPoints(:,2), 'g-');
                hold(obj.ax, 'off');
            end    
            
            % Plot MPC horizon (predicted) points
            if (size(obj.MPCPredictionTrajectoryPoints,2) == 2)
                hold(obj.ax, 'on');
                plot(obj.ax, obj.MPCPredictionTrajectoryPoints(:,1), obj.MPCPredictionTrajectoryPoints(:,2), 'g.', 'MarkerSize', 20);
                hold(obj.ax, 'off');
            end               
        end
    end
end