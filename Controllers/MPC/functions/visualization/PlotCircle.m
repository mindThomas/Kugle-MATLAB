function PlotCircle(center, radius)
    d = 2 * radius;
    px = center(1) - radius;
    py = center(1) - radius;
    rectangle('Position',[px py d d],'Curvature',[1,1],'EdgeColor',[1 0 0]);   
end