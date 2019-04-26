function [TrajectoryPoints, coeff_xs, coeff_ys, windowTrajectoryLength, minDistancePoint] = FitReferencePathPolynomial(WindowTrajectoryPoints, RobotPosInWindow, approximation_order, velocity, ts, N)
    % ts = sample time
    % N = MPC horizon (samples)

    % Parameters
    order = approximation_order; % x(t), y(t)
    order_ts = order+1; % t(s)
    order_fs = order+1; % x(s), y(s)
    EnforceBeginEndConstraint = true;
    EnforceBeginEndAngleConstraint = true;
    
    %% Fit window points to two polynomials, x(t) and y(t), using parameters t starting with t0=0 and spaced with the distance between each point (chordal parameterization)
    diffWindowTrajectory = diff(WindowTrajectoryPoints);
    t = [0;cumsum(sqrt(diffWindowTrajectory(1:length(diffWindowTrajectory),1).^2 + diffWindowTrajectory(1:length(diffWindowTrajectory),2).^2))];
    
    coeff_x = ConstrainedPolyFit(t, WindowTrajectoryPoints(1:length(WindowTrajectoryPoints),1), order, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
    coeff_y = ConstrainedPolyFit(t, WindowTrajectoryPoints(1:length(WindowTrajectoryPoints),2), order, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);    
    
    %% Compute numerical approximation of arc length at the points, t, using the fitted polynomial
    s = zeros(length(t),1);
    for (i = 1:length(t))
       s(i) = ArcLengthApproximation(coeff_x, coeff_y, t(i));
    end
    %s = ArcLengthApproximation(coeff_x, coeff_y, t);
            
    s_total = s(end); % total length of approximated trajectory
        
    %% Fit t(s) polynomial
    coeff_ts = ConstrainedPolyFit(s, t, order_ts, true, false);
    
    s_eval = linspace(0, s_total, 100)'; % make 100 linearly seperated distance points for plotting
    t_eval = EvaluatePolynomial(coeff_ts, s_eval);
    
%     figure(100);
%     plot(s, t, s_eval, t_eval);
%     legend('Fitting points', 'Polynomial approximation');
%     xlabel('s');
%     ylabel('t');
%     title('t(s) relationship');
    
    %% Use the evenly spaced parameter values to generate new points from the two fitted polynomials, x_0,...,x_n = x(t_0 ),...,x(t_n) and same for y(t)
    x_eval = EvaluatePolynomial(coeff_x, t_eval);
    y_eval = EvaluatePolynomial(coeff_y, t_eval);
    
    %% Fit x(s) and y(s) on the newly generated points, x_0,...,x_n and y_0,...,y_n using the evenly spaced distance parameters, s_0,...,s_n, as the parameter
    coeff_xs = ConstrainedPolyFit(s_eval, x_eval, order_fs, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
    coeff_ys = ConstrainedPolyFit(s_eval, y_eval, order_fs, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);    
    
    %% Use the newly fitted polynomial to find the distance, s, closest to the current robot position, which is (0,0) because we are using window trajectory points
    % This only works if RobotInWindow = [0;0] since the distance between robot and trajectory polynomial the
    % is then given by just the distance from origin to the polynomial
    % dist(s) = sqrt( f_x(s)^2 + f_y(s)^2 )
    % Create a distance polynomial and differentiate it 
    xs_squared_coeff = ComputeSquaredPolynomialCoefficients(coeff_xs);
    ys_squared_coeff = ComputeSquaredPolynomialCoefficients(coeff_ys);
    % And since we want to find the closest distance, we can also just
    % minimize the squared distance: dist(s)^2
    % dist(s)^2 = f_x(s)^2 + f_y(s)^2
    dist_squared_coeff = xs_squared_coeff + ys_squared_coeff;
    % find the distance, s, corresponding to the smallest distance
    s_mindist = PolynomialMinimumFinderWithBounds(dist_squared_coeff, 0, 0, s_total, 0.001, 100);
    
    %% Visualization/debugging only
%     xs_eval = polyval(coeff_xs, s_eval);
%     ys_eval = polyval(coeff_ys, s_eval);
%    
%     figure(2);
%     subplot(2,1,1);
%     plot(s_eval, x_eval, s_eval, xs_eval);
%     subplot(2,1,2);
%     plot(s_eval, y_eval, s_eval, ys_eval);
%    
%     figure(3);
%     plot(WindowTrajectory(:,1), WindowTrajectory(:,2));
%     hold on;
%     plot(x_eval, y_eval);
%     plot(xs_eval, ys_eval);
%     hold off;
%     legend('Window trajectory', 't-approximation', 's-approximation');
%        
%     %% Compute numerical approximation of arc length at the points, s, using the final polynomial (as confirmation/test)
%     % Arc length is given as:
%     % integral( sqrt( dxdt^2 + dydt^2 ) ) dt
%     dxs_coeff = coeff_xs(1:end-1) .* (order_fs:-1:1)'; % taking the difference of a polynomial, moves the coefficients
%     dys_coeff = coeff_ys(1:end-1) .* (order_fs:-1:1)';
%     
%     dxdt = @(t) t.^(order_fs-1:-1:0) * dxs_coeff;
%     dydt = @(t) t.^(order_fs-1:-1:0) * dys_coeff;
%     arclength = @(t) sqrt( dxdt(t)^2 + dydt(t)^2 );
%     s = zeros(length(s_eval), 1);
%     for (i = 1:length(s_eval))
%         s(i) = integral(arclength, 0, s_eval(i), 'ArrayValued', true);
%     end       
%     s-s_eval'
%     mean(s-s_eval')
%     3*sqrt(var(s-s_eval'))

    %% Generate trajectory reference points based on velocity, sample rate and horizon length, starting in the location closest to the robot
    % Using constant velocity model
    s_ref = (0:(N-1))'*ts * velocity + s_mindist;
    x_ref = EvaluatePolynomial(coeff_xs, s_ref);
    y_ref = EvaluatePolynomial(coeff_ys, s_ref);
    
    TrajectoryPoints = [x_ref, y_ref];
    windowTrajectoryLength = s_total;
    minDistancePoint = s_mindist;
end