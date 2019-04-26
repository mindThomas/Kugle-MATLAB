%% Constrained Least Squares polynomial fit for arbitrary function parameter however with t(1)=0
function polyCoeffs = ConstrainedPolyFit(t, data, order, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint)
    if (order < 1)
        error('Polynomial order needs to be at least 1');
    elseif (order < 3 && EnforceBeginEndConstraint && EnforceBeginEndAngleConstraint)
        error('Polynomial order too low to enforce both type of constraints');
    end
        
    n = length(t);     % should be the same as length(data)

    if (n ~= length(data))
        error('Mismatch in data length and length of parameter values, t');
    end
    
    if (n < (order+1)) % number of coefficients are order+1
        error('Underdetermined least squares polynomial fitting due to too few data points');
    end
    
    if (EnforceBeginEndAngleConstraint && ...
            (t(1) == t(2)) ||  ...
            (t(end) == t(end-1)) ...
        )
        error('Can not enforce begin-end angle constraint when input parameter, t, is not changing near the beginning or end');
    end
    
    %A = [];    
    %for (j = order:-1:0)
    %    A = [A, t.^j];
    %end
    %A = t.^(order:-1:0);
    A = zeros(n, order+1);
    for (i = order:-1:0)
        A(:,order-i+1) = t .^ i;
    end
    b = data;
   
    if (order >= 3 && EnforceBeginEndConstraint && EnforceBeginEndAngleConstraint) % enforce all constraints
        Aeq = [t(1).^(order:-1:0); % polynomial evaluated in t(1)
               t(end).^(order:-1:0); % polynomial evaluated in t(end)
               (order:-1:1) .* (t(1).^(order-1:-1:0)), 0; % derivative polynomial evaluated in t(1)
               (order:-1:1) .* (t(end).^(order-1:-1:0)), 0]; % derivative polynomial evaluated in t(end)
        beq = [b(1);
               b(end);
               (b(2)-b(1))/(t(2)-t(1));
               (b(end)-b(end-1))/(t(end)-t(end-1))];        
    else % limit constraints due to reduced order such that we only require start and end point to be fulfilled
        if (EnforceBeginEndConstraint)
            Aeq = [t(1).^(order:-1:0); % polynomial evaluated in t(1)
                   t(end).^(order:-1:0)]; % polynomial evaluated in t(end) 
            beq = [b(1);
                   b(end)];       
        elseif (EnforceBeginEndAngleConstraint)
            Aeq = [(order:-1:1) .* (t(1).^(order-1:-1:0)), 0;  % derivative polynomial evaluated in t(1)
               (order:-1:1) .* (t(end).^(order-1:-1:0)), 0];  % derivative polynomial evaluated in t(end)
            beq = [(b(2)-b(1))/(t(2)-t(1)); 
               (b(end)-b(end-1))/(t(end)-t(end-1))];         
        end
    end  
    
    %opts = optimset('display','off');
    
    if (EnforceBeginEndConstraint || EnforceBeginEndAngleConstraint)
        %polyCoeffs = lsqlin(A,b, [], [], Aeq, beq, [], [], [], opts);
        polyCoeffs = ConstrainedLeastSquares(A, b, Aeq, beq, n*10000);
    else
        %polyCoeffs = lsqlin(A,b, [], [], [], [], [], [], [], opts);
        polyCoeffs = ConstrainedLeastSquares(A, b, [], [], 1);
    end

%     t_visualize_fit = linspace(0,1,100);
%     data_fit = polyval(polyCoeffs, t_visualize_fit);
%     figure(10);
%     plot(t,data,'r--');
%     hold on;
%     plot(t_visualize_fit,data_fit,'b--');
%     hold off;
end    