% Find the minimum of the derivative polynomial by brute-force line search
function s_min = PolynomialMinimumFinderNumerical2(coeff, s0, precision)
    span = 0.1;
    dcoeff = ComputeDerivativePolynomialCoefficients(coeff);
    
    converged = false;
    while (~converged)
        s = (-span:precision:span)' + s0;    
        y = EvaluatePolynomial(dcoeff, s) .^ 2; % we need to find a value as close to 0 as possible
        
        min = 1e99;
        s_min = 0;
        max = -1e99;
        s_max = 0;
        for (i = 1:length(s))
            if (y(i) < min)
                min = y(i);
                s_min = s(i);
            end
            if (y(i) > max)
                max = y(i);
                s_max = s(i);
            end
        end
        
        if (s_min ~= 0 && y(1) > min && y(end) > min)
            converged = true;
        end
        span = span * 2;
    end
        
    plot(s, y);