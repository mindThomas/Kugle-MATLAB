% Find the minimum by Newton minimization given upper and lower parameter bounds
function s_min = PolynomialMinimumFinderWithBounds(coeff, s0, sLower, sUpper, stoppingCriteria, maxIterations)
    dcoeff = ComputeDerivativePolynomialCoefficients(coeff);
    ddcoeff = ComputeDerivativePolynomialCoefficients(dcoeff);
  
    % Newton's method - https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization
    converged = false;
    iterations = 0;
    s = s0;
    while (~converged && iterations < maxIterations)
        FirstDerivative = EvaluatePolynomial(dcoeff, s); % first derivative, for higher dimensions it would be the Gradient
        SecondDerivative = EvaluatePolynomial(ddcoeff, s); % second derivative, for higher dimensions it would be the Hessian
        delta_s = -FirstDerivative / SecondDerivative;
        s = s + delta_s;
        if (s < sLower)
            s = sLower;
        end
        if (s > sUpper)
            s = sUpper;
        end

        if (abs(delta_s) < stoppingCriteria)
            converged = true;
        end
        iterations = iterations + 1;
    end

    s_min = s;