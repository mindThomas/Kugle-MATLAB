% Find the minimum by Newton minimization
function s_min = PolynomialMinimumFinder(coeff, s0, stoppingCriteria)
    dcoeff = ComputeDerivativePolynomialCoefficients(coeff);
    ddcoeff = ComputeDerivativePolynomialCoefficients(dcoeff);              
    
    % Newton's method - https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization
    converged = false;
    s = s0;
    while (~converged)
        FirstDerivative = EvaluatePolynomial(dcoeff, s); % first derivative, for higher dimensions it would be the Gradient
        SecondDerivative = EvaluatePolynomial(ddcoeff, s); % second derivative, for higher dimensions it would be the Hessian
        delta_s = -FirstDerivative / SecondDerivative;
        s = s + delta_s;
        
        if (abs(delta_s) < stoppingCriteria)
            converged = true;
        end
    end
    
    s_min = s;