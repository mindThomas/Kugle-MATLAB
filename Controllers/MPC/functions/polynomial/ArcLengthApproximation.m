% Based on http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.5.2912&rep=rep1&type=pdf
function s = ArcLengthApproximation(coeff_x, coeff_y, t)
    % Arc length is given as:
    % integral( sqrt( dxdt^2 + dydt^2 ) ) dt
    % This can however be approximated by using the velocity polynomial
    % Q = sqrt( (df_x/dx)^2 + (df_dy)^2 );
    % First we compute the coefficients of the inner polynomial, f
    % f = (df_x/dx)^2 + (df_dy)^2   
    dx_coeff = ComputeDerivativePolynomialCoefficients(coeff_x); % taking the difference of a polynomial, moves the coefficients
    dy_coeff = ComputeDerivativePolynomialCoefficients(coeff_y);
    dx_squared_coeff = ComputeSquaredPolynomialCoefficients(dx_coeff);
    dy_squared_coeff = ComputeSquaredPolynomialCoefficients(dy_coeff);
    f_coeff = dx_squared_coeff + dy_squared_coeff;
    % Such that
    % Q = sqrt(EvaluatePolynomial(f_coeff, t))   
    
    % Now the approximation (see paper above) is given by
    % s(t) = t/2 * (5/9 * Q(1.774597*t/2) + 8/9 * Q(t/2) + 5/9 * Q(0.225403*t/2))
    s = t/2 .* (5/9 * sqrt(EvaluatePolynomial(f_coeff, 1.774597*t/2)) + 8/9 * sqrt(EvaluatePolynomial(f_coeff, t/2)) + 5/9 * sqrt(EvaluatePolynomial(f_coeff, 0.225403*t/2)));        