% Numerical integration to compute arc curve length
function s = ArcLengthApproximationNumerical(coeff_x, coeff_y, t, precision)
    % Arc length is given as:
    % integral( sqrt( dxdt^2 + dydt^2 ) ) dt
    % First we compute the coefficients of the inner polynomial, f
    % f = (df_x/dx)^2 + (df_dy)^2   
    dx_coeff = ComputeDerivativePolynomialCoefficients(coeff_x); % taking the difference of a polynomial, moves the coefficients
    dy_coeff = ComputeDerivativePolynomialCoefficients(coeff_y);
    dx_squared_coeff = ComputeSquaredPolynomialCoefficients(dx_coeff);
    dy_squared_coeff = ComputeSquaredPolynomialCoefficients(dy_coeff);
    f_coeff = dx_squared_coeff + dy_squared_coeff;

    % Now compute arc curve length by performing numerical integration    
    t_ = (0:precision:t)';
    y = sqrt(EvaluatePolynomial(f_coeff, t_));    
    % Perform trapezoidal integration
    y = [y;0];
    % s[k] = s[k-1] + dt * (f[k-1] + f[k]) / 2
    yT = precision * (y(1:end-1) + y(2:end)) / 2;
    integrate = cumsum(yT);
    s = integrate(end);