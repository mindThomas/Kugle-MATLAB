% Given polynomial coefficients for:
% f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0
% This function computes the coefficients of the polynomial f'(x)
% f'(x) = df/dx = n*c_n*x^(n-1) + (n-1)*c_n-1*x^(n-2) + ... + 
% Coefficients are ordered such that coeff(1) = c_n  (highest order)
function dcoeff = ComputeDerivativePolynomialCoefficients(coeff)
    n = length(coeff) - 1;
    dcoeff = (n:-1:1)'.*coeff(1:end-1);
    