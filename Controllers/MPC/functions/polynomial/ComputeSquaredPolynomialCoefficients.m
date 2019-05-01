% Given polynomial coefficients for:
% f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0
% This function computes the coefficients of the polynomial f^2(x)
% Coefficients are ordered such that coeff(1) = c_n  (highest order)
function coeff_squared = ComputeSquaredPolynomialCoefficients(coeff)  %#codegen
    % f^2(x) = sum(sum(c_i * c_j * x^(i+j))
    % The resulting coefficients, coeff_k = c_i*c_j | i+j=k
    n = length(coeff) - 1; % input polynomial order
    m = 2*n; % squared (output) polynomial order
    
    coeff_squared = zeros(m+1,1);
    for (k = 0:m)
        for (i = 0:n)
            for (j = 0:n)                
                if (i+j == k)
                    coeff_squared(m-k+1) = coeff_squared(m-k+1) + coeff(n-i+1)*coeff(n-j+1);
                end
            end
        end    
    end    