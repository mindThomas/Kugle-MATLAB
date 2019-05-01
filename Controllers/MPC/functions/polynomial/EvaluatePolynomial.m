% Evaluate a polynomial   ( similar to Y = polyval(P,X) )
function y = EvaluatePolynomial(coeff, x)  %#codegen
    order = length(coeff) - 1;
    n = length(x);
    %y = (coeff' * (x.^(order:-1:0))')';
    M = zeros(n, order+1);
    for (i = order:-1:0)
        M(:,order-i+1) = x .^ i;
    end
    y = M * coeff;