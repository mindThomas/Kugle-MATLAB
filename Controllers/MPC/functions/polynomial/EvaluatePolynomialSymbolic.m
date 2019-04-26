% Evaluate a polynomial   ( similar to Y = polyval(P,X) )
% Can also be used to create a symbolic polynomial expression from the coefficients and the symbolic variable x
% But can not be used for code generation
function y = EvaluatePolynomialSymbolic(coeff, x)
    order = length(coeff) - 1;
    y = (coeff' * (x.^(order:-1:0))')';