% x = ConstrainedLeastSquares(A, b, Aeq, beq, lambda)
%
% Fit linear function
% c2*x^2 + c1*x + c0 = y
% Perform through least squares as: min |Ax - y|^2 
% x = [c2, c1, c0]'
% A = [x1^2, x1, 1; x2^2, x2, 1; ...];
% Which is the same as using the pseudo-inverse of A: A^+
% x_hat = A^+ y
function x = ConstrainedLeastSquares(A, b, Aeq, beq, lambda)
    
    if (size(Aeq,1) == size(beq,1) && size(A,2) == size(Aeq,2))
        A_ = [A; lambda*Aeq];
        b_ = [b; lambda*beq];
    else
        A_ = A;
        b_ = b;
    end

    % Pseudo-inverse through numerically unstable way
    %A_invpseudo = inv(A_'*A_) * A_'
    % Pseudo-inverse through MATLAB
    %A_invpseudo2 = pinv(A_)  % Moore-Penrose Pseudoinverse of matrix of A
    % Pseudo-inverse through SVD
    [U,S,V] = svd(A_);
    Sinv = [diag(1./diag(S)), zeros(size(S,2), size(S,1)-size(S,2))];
    A_invpseudo3 = V * Sinv * U';

    x = A_invpseudo3 * b_;