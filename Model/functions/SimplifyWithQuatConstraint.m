function simplifiedFun = SimplifyWithQuatConstraint(fun, q)
    simplifiedFun = subs(simplify(subs(fun, q(1)^2+q(2)^2+q(3)^2+q(4)^2, 1)), q(1)^2+q(2)^2+q(3)^2+q(4)^2, 1);