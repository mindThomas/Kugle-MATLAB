% Example script to illustrate how to optimize functions with MATLAB Coder
% by converting them into MEX

% Define inputs
A = coder.typeof(single(0),[3 3],0); % fixed size of floats

% Convert desired function to MEX (in this case 'svd')
codegen svd -nargout 3 -args { A }