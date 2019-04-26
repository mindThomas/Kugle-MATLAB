scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));

% Test parameters
coeff_a = 2;
coeff_b = -15;
sigma = 10;

% Make test line with known relationship
x = linspace(0, 100, 101)';
y = coeff_a*x + coeff_b;

% Add noise
y = y + sigma*randn(length(x),1);

%% Perform least squares to estimate a and b
% Let the unknown parameter vector be defined as:
% x = [a,b]
n = length(x);
A = [x, ones(n,1)];
b = y;
Aeq = [0,1]; % test with line start should be in zero, hence requiring b=0
beq = 0;
params = ConstrainedLeastSquares(A, b, Aeq, beq, n*1000);

% Test the fit
y_fit = A * params;

% Compute estimation error
estimation_error = params - [coeff_a;coeff_b]

% Visualize line and fit
figure(1);
plot(x, y);
hold on;
plot(x, y_fit);
hold off;

ylim([-50, 250]);