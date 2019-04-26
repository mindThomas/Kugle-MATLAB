clear all;
close all;

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'functions/polynomial'));

syms t x y real;

% Construct polynomial
coeff = [1,2,3,4,5]';
f = EvaluatePolynomialSymbolic(coeff, x)

% Test derivative
df = EvaluatePolynomialSymbolic(ComputeDerivativePolynomialCoefficients(coeff), x)
df2 = diff(f)
simplify(df == df2)

% Test squared
f2 = EvaluatePolynomialSymbolic(ComputeSquaredPolynomialCoefficients(coeff), x)
f2_2 = expand(f^2)
simplify(f2 == f2_2)

%% Compare arc curve length approximation methods with MATLAB-based integral
syms t real;
coeff_x = [0.0176
   -0.0724
    0.1045
   -0.0619
   -0.9868
    1.0007];
coeff_y = [0.0008
   -0.0875
    0.2621
   -0.2472
    0.0752
    0.1962];
f_x = EvaluatePolynomialSymbolic(coeff_x, t);
f_y = EvaluatePolynomialSymbolic(coeff_y, t);

df_x = diff(f_x);
df_y = diff(f_y);
Q = sqrt(df_x^2 + df_y^2);
Q_fun = matlabFunction(Q);

t = 0.4;
s1 = integral(Q_fun, 0, t)
s2 = ArcLengthApproximation(coeff_x, coeff_y, t)
s3 = ArcLengthApproximationNumerical(coeff_x, coeff_y, t, 0.0001)

%% Test minimum distance finders
xs_squared_coeff = ComputeSquaredPolynomialCoefficients(coeff_x);
ys_squared_coeff = ComputeSquaredPolynomialCoefficients(coeff_y);
dist_squared_coeff = xs_squared_coeff + ys_squared_coeff;

s = 0:0.01:2;
dist = EvaluatePolynomial(dist_squared_coeff, s');
figure(2);
plot(s, dist);
hold on;
PolynomialMinimumFinderNumerical(dist_squared_coeff, 1, 0.00001)
PolynomialMinimumFinderNumerical2(dist_squared_coeff, 1, 0.00001)
PolynomialMinimumFinder(dist_squared_coeff, 1, 0.00001)
hold off;