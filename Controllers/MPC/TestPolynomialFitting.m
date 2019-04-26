clear all;

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, 'functions/polynomial'));

data = [
    0.2000    0.2000
    0.3000    0.2000
    0.4000    0.2000
    0.5000    0.2000
    0.6000    0.3000
    0.7000    0.4000
    0.8000    0.5000
    0.9       0.6
    1.0       0.7
    ];    

% Fitting Parameters
order = 7;
EnforceBeginEndConstraint = false;
EnforceBeginEndAngleConstraint = true;

%% Do individual fitting
% Fit points to two polynomials, x(t) and y(t), using parameters t starting with t0=0 and spaced with the distance between each point (chordal parameterization)
deltaDistance = diff(data);
t = [0;cumsum(sqrt(deltaDistance(:,1).^2 + deltaDistance(:,2).^2))];   

% Fit x(t), y(t)
coeff_x = ConstrainedPolyFit(t, data(:,1), order, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint); % x(t)
coeff_y = ConstrainedPolyFit(t, data(:,2), order, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint); % y(t)

% Compare one of the fitted polynomials with the polynomial fitting method of MATLAB
coeff_x_MATLAB = polyfit(t, data(:,1), order)'
if (round(coeff_x, 3) == round(coeff_x_MATLAB, 3)) disp('Fitted polynomials are equal');
else disp('Fitted polynomials are NOT equal');
end

% Visualize fit
t1 = linspace(min(t),max(t));
fx = EvaluatePolynomial(coeff_x,t1);
fy = EvaluatePolynomial(coeff_y,t1);

figure(1);
subplot(2,1,1);
plot(t,data(:,1),'o')
hold on
plot(t1,fx,'r--')
hold off
legend('x','x_{fit}')
xlabel('t');

subplot(2,1,2);
plot(t,data(:,2),'o')
hold on
plot(t1,fy,'r--')
hold off
legend('y','y_{fit}')
xlabel('t');


%% Compute numerical approximation of arc curve length at the points, t, using the fitted polynomial
s = ArcLengthApproximation(coeff_x, coeff_y, t);
s_total = s(end); % total length of approximated trajectory

%% Fit t(s) polynomial using the approximate arc curve lengths
coeff_ts = ConstrainedPolyFit(s, t, order+1, true, false); % t(s)

%% Make 100 linearly seperated distance points from the mapping, t(s)
s_eval = linspace(0, s_total, 100)'; 
t_eval = EvaluatePolynomial(coeff_ts, s_eval);

%% Use the evenly spaced parameter values, t, to generate new points from the two t-parameter fitted polynomials, x_0,...,x_n = x(t_0),...,x(t_n) and same for y(t)
x_eval = EvaluatePolynomial(coeff_x, t_eval);
y_eval = EvaluatePolynomial(coeff_y, t_eval);

%% Fit x(s) and y(s) on the newly generated points, x_0,...,x_n and y_0,...,y_n using the evenly spaced distance parameters, s_0,...,s_n, as the parameter
coeff_xs = ConstrainedPolyFit(s_eval, x_eval, order+1, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);
coeff_ys = ConstrainedPolyFit(s_eval, y_eval, order+1, EnforceBeginEndConstraint, EnforceBeginEndAngleConstraint);    

%% Visualize fit
xs_eval = polyval(coeff_xs, s_eval);
ys_eval = polyval(coeff_ys, s_eval);

figure(2);
subplot(2,1,1);
plot(s_eval, x_eval, s_eval, xs_eval);
subplot(2,1,2);
plot(s_eval, y_eval, s_eval, ys_eval);

figure(3);
plot(data(:,1), data(:,2));
hold on;
plot(x_eval, y_eval);
plot(xs_eval, ys_eval);
hold off;
legend('Window trajectory', 't-approximation', 's-approximation');