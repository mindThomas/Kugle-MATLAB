BaseFrequency = 0.1;
freqRate = 0.2;

ReferenceGenerationStep = (0:4000)';
SampleRate = 200;
dt = 1/SampleRate;

% f = a * sin(2*pi*freq(t)*t) = a * sin(g(t))
% g = 2*pi*freq(t)*t
% freq(t) = BaseFrequency + freqRate * t
% dfdt = a * (dfdg * dgdt) = a * cos(g(t)) * dgdt
% dgdt = 2*pi* (dfreqdt*t + freq(t)) = 2*pi* (freqRate*t + freq(t))

t = ReferenceGenerationStep/SampleRate;
freq = (BaseFrequency + freqRate*t);
g = 2*pi*freq.*t;
a = deg2rad(3);
f = a * sin(g);
dgdt = 2*pi* (freqRate*t + freq);
dfdt = a * cos(g) .* dgdt;


dfdt_numerical = diff(f) ./ diff(t);
t_dfdt = t(1:(end-1)) + diff(t)/2;

figure(1);
subplot(2,1,1);
plot(t, rad2deg(f));
subplot(2,1,2);
plot(t_dfdt, dfdt_numerical, t, dfdt);
