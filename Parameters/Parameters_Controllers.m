% General sample rate
SampleRate = 200; % 200 Hz controller sample rate
Ts = 1/SampleRate; 

% Torque output parameter
SaturationTorqueOfMaxOutputTorque = 0.8; % saturation percentage of maximum output torque

% Sliding Mode controller selection and gains
SlidingManifold = 1; % 0=q_dot inertial,  1=q_dot body,  2=Body angular velocity,  3=Inertial angular velocity
SwitchingLaw = 2; % 1=discontinous,  2=continous

AggressiveGains = true; % true = aggressive gains, false = non-aggressive gains
if (AggressiveGains)
    K = diag([15, 15, 6]);
    eta = [6; 6; 3];
    epsilon = [0.5; 0.5; 0.2];
else
    K = diag([6, 6, 6]);
    eta = [5; 5; 6];
    epsilon = [0.8; 0.8; 0.3];
end