% General sample rate
SampleRate = 200; % 200 Hz controller sample rate
Ts = 1/SampleRate; 

% Torque output parameter
SaturationTorqueOfMaxOutputTorque = 0.8; % saturation percentage of maximum output torque

%% Sliding Mode controller selection and gains
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

%% Velocity controller
VelocityLQR_IntegralEnabled = false;
VelocityLQR_PositionControlAtZeroVelocityReference = true;
VelocityLQR_MaximumKickinVelocity = 0.1; % m/s
VelocityLQR_IntegratorPowerupStabilizeTime = 3.0; % wait 3 seconds in the beginning for integrator to settle (and before allowing manual movement)
VelocityController_StabilizationDetectionVelocity = 0.2; % if the robot is pushed with a velocity of more than 0.2 m/s after the initialization time the initialization integrator will be disabled allowing manual movement