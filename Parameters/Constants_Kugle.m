Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
          q(2) q(1)  -q(4) q(3);
          q(3) q(4)  q(1)  -q(2);
          q(4) -q(3) q(2)  q(1)];           
Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
             p(2) p(1) p(4) -p(3);
             p(3) -p(4) p(1) p(2);
             p(4) p(3) -p(2) p(1)];  
devec = [0,1,0,0;0,0,1,0;0,0,0,1]; % 'v' in notes
vec = [0,0,0;1,0,0;0,1,0;0,0,1]; % '^' in notes
I_conj = diag([1,-1,-1,-1]);

% These constants have been updated to reflect the latest Kugle V1 prototype

g = 9.82;

alpha = deg2rad(45); % motor angle of attack on ball
gamma = deg2rad(120); % motor seperation (3 motors evenly seperated around 360 deg)

rk = 0.129;
Mk = 1.478;
coating = 0.004;
Jk = 2/3 * Mk * (rk-coating)^2;

rw = 0.05;
Mw = 0.270;
i_gear = 13/3; % gear ratio = 4.3 : 1   (https://www.maxonmotor.com/maxon/view/product/223081)
Jgear = 1.2e-6; % 12 g cm^2   (inertia motors see: https://support.maxonmotor.com/hc/en-us/articles/360006129633-Gearhead-Mass-inertia)
%Jow = 1/2 * Mw * rw^2; % 3.375 * 10^-4  but Rezero = 9*10^-4
Jow = 9 * 10^-4; % Rezero - assumed larger than disk equation, probably because type of wheel with larger mass concentration around perimeter
Jm = 1.21 * 10^-4;  % 1210 g cm^2
Jw = Jow + i_gear^2*(Jm+Jgear);

Mb = 14.31 + 1.844;

% Body center of mass defined with origin in ball center
SimulateWithAlignedCOM = true; % if set to true the COM is corrected to be aligned, similar to the calibration performed on the robot
COM_X = -0.02069e-3;
COM_Y = -3.20801e-3;
COM_Z = 550.23854e-3 - rk; % subtract rk since the values are extracted from OnShape with origin in contact point (bottom of ball)
COM = [COM_X, COM_Y, COM_Z]';

l = norm(COM); % distance to COM from ball center

% Moment of inertia around center of mass
Jb_COM = (1e-3)^2 * [
        1.29465e+6,     -6.43967e+1,    2.37297e+2
        -6.43967e+1,    1.30640e+6,     3.17317e+4
        2.37297e+2,     3.17317e+4,     1.00218e+5];
Jb_COM = rotz(deg2rad(-90)) * Jb_COM * rotz(deg2rad(-90))'; % since the x-axis of the OnShape CAD drawing points in the y-axis direction
    
% Shift moment of inertia to be around origin = ball center using
% generalized parallel axis theorem (https://en.wikipedia.org/wiki/Parallel_axis_theorem)
Jb = Jb_COM + Mb*(COM'*COM*eye(3) - COM*COM');

% OBS! Maybe the model should be enhanced/changed to allow non-diagonal inertias
% (constants should be the inertia tensor elements)
Jbx = Jb(1,1);
Jby = Jb(2,2);
Jbz = Jb(3,3);

% Center of rotation parameter - used in estimator
CoR = 0.8720; % (2.07 * l)   run "NonlinearModelAnalysis.m" script to get value

% Friction parameters
Bvk = 0.001;
Bvm = 0.001;
Bvb = 0.001;

% Motor limit
max_motor_current = 15; % ESCON 50/5 motor driver (https://www.maxonmotor.com/maxon/view/product/control/4-Q-Servokontroller/438725)
motor_constant = 30.5e-3; % Nm / A   (https://www.maxonmotor.com/maxon/view/product/412819)
max_motor_torque = motor_constant * max_motor_current; % Nm      
max_output_torque = i_gear * max_motor_torque;
motor_gear_efficiency = 0.91; % (https://www.maxonmotor.com/maxon/view/product/223081)

% Motor constants (not used with torque/current control)
% Calculate/determine motor constants
motor_nominal_voltage = 12; % V
motor_no_load_speed = 3710 * 2*pi/60; % RPM -> rad/s
motor_no_load_current = 0.671; % A
motor_nominal_speed = 3260 * 2*pi/60; % RPM -> rad/s
motor_nominal_torque = 0.231; % Nm
motor_nominal_current = 7.81; % A
motor_stall_torque = 2.850; % Nm
motor_starting_current = 93.5; % A (equal to stall current)

motor_Kt = motor_stall_torque / motor_starting_current;
motor_R = motor_nominal_voltage / motor_starting_current;

tau_no_load_friction = motor_Kt * motor_no_load_current;
motor_no_load_backemf = motor_nominal_voltage - motor_R * motor_no_load_current;
motor_Ke = motor_no_load_backemf / motor_no_load_speed;
% motor_nominal_backemf = motor_nominal_voltage - motor_R * motor_nominal_current;
% motor_Ke2 = motor_nominal_backemf / motor_nominal_speed;
% motor_Ke = (motor_Ke1+motor_Ke2)/2;
SpeedConstant = 1/motor_Ke* 60 / (2*pi);

% Motor velocity controller gains
Kp = 1000;

% Battery parameters
battery_voltage = 15; % 14-16V
battery_max_current = 8; % A continous discharge

% Encoder parameters
TicksPrRev = 4*4096 * i_gear; % Maxon encoder gives 4096 pulses pr. revolution, each with 4 edges (due to quadrature detection)   (https://www.maxonmotor.com/maxon/view/product/sensor/encoder/Induktive-Encoder/Encoder-MILE-512-4096/421988)

%% Correct COM for simulation
if (SimulateWithAlignedCOM)
    COM_X = 0;
    COM_Y = 0;
    COM_Z = l;
    COM = [COM_X; COM_Y; COM_Z];
end

%% Assemble constant vector used in simulation
constants = [Jk, Mk, rk, Mb, Jbx, Jby, Jbz, Jw, rw, Bvk, Bvm, Bvb, g]';