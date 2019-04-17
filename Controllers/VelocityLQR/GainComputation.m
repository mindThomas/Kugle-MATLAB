addpath('../../Parameters');
addpath('../../Misc'); % for quaternion functions
load('../../Linearization/generated/ClosedLoopModelMatrices_SlidingMode_WithOmegaRef.mat');
Constants_Kugle;
Parameters_General % load sample rate
Parameters_Controllers % load controller parameters

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

%% Construct LQR error dynamics for velocity constrolled
% Create a state vector for the LQR consisting of
% X = [x,y,q2,q3,dx,dy,dq2,dq3,q2_ref,q3_ref]   - x,y is included either to be used for position control or for velocity error integral
stateIdx = [1,2,4,5,7,8,10,11,14,15]'; % extract states from linearized closed loop model   (ClosedLoopModelMatrices_SlidingMode_WithOmegaRef)
inputIdx = [1,2]; % omega_ref_x and omega_ref_y
A_cl = A(stateIdx,stateIdx);
B_cl = B(stateIdx,inputIdx);

%% Now create LQR for this reduced system
Q = diag([20, 20, ... % x, y
          0.01, 0.01, ... % q2, q3
          10, 10, ... % dx, dy
          0.1, 0.1, ... % dq2, dq3
          0.01, 0.01, ... % q2_ref, q3_ref
          ]);
R = 20 * diag([1 1]); % omega_ref_x, omega_ref_y

[K, S, e] = lqr(A_cl, B_cl, Q, R);
disp(regexprep( mat2str(K), {'\[', '\]', '\s+', '\;'}, {'', '', ',\t', ',\n'}))
disp(' ');

% Save gain
VelocityLQRgain = K;
save('generated/VelocityLQRgain', 'VelocityLQRgain');

Kvel = K;

%% Test velocity controller
% Velocity reference
vel = 1.0;
vref = repmat([vel,0], [length(t),1]);

% LQR gain
Kvel = K;
if (norm(vref) > 0)
    Kvel(1,2) = 0; Kvel(2,1) = 0; % disable position control 
end

% Construct closed loop system with velocity controller
B_vel_cl = B_cl*Kvel*[zeros(2,2);zeros(2,2);eye(2);zeros(4,2)];
C_test = eye(10);
D_test = zeros(10,2);
sys_vel_cl = ss((A_cl-B_cl*Kvel), B_vel_cl, C_test, D_test);
t = (0:0.01:5)';

% Initial conditions for test
xy0 = [0,0]';
q0 = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
dxy0 = [0,0.4]'; % non-zero initial velocity
dq0 = [0,0,0,0]';
q_ref0 = eul2quat(deg2rad([0, 0, 0]), 'ZYX')';
x0 = [xy0;q0(2:3);dxy0;dq0(2:3);q_ref0(2:3)];

% Simulate and plot
y = lsim(sys_vel_cl, vref, t, x0);
figure(1);
subplot(2,1,1);
plot(t, y(:,5)); title('Velocity X');
subplot(2,1,2);
plot(t, y(:,6)); title('Velocity Y');

figure(2);
subplot(2,1,1);
plot(t, rad2deg(2*y(:,9))); title('q2_{ref}'); ylabel('deg');
subplot(2,1,2);
plot(t, rad2deg(2*y(:,10))); title('q3_{ref}'); ylabel('deg');

omega_ref = -( Kvel*y' - Kvel(:,5:6)*vref' )';
figure(3);
subplot(2,1,1);
plot(t, omega_ref(:,1)); title('omega_{ref_x}'); ylabel('rad/s');
subplot(2,1,2);
plot(t, omega_ref(:,2)); title('omega_{ref_y}'); ylabel('rad/s');

%% Test gains with same initial conditions
X = [xy0;q0;dxy0;dq0];
vel_err_integral_prev = [0,0]';

vel_ref = vref(1,:)';

omega_ref_xy = VelocityLQR(X, q_ref0, vel_err_integral_prev, vel_ref, Ts, K)

%% Pole/zero analysis
C = [0,0,0,0,1,0,0,0,0,0;
     0,0,0,0,0,1,0,0,0,0];

sys = ss(A_cl, B_cl, C, zeros(2,2));
[Poles, Zeros] = pzmap(sys)
pzmap(sys)