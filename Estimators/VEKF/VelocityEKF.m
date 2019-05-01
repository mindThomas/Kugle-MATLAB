function [X_out, P_out] = VelocityEKF(X, P_prev, EncoderDiffMeas, eta_encoder, Accelerometer, cov_acc, eta_accelerometer, eta_bias, qQEKF, cov_qQEKF, qdotQEKF, eta_acceleration, SamplePeriod, TicksPrRev, rk,rw,g)  %#codegen
    
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
		  
    dt = SamplePeriod;         
    dx = X(1);
    dy = X(2);
    ddx = X(3);
    ddy = X(4);
    acc_bias = X(5:7);
    
	% motor mapping (inverse kinematics)
	alpha = deg2rad(45);
	gamma = deg2rad(120);

	e1 = [1,0,0]';
	e2 = [0,1,0]';
	e3 = [0,0,1]';
	R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1];
	R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0];

	W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1);
	W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2);
	W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3);
	W = [W1;W2;W3];	       
       
    %% Prediction step  
    X_apriori = zeros(7,1);
          
    % Rotate acceleration measurement from body frame into inertial frame
    acceleration = devec * Phi(qQEKF) * Gamma(qQEKF)' * [0;Accelerometer];
    
    % Propagate the velocity based on acceleration
    dx_apriori = dx + dt * ddx;
    dy_apriori = dy + dt * ddy;
    
    ddx_apriori = ddx;
    ddy_apriori = ddy;
    
    acc_bias_apriori = acc_bias;
    
    % Setup process covariance
    Q = diag([zeros(1,2), eta_acceleration*ones(1,2), eta_bias*ones(1,3)]);
    
    % Determine model Jacobian (F)
    F_prev = eye(7);   
    F_prev(1:2,3:4) = dt * eye(2);
    
    % Set apriori state
    X_apriori = [dx_apriori
				 dy_apriori;
                 ddx_apriori;
                 ddy_apriori;
                 acc_bias_apriori];
             
    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q;
    
    %% Update/correction step   
    z = [EncoderDiffMeas; Accelerometer];          
    
	% Encoder Measurement model  
    dx_ball_apriori = dx_apriori;
    dy_ball_apriori = dy_apriori;

    dpsi_apriori = W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); % InverseKinematics(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dx_ball_apriori,dy_ball_apriori,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw);
    z_encoder_hat = TicksPrRev/(2*pi) * dt * dpsi_apriori;
	    
    % Accelerometer measurement model    
    z_accelerometer_hat = devec * Phi(qQEKF)' * Gamma(qQEKF) * [0;ddx_apriori;ddy_apriori;g] + acc_bias_apriori; % Rotate acceleration from inertial frame into body frame    
    
    % Assemble measurement prediction vector
    z_hat = [z_encoder_hat; z_accelerometer_hat];
    
	% Measurement Jacobian	
	H = zeros(6,7);
	H(1:3,1) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; % d encoder_meas  /  d dx_2L
	H(1:3,2) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; % d encoder_meas  /  d dy_2L
	H(4:6,3:4) = devec * Phi(qQEKF)' * Gamma(qQEKF) * [zeros(1,2);eye(2);zeros(1,2)];
    H(4:6,5:7) = eye(3);
    
	% Measurement covariances	    
	cov_quantization = 0.5 * eye(3);    
    cov_encoder = eta_encoder * 4*cov_quantization;    
    
    daccelerometer_dqQEKF = devec * Phi(qQEKF)' * Phi([0;Accelerometer]) + devec * Gamma(qQEKF) * Gamma([0;Accelerometer]) * I_conj;
    R_accelerometer = eta_accelerometer*cov_acc + daccelerometer_dqQEKF * cov_qQEKF * daccelerometer_dqQEKF';
    
    % Setup measurement covariance
	R = [cov_encoder, zeros(3,3);   
         zeros(3,3),  R_accelerometer];
    
    % Calculate Kalman gain
    S = H * P_apriori * H' + R;
    %K = P_apriori * H' * inv(S);
    K = P_apriori * H' / S;

    % Correct using innovation
    X_aposteriori = X_apriori + K * (z - z_hat);       
    P_aposteriori = (eye(7) - K*H) * P_apriori;
    
    %% Send output to Simulink
    X_out = X_aposteriori;  
    P_out = P_aposteriori;     
end