function [X_out, P_out] = QEKF(X, P_prev, Gyroscope, Accelerometer, Heading, UseHeadingForCorrection, SamplePeriod, SensorDriven, BiasEstimationEnabled, YawBiasEstimationEnabled, NormalizeAccelerometer, cov_gyro, cov_acc, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias, AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau, AccelerometerVibrationCovarianceVaryFactor, MaxVaryFactor, g)  %#codegen
    
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

    % Split state vector, X[k-1], into individual variables    
    q1 = X(1);
    q2 = X(2);
    q3 = X(3);
    q4 = X(4);    
    q = [q1, q2, q3, q4]';    
    
    omega = X(5:7);    
    
    gyro_bias_x = X(8);
    gyro_bias_y = X(9);
    gyro_bias_z = X(10);
    gyro_bias = [gyro_bias_x; gyro_bias_y; gyro_bias_z];
    
    gyro_x = Gyroscope(1);
    gyro_y = Gyroscope(2);
    gyro_z = Gyroscope(3);
    gyro_input = [gyro_x; gyro_y; gyro_z];    
    
    BiasEstimationEnabledMat = single(zeros(3,3));
    BiasEstimationEnabledMat(1,1) = BiasEstimationEnabled;
    BiasEstimationEnabledMat(2,2) = BiasEstimationEnabled;
    BiasEstimationEnabledMat(3,3) = BiasEstimationEnabled*YawBiasEstimationEnabled;      
    
    % Process covariances
    cov_q = single(zeros(4,4)); % quaternion kinematics is correct since we propagate with Quaternion exponential
    cov_omega = sigma2_omega * eye(3);    
    cov_bias = sigma2_bias*dt*BiasEstimationEnabledMat + zeros(3,3); % bias stays constant    
    
    % Vary gyroscope covariance based on accelerometer vibrations noise -
    % trust gyroscope more at high accelerometer vibrations
    persistent acc_norm_filtered;
    if (isempty(acc_norm_filtered))
        acc_norm_filtered = g;
    end
    
    persistent acc_norm_old;
    if (isempty(acc_norm_old))
        acc_norm_old = g;
    end    
    
    if (AccelerometerVibrationDetectionEnabled)        
        coeff_b = 1/(2*AccelerometerVibrationNormLPFtau/dt + 1); % nominator
        coeff_a = 1/(2*AccelerometerVibrationNormLPFtau/dt + 1) - 2/(2 + dt/AccelerometerVibrationNormLPFtau); % denominator
        acc_norm_filtered = coeff_b*norm(Accelerometer) + coeff_b*acc_norm_old - coeff_a*acc_norm_filtered;
        acc_norm_old = norm(Accelerometer);

        %if (abs(acc_norm_filtered - g) > AccelerometerExaggerationAmount)
            VaryFactor = exp(AccelerometerVibrationCovarianceVaryFactor * (abs(acc_norm_filtered - g)));
            VaryFactor = min(VaryFactor, MaxVaryFactor);
            %cov_acc_ = cov_acc_ * VaryFactor;
            cov_omega = cov_omega / VaryFactor;     
        %end
        acc_norm_out = acc_norm_filtered;
    end
        
    % Setup covariance matrices
    Q = [cov_q, zeros(4,3), zeros(4,3);
         zeros(3,4), cov_omega, zeros(3,3);
         zeros(3,4), zeros(3,3), cov_bias];

    R = [GyroscopeTrustFactor*cov_acc, zeros(3,3)
         zeros(3,3), cov_gyro];
    
    % Measurement vector
    % Normalize accelerometer seems to give better results
    if (NormalizeAccelerometer)
        %norm_acc = norm(Accelerometer);
        %if (norm_acc > 0)% && norm_acc < 9.85)
        %    z_acc = Accelerometer / norm_acc;   
        %else
        %    z_acc = 0*Accelerometer;
        %end               
        z_acc = Accelerometer / g; % this is not actually a normalization, but rather a nominal normalization - but it seems to yield better results?
    else
        z_acc = Accelerometer;
    end
    
    z_gyro = gyro_input;
    
    z = [z_acc; z_gyro];   
    
    %% Prediction step  
    X_apriori = single(zeros(10,1));
     
    % Propagate quaternion
    if (SensorDriven)
        dq = 1/2 * Phi(q) * vec * (gyro_input - gyro_bias);            
    else
        dq = 1/2 * Phi(q) * vec * omega;   % and conversely:  omeg = 2 * devec * Phi(q)' * dq;     
    end
    q_apriori = q + dt * dq; % Forward Euler
    %q_apriori = Phi(q) * [1; 1/2*dt*omega]; % Delta-Quaternion integration
       
    % Propagate/set angular velocity states
    omega_apriori = omega;
    
    % Propagate gyro bias
    gyro_bias_apriori = gyro_bias;
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = single(zeros(10,10));
    F_prev(1:4,1:4) = eye(4) + dt * 1/2 * Gamma(vec*omega); %Gamma([1, 1/2*dt*omega]); % eye(4);
    F_prev(1:4,5:7) = dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)];
    F_prev(1:4,8:10) = zeros(4,3);
    F_prev(5:7,1:4) = zeros(3,4);
    F_prev(5:7,5:7) = eye(3);               
    F_prev(5:7,8:10) = zeros(3,3);
    F_prev(8:10,1:4) = zeros(3,4);
    F_prev(8:10,5:7) = zeros(3,3);
    F_prev(8:10,8:10) = eye(3);
    
    if (SensorDriven)
        % Change covariance matrix and model Jacobian
        F_prev(1:4,1:4) = eye(4);
        F_prev(1:4,5:7) = zeros(4,3);
        F_prev(1:4,8:10) = -dt * 1/2 * Phi(q) * [zeros(1,3); BiasEstimationEnabledMat];
        Q(1:4,1:4) = (dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)]) * cov_gyro * (dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)])';
    end
    
    % Set apriori state
    X_apriori(1:4) = q_apriori;
    X_apriori(5:7) = omega_apriori;
    X_apriori(8:10) = gyro_bias_apriori;    
    
    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q;           
    
    %% Update/correction step  
    if (NormalizeAccelerometer)
        % Accelerometer Measurement model    
        z_hat_acc = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-1];    
    
        % Measurement Jacobian
        H_acc = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-1])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-1])), ...
                 zeros(3,3), ...
                 zeros(3,3)];
    else
        % Accelerometer Measurement model    
        z_hat_acc = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-g];    
    
        % Measurement Jacobian
        H_acc = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-g])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-g])), ...
                 zeros(3,3), ...
                 zeros(3,3)];
    end
    
    z_hat_gyro = omega_apriori + gyro_bias_apriori;
    H_gyro = [zeros(3,4), ...
              eye(3), ...
              BiasEstimationEnabledMat];
    z_hat = [z_hat_acc; z_hat_gyro];    
    H = [H_acc; H_gyro];
    
    if (UseHeadingForCorrection)
        z_heading = Heading;
        heading_vector_hat = [zeros(2,1),eye(2),zeros(2,1)] * Phi(q_apriori) * Gamma(q_apriori)' * [0;1;0;0]; % estimated measurement
        z_hat_heading = atan2(heading_vector_hat(2), heading_vector_hat(1));
        % d atan2(y,x) / dx = -y / (x^2+y^2)
        % d atan2(y,x) / dy = x / (x^2+y^2)
        
        % d heading_hat / d q = (d atan2(y,x) / dx)*(d heading_vector_hat(1) / dq) + (d atan2(y,x) / dy)*(d heading_vector_hat(2) / dq)
        
        H_heading_vector = [[zeros(2,1),eye(2),zeros(2,1)] * (Phi(Phi(q_apriori)*[0;1;0;0])*I_conj + Gamma(Gamma(q_apriori)'*[0;1;0;0])), zeros(2,3), zeros(2,3)];        
        dAtan2dxy = [-heading_vector_hat(2) / (heading_vector_hat(1)^2+heading_vector_hat(2)^2), ...
                   heading_vector_hat(1) / (heading_vector_hat(1)^2+heading_vector_hat(2)^2)];
        H_heading = dAtan2dxy * H_heading_vector;
        
        % Heading measurement covariance
        cov_heading = sigma2_heading;
        
        z2 = [z; z_heading];
        z2_hat = [z_hat; z_hat_heading];
        H2 = [H; H_heading];          
        R2 = [R, zeros(6,1); zeros(1,6), cov_heading];
       
        % Calculate Kalman gain
        S2 = H2 * P_apriori * H2' + R2;        
        K2 = P_apriori * H2' / S2;  % P_apriori * H2' * inv(S2)
        
        %if (NormalizeAccelerometer && (norm_acc <= 0)) 
        %    K2 = single(zeros(size(K2))); % if there is no accelerometer measurement, then no correction will be performed
        %end
        
        % Correct using innovation
        X_aposteriori = X_apriori + K2 * (z2 - z2_hat);    
        P_aposteriori = (eye(10) - K2*H2) * P_apriori; 
    else
        % Calculate Kalman gain
        S = H * P_apriori * H' + R;        
        K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        
        
        %if (NormalizeAccelerometer && (norm_acc <= 0)) 
        %    K = single(zeros(size(K))); % if there is no accelerometer measurement, then no correction will be performed
        %end
        
        X_aposteriori = X_apriori + K * (z - z_hat);    
        P_aposteriori = (eye(10) - K*H) * P_apriori; 
    end
     
    %% Normalize quaternion
    X_aposteriori(1:4) = X_aposteriori(1:4) / norm(X_aposteriori(1:4));    
    
    %% Send output to Simulink
    X_out = X_aposteriori;
    P_out = P_aposteriori;
end