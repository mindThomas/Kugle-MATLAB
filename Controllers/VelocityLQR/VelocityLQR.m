% Compute balance LQR control action
% Remember to add the folder with the quaternion functions:
%   addpath('../../Misc');
% Note that:
%   'vel_ref' input is given in inertial frame
%   'omega_ref' output is given in body frame
function [omega_ref_xy, vel_err_integral] = VelocityLQR(X, q_ref, vel_err_integral_prev, vel_ref, dt, K, VelocityIntegralEnabled, PositionControlAtZeroVelocityReference, IntegratorPowerupStabilizeTime, StabilizationDetectionVelocity, MaximumKickinVelocityForPositionHold)

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

    %% Extract States
    x = X(1);
    y = X(2);
    q1 = X(3);
    q2 = X(4);
    q3 = X(5);
    q4 = X(6);

    dx = X(7);
    dy = X(8);
    dq1 = X(9);
    dq2 = X(10);
    dq3 = X(11);
    dq4 = X(12);

    xy = X(1:2);
    q = X(3:6);
    dq = X(9:12);
    dxy = X(7:8);
    dchi = reshape(X(7:12), 6, 1);

    %% Construct LQR state vector
    % X = [x,y,q2,q3,dx,dy,dq2,dq3,q2_ref,q3_ref]
    % Note that all parts has to be converted into heading frame (heading removed first)
    q_heading = HeadingQuaternion(q);
    q_tilt = Phi(q_heading)' * q;
    omega_body = devec * 2 * Phi(q)' * dq;
    dq_tilt = 1/2 * Phi(q_tilt) * vec * omega_body;   
    q_ref_heading = HeadingQuaternion(q_ref);
    q_ref_tilt = Phi(q_ref_heading)' * q_ref;
    
    persistent VelocityIntegralAfterPowerup;
    if (isempty(VelocityIntegralAfterPowerup))
        VelocityIntegralAfterPowerup = true;
    end    
    persistent VelocityIntegralInitializationTime;
    if (isempty(VelocityIntegralInitializationTime))
        VelocityIntegralInitializationTime = IntegratorPowerupStabilizeTime;
    end
    persistent pos_ref;    
    if (isempty(pos_ref))
        pos_ref = xy;        
    end
    persistent position_control_enabled;
    if (isempty(position_control_enabled))
        position_control_enabled = true;
    end
    
    % Update position reference if we are supposed to be moving
	if (position_control_enabled == false)
		pos_ref = xy;
    end
    
    % Enable position control (station keeping) if velocity reference is zero
    if (VelocityIntegralAfterPowerup == false && PositionControlAtZeroVelocityReference == true && norm(vel_ref) == 0 && (dxy'*dxy < MaximumKickinVelocityForPositionHold^2))
        position_control_enabled = true;               
    else
        position_control_enabled = false;
    end
    
	if (VelocityIntegralInitializationTime > 0)
		VelocityIntegralInitializationTime = VelocityIntegralInitializationTime - dt;
		if (VelocityIntegralInitializationTime < 0)
            VelocityIntegralInitializationTime = 0;
        end

		if (vel_ref'*vel_ref > 0) % velocity reference demands movement
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
        end        
    elseif (VelocityIntegralAfterPowerup)
		if ((vel_ref'*vel_ref > 0) || ...
			(dxy'*dxy > StabilizationDetectionVelocity^2)) % manual movement of velocity reference requires the initialization integral to be disabled
			VelocityIntegralInitializationTime = 0;
			VelocityIntegralAfterPowerup = false;
        end
    end
    
    pos_err = xy - pos_ref;
    pos_err_in_heading = [eye(2),zeros(2,1)] * devec * Phi(q_heading)' * Gamma(q_heading) * vec * [pos_err;0];
    
    vel_err = dxy - vel_ref;
    vel_err_in_heading = [eye(2),zeros(2,1)] * devec * Phi(q_heading)' * Gamma(q_heading) * vec * [vel_err;0];
    vel_err_integral_in_heading = vel_err_integral_prev;
            
    if (position_control_enabled) % enable position control
        Xve = [pos_err_in_heading + vel_err_integral_in_heading; q_tilt(2:3); vel_err_in_heading; dq_tilt(2:3); q_ref_tilt(2:3)];    
    else % velocity control only
        Xve = [vel_err_integral_in_heading; q_tilt(2:3); vel_err_in_heading; dq_tilt(2:3); q_ref_tilt(2:3)];    
    end
    
    % Compute control output by using LQR gain
    omega_ref_in_heading = -K*Xve;
    
    % Use control output in heading frame as omega_ref in body frame, since we only output angular velocity reference around x and y axis    
    omega_ref_xy = [omega_ref_in_heading(1);omega_ref_in_heading(2)];
    
    % Update integral
    vel_err_integral = vel_err_integral_prev;
    if (VelocityIntegralEnabled || VelocityIntegralAfterPowerup)
        vel_err_integral = vel_err_integral + dt * max(min(vel_err_in_heading, 0.05), -0.05);
    end