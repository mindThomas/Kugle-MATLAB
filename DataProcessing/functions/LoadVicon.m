function out = LoadVicon(ViconLogFolder, file)
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

    if (isempty(file))
        file = getlatestfile([ViconLogFolder '*.mat']);
    end

    ViconLogFile = [ViconLogFolder, file];
    log = load(ViconLogFile);

    % Prepare different outputs
    out.time = log.ObjData.Time;
    position_vicon_origin = log.ObjData.XYZ / 1000; % convert from millimeter to meter
    out.quality = log.ObjData.Quality;

    % Perform quaternion correction (flip based on scalar value)
%     idx = find(...
%                 (((sign(log.ObjData.Quaternion(1:(end-1),1)) ~= sign(log.ObjData.Quaternion(2:end,1))) + ...
%                 (sign(log.ObjData.Quaternion(1:(end-1),2)) ~= sign(log.ObjData.Quaternion(2:end,2))) + ...
%                 (sign(log.ObjData.Quaternion(1:(end-1),3)) ~= sign(log.ObjData.Quaternion(2:end,3))) + ...
%                 (sign(log.ObjData.Quaternion(1:(end-1),4)) ~= sign(log.ObjData.Quaternion(2:end,4)))) > 2) ...
%             );
    idx = find( ...
                ((abs(diff(log.ObjData.Quaternion(:,1))) > 0.5) | ...
                (abs(diff(log.ObjData.Quaternion(:,2))) > 0.5) | ...
                (abs(diff(log.ObjData.Quaternion(:,3))) > 0.5) | ...
                (abs(diff(log.ObjData.Quaternion(:,4))) > 0.5)) ...
               );
            
    signSwapIdx = [1; idx+1; length(log.ObjData.Time)];
    %signSwapIdx = [1; (find(abs(diff(log.ObjData.Quaternion(:,1))) > 0.5)+1); length(log.ObjData.Time)];
    currentSign = 1;
    q = log.ObjData.Quaternion;    
    for (i = 1:(length(signSwapIdx)-1))
        q(signSwapIdx(i):signSwapIdx(i+1),:) = currentSign * log.ObjData.Quaternion(signSwapIdx(i):signSwapIdx(i+1),:);
        currentSign = -currentSign;
    end
    out.q = q;

    % Compute Euler angles
    eul = quat2eul(out.q, 'ZYX');
    out.roll = eul(:,3);
    out.pitch = eul(:,2);
    out.yaw = (eul(:,1));
    
    % Compute linear velocity by numerical differentiation
    dxy_time = log.ObjData.Time(1:end-1) + (diff(log.ObjData.Time)/2);
    dxy = diff(log.ObjData.XYZ / 1000) ./ diff(log.ObjData.Time);
    % Interpolate velocity to necessary time locations
    velocity_vicon_origin = interp1(dxy_time, dxy, out.time, 'linear', 'extrap');
    
    % Compute quaternion derivative by numerical differentiation
    dq_time = log.ObjData.Time(1:end-1) + (diff(log.ObjData.Time)/2);
    dq = diff(out.q) ./ diff(log.ObjData.Time);
    % Interpolate qdot to necessary time locations
    out.dq = interp1(dq_time, dq, out.time, 'linear', 'extrap');

    % Compute angular velocity in body and inertial frame (based on numerical derivative of quaternion)
    out.omega_body = zeros(length(out.q), 3);
    out.omega_inertial = zeros(length(out.q), 3);
    for (i = 1:length(out.q))
        out.omega_body(i,:) = ( 2 * devec * Phi(out.q(i,:)')' * out.dq(i,:)' )';
        out.omega_inertial(i,:) = ( 2 * devec * Gamma(out.q(i,:)')' * out.dq(i,:)' )';
    end    
    
    % Compute position of ball center (origin frame of ballbot) by transforming
    B_p_vicon = [-9.9; 0; 911.6] / 1000;
    K_p_vicon = zeros(length(out.q), 3);
    dK_p_vicon = zeros(length(out.q), 3);
    for (i = 1:length(out.q))
        K_p_vicon(i,:) = ( devec * Phi(out.q(i,:)') * Gamma(out.q(i,:)')' * vec * B_p_vicon )';
        dK_p_vicon(i,:) = ( 2 * devec * Gamma(out.q(i,:)')' * Gamma(vec * B_p_vicon) * out.dq(i,:)' )';
    end
    % position_vicon_origin = I_O_K + K_p_vicon
    out.position = position_vicon_origin - K_p_vicon;
    
    % Compute linear velocity by numerical differentiation
    dxy_time = log.ObjData.Time(1:end-1) + (diff(log.ObjData.Time)/2);
    dxy = diff(out.position) ./ diff(log.ObjData.Time);
    % Interpolate qdot to necessary time locations
    %out.velocity = interp1(dxy_time, dxy, out.time, 'linear', 'extrap');
    
    out.velocity = velocity_vicon_origin - dK_p_vicon;
    
    % Compute heading velocity
    out.velocity_heading = zeros(length(out.q), 3);
    for (i = 1:length(out.yaw))
        out.velocity_heading(i,:) = ( rotz(out.yaw(i))'*out.velocity(i,:)' )';
    end
end