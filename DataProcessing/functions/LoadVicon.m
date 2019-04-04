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
    out.pos = log.ObjData.XYZ;
    out.quality = log.ObjData.Quality;

    % Perform quaternion correction (flip based on scalar value)
    signSwapIdx = [1; (find(diff(log.ObjData.Quaternion(:,1)) > 0.5)+1); length(log.ObjData.Time)];
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
    out.yaw = eul(:,1);
    
    % Compute linear velocity by numerical differentiation
    dxy_time = log.ObjData.Time(1:end-1) + (diff(log.ObjData.Time)/2);
    dxy = diff(log.ObjData.XYZ) ./ diff(log.ObjData.Time);
    % Interpolate qdot to necessary time locations
    out.velocity = interp1(dxy_time, dxy, out.time, 'linear', 0);
    
    % Compute quaternion derivative by numerical differentiation
    dq_time = log.ObjData.Time(1:end-1) + (diff(log.ObjData.Time)/2);
    dq = diff(log.ObjData.Quaternion) ./ diff(log.ObjData.Time);
    % Interpolate qdot to necessary time locations
    out.dq = interp1(dq_time, dq, out.time, 'linear', 0);

    % Compute angular velocity in body and inertial frame (based on numerical derivative of quaternion)
    out.omega_body = zeros(length(out.q), 3);
    out.omega_inertial = zeros(length(out.q), 3);
    for (i = 1:length(out.q))
        out.omega_body(i,:) = ( 2 * devec * Phi(out.q(i,:)') * out.dq(i,:)' )';
        out.omega_inertial(i,:) = ( 2 * devec * Gamma(out.q(i,:)') * out.dq(i,:)' )';
    end    
    
end