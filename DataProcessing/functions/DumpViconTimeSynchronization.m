function [dump, vicon] = DumpViconTimeSynchronization(dumpData, viconData)

    figure(10); 

    % Find peak time in Vicon data
    dq_vicon_time = viconData.time(1:end-1) + (diff(viconData.time)/2);
    dq4_vicon = diff(viconData.q(:,4)) ./ diff(viconData.time);
    [PKS,LOCS] = findpeaks(abs(dq4_vicon), 'MinPeakHeight', 1, 'MinPeakProminence', 0.2);    
    peakTimes = dq_vicon_time(LOCS-1);
    viconSyncTime = peakTimes(1);
    
    plot(dq_vicon_time, abs(dq4_vicon));
    hold on;   
    plot(dq_vicon_time(LOCS(1)), PKS(1), 'bx');
    
    % Find peak time in dump data
    [PKS,LOCS] = findpeaks(abs(dumpData.mti_dq(:,4)), 'MinPeakHeight', 1, 'MinPeakProminence', 0.2);
    peakTimes = dumpData.time(LOCS-1);
    dumpSyncTime = peakTimes(1);  
    
    plot(dumpData.time, abs(dumpData.mti_dq(:,4)));    
    plot(dumpData.time(LOCS(1)), PKS(1), 'rx');
    hold off;      
    
    % Try to find synchronization offset with cross-correlation
    samplesTimeUsedForCrossCorrelation = 20; % use 20 seconds of samples to estimate synchronization
    dt = mean(diff(dumpData.time));
    t_min = min(dq_vicon_time(1), dumpData.time(end));
    t_max = max(dq_vicon_time(1), dumpData.time(end));
    %t = t_min:dt:t_max;    
    t = t_min:dt:(t_min+samplesTimeUsedForCrossCorrelation);    
    %t = [-t(end:-1:1); t; t(end)+dt+t];
    x = interp1(dq_vicon_time, dq4_vicon, t, 'linear', 0);
    y = interp1(dumpData.time, dumpData.mti_dq(:,4), t, 'linear', 0);           
    [C lags] = xcorr(x, y); %  ceil(maxTimeOffset/dt))        
    [pos idx] = max(C);       
    syncOffset = dt*lags(idx);
    
    figure(22);    
    subplot(3,1,1); plot(t, x, t, y); legend('Vicon', 'IMU');    
    subplot(3,1,2); stem(dt*lags,C); title('Synchronization lag'); xlabel('Delay [s]'); ylabel('Correlation');
    subplot(3,1,3); plot(t-syncOffset, x, t, y); legend('Vicon', 'IMU');    
    
    % Remove data prior to sync and slightly after (syncWidth)
    syncWidth = -1; % remove two seconds related to sync
    extractionLength = 0; % should only a part of the log be extracted?
    
    dump = [];
    dumpStartIdx = find(dumpData.time > (dumpSyncTime+syncWidth));
    dumpStartIdx = dumpStartIdx(1); 
    dumpEndIdx = length(dumpData.time);
    if (extractionLength > 0)
        dumpEndIdx = find(dumpData.time > (dumpSyncTime+syncWidth+extractionLength));
        dumpEndIdx = dumpEndIdx(1); 
    end
    f = fieldnames(dumpData);
    for i = 1:length(f)
        if (~isfield(dump, f{i}))
            dump.(f{i}) = dumpData.(f{i})(dumpStartIdx:dumpEndIdx,:);
        end
    end  
    
    vicon = [];
    viconStartIdx = find(viconData.time > (viconSyncTime+syncWidth));
    viconStartIdx = viconStartIdx(1);   
    viconEndIdx = length(viconData.time);
    if (extractionLength > 0)
        viconEndIdx = find(viconData.time > (viconSyncTime+syncWidth+extractionLength));
        viconEndIdx = viconEndIdx(1);     
    end
    f = fieldnames(viconData);
    for i = 1:length(f)
        if (~isfield(vicon, f{i}))
            vicon.(f{i}) = viconData.(f{i})(viconStartIdx:viconEndIdx,:);
        end
    end       
    
%     % Try to find synchronization offset with cross-correlation
%     dt = mean(diff(dump.time));
%     t_min = min(dq_vicon_time(viconStartIdx), dumpData.time(dumpStartIdx));
%     t_max = max(dq_vicon_time(viconEndIdx), dumpData.time(dumpEndIdx));
%     t = t_min:dt:t_max;    
%     %t = [-t(end:-1:1); t; t(end)+dt+t];
%     x = interp1(dq_vicon_time(viconStartIdx:viconEndIdx), abs(dq4_vicon(viconStartIdx:viconEndIdx)), t, 'linear', 0)';
%     y = interp1(dumpData.time(dumpStartIdx:dumpEndIdx), abs(dumpData.mti_dq(dumpStartIdx:dumpEndIdx,4)), t, 'linear', 0)';
%     maxTimeOffset = 10;
%     figure(20);    
%     if (viconSyncTime < dumpSyncTime)
%         disp('xcorr(x,y)');
%         [C lags] = xcorr(x, y); %  ceil(maxTimeOffset/dt))
%     else
%         disp('xcorr(y,x)');
%         [C lags] = xcorr(x, y); %  ceil(maxTimeOffset/dt))
%     end   
%     stem(lags,C);
%     [pos idx] = max(C);       
%     syncOffset = t(lags(idx)) - t(1)  
%     
%     figure(22);
%     cnt = (1:length(t))';
%     plot(t, x, t, y); legend('x', 'y');
    
    % Synchronize the time
    %dump.time = dump.time - dumpSyncTime;
    %vicon.time = vicon.time - viconSyncTime;
    vicon.time = vicon.time - syncOffset;    
    
    % And align the start time of one of them to be t=0
    offset = min(dump.time(1), vicon.time(1));
    dump.time = dump.time - offset;
    vicon.time = vicon.time - offset;

end