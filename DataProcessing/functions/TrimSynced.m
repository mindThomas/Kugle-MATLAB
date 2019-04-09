function [dumpOut, viconOut] = TrimSynced(dump, vicon, timeStart, varargin)
    if (length(varargin) > 0)
        extractionLength = varargin{1};
    else
        extractionLength = inf;
    end

    % Trim dump
    idxStart = find(dump.time >= timeStart); idxStart = idxStart(1);
    idxEnd = find(dump.time <= timeStart+extractionLength); idxEnd = idxEnd(end);

    if ((idxStart < 0) || (idxStart > length(dump.time)))
        error('Could not trim data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(dump.time)) || (idxEnd < idxStart))
        error('Could not trim data: Incorrect end index');
    end    

    dumpOut = [];
    f = fieldnames(dump);
    for i = 1:length(f)
        dumpOut.(f{i}) = dump.(f{i})(idxStart:idxEnd,:);    
    end

    % Trim vicon
    idxStart = find(vicon.time >= timeStart); idxStart = idxStart(1);
    idxEnd = find(vicon.time <= timeStart+extractionLength); idxEnd = idxEnd(end);

    if ((idxStart < 0) || (idxStart > length(vicon.time)))
        error('Could not trim data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(vicon.time)) || (idxEnd < idxStart))
        error('Could not trim data: Incorrect end index');
    end    

    viconOut = [];
    f = fieldnames(vicon);
    for i = 1:length(f)
        viconOut.(f{i}) = vicon.(f{i})(idxStart:idxEnd,:);    
    end    
    
    % And align the start time of one of them to be t=0
    offset = min(dumpOut.time(1), viconOut.time(1));
    dumpOut.time = dumpOut.time - offset;
    viconOut.time = viconOut.time - offset;
    
end