function out = Trim(in, timeStart, varargin)
    if (length(varargin) > 0)
        extractionLength = varargin{1};
    else
        extractionLength = inf;
    end

    % trim output data
    idxStart = find(in.time >= timeStart); idxStart = idxStart(1);
    idxEnd = find(in.time <= timeStart+extractionLength); idxEnd = idxEnd(end);

    if ((idxStart < 0) || (idxStart > length(in.time)))
        error('Could not trim data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(in.time)) || (idxEnd < idxStart))
        error('Could not trim data: Incorrect end index');
    end    

    out = [];
    f = fieldnames(in);
    for i = 1:length(f)
        out.(f{i}) = in.(f{i})(idxStart:idxEnd,:);    
    end

    out.time = out.time - out.time(1); % correct output time to match trimmed away time
    
end