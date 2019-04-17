function out = RemoveTime(in, tRemoveStart, tRemoveEnd)

    % trim output data
    idxStart = find(in.time >= tRemoveStart); idxStart = idxStart(1);
    idxEnd = find(in.time <= tRemoveEnd); idxEnd = idxEnd(end);

    if ((idxStart < 0) || (idxStart > length(in.time)))
        error('Could not remove data: Incorrect start index');
    end
    if ((idxEnd < 0) || (idxEnd > length(in.time)) || (idxEnd < idxStart))
        error('Could not remove data: Incorrect end index');
    end    

    out = in;
    f = fieldnames(out);
    for i = 1:length(f)
        out.(f{i})(idxStart:idxEnd,:) = [];  
    end
    
end