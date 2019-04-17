function out = Downsample(in, nthSampleToKeep)

    if (isstruct(in))
        out = [];
        f = fieldnames(in);
        for i = 1:length(f)
            out.(f{i}) = in.(f{i})(1:nthSampleToKeep:end,:);    
        end
    else
        out = in(1:nthSampleToKeep:end,:);
    end
end