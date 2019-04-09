function out = Downsample(in, nSamplesToDiscard)

    out = [];
    f = fieldnames(in);
    for i = 1:length(f)
        out.(f{i}) = in.(f{i})(1:(nSamplesToDiscard+1):end,:);    
    end
    

end