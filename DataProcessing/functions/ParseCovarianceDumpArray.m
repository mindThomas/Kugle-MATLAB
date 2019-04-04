function out = ParseCovarianceDumpArray(array)
    if (size(array,2) ~= 50) 
        error('Incorrect dump array size when parsing');
    end
    out.time = array(:,1);        
    out.Cov_q = array(:,2:2+16-1);
    out.Cov_dq = array(:,18:18+16-1);
    out.Cov_bias = array(:,34:34+9-1);
    out.Cov_velocity = array(:,43:43+4-1);
    out.Cov_COM = array(:,47:47+4-1);
end
