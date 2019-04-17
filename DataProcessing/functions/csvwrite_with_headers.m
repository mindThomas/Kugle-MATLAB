% From https://se.mathworks.com/matlabcentral/fileexchange/29933-csv-with-column-headers
% Author: Keith Brady
% Version 1.0.0.0
%
% This function functions like the build in MATLAB function csvwrite but
% allows a row of headers to be easily inserted
%
% known limitations
% 	The same limitation that apply to the data structure that exist with 
%   csvwrite apply in this function, notably:
%       m must not be a cell array
%
% Inputs
%   
%   filename    - Output filename
%   m           - array of data
%   headers     - a cell array of strings containing the column headers. 
%                 The length must be the same as the number of columns in m.
%   r           - row offset of the data (optional parameter)
%   c           - column offset of the data (optional parameter)
%
%
% Outputs
%   None
function csvwrite_with_headers(filename,m,headers,r,c)

%% initial checks on the inputs
if ~ischar(filename)
    error('FILENAME must be a string');
end

% the r and c inputs are optional and need to be filled in if they are
% missing
if nargin < 4
    r = 0;
end
if nargin < 5
    c = 0;
end

if ~iscellstr(headers)
    error('Header must be cell array of strings')
end

 
if length(headers) ~= size(m,2)
    error('number of header entries must match the number of columns in the data')
end

%% write the header string to the file

%turn the headers into a single comma seperated string if it is a cell
%array, 
header_string = headers{1};
for i = 2:length(headers)
    header_string = [header_string,',',headers{i}];
end
%if the data has an offset shifting it right then blank commas must
%be inserted to match
if r>0
    for i=1:r
        header_string = [',',header_string];
    end
end

%write the string to a file
fid = fopen(filename,'w');
fprintf(fid,'%s\r\n',header_string);
fclose(fid);

%% write the append the data to the file

%
% Call dlmwrite with a comma as the delimiter
%
dlmwrite(filename, m,'-append','delimiter',',','roffset', r,'coffset',c);
