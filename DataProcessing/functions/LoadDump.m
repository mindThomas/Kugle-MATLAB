function out = LoadDump(DumpFolder, file)

if (DumpFolder(end) ~= '/')
    DumpFolder = [DumpFolder '/'];
end

if (isempty(file))
    file = getlatestfile([DumpFolder '*.txt']);
end

DumpFile = [DumpFolder, file];
data = ParseDumpArray(dlmread(DumpFile, '\t'));
DumpFile = [DumpFolder, 'sensor/', file];
sensor = ParseSensorDumpArray(dlmread(DumpFile, '\t'));
DumpFile = [DumpFolder, 'covariance/', file];
if (exist(DumpFile))
    covariance = ParseCovarianceDumpArray(dlmread(DumpFile, '\t'));
else
    covariance = [];
end

dataLength = min(length(data.time), length(sensor.time));
if (~isempty(covariance))
    dataLength = min(dataLength, length(covariance.time));
end
dataStart = 1;
sensorStart = 1;

if (data.time(dataStart:dataStart+dataLength-1) ~= sensor.time(sensorStart:sensorStart+dataLength-1))
    dataStart = 2;
    sensorStart = 1;
    dataLength = dataLength - 1;
    if (data.time(dataStart:dataStart+dataLength-1) ~= sensor.time(sensorStart:sensorStart+dataLength-1))
        dataStart = 1;
        sensorStart = 2;
        if (data.time(dataStart:dataStart+dataLength-1) ~= sensor.time(sensorStart:sensorStart+dataLength-1))
            error('Misalignment detected in received data - needs manual correction');
        end
    end
end

covarianceStart = 1;
if (~isempty(covariance))
    if (data.time(dataStart:dataStart+dataLength-1) ~= covariance.time(covarianceStart:covarianceStart+dataLength-1))                
        covarianceStart = 2;
        if (covarianceStart+dataLength-1 > length(covariance.time))
            dataLength = dataLength-1;
        end
        if (data.time(dataStart:dataStart+dataLength-1) ~= covariance.time(covarianceStart:covarianceStart+dataLength-1))
            dataStart = dataStart + 1;
            sensorStart = sensorStart + 1;
            dataLength = dataLength - 1;
            if (covarianceStart+dataLength-1 > length(covariance.time))
                dataLength = dataLength-1;
            end
            if (data.time(dataStart:dataStart+dataLength-1) ~= covariance.time(covarianceStart:covarianceStart+dataLength-1))
                dataStart
                sensorStart
                data.time(dataStart)
                covariance.time(covarianceStart)
                error('Misalignment detected in received data - needs manual correction');
            end
        end
    end
end

out = [];
f = fieldnames(data);
for i = 1:length(f)
    if (~isfield(out, f{i}))
        out.(f{i}) = data.(f{i})(dataStart:dataStart+dataLength-1,:);    
    end
end

f = fieldnames(sensor);
for i = 1:length(f)
    if (~isfield(out, f{i}))
        out.(f{i}) = sensor.(f{i})(sensorStart:sensorStart+dataLength-1,:);
    end
end

if (~isempty(covariance))
    f = fieldnames(covariance);
    for i = 1:length(f)
        if (~isfield(out, f{i}))
            out.(f{i}) = covariance.(f{i})(covarianceStart:covarianceStart+dataLength-1,:);
        end
    end
end

out.time = out.time - out.time(1); % subtract initial timestamp to start from 0

end