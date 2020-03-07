addpath('..');
addpath('C:\Users\Thomas\Documents\Kugle-MATLAB\DataProcessing\functions');

DumpFolder = [pwd '/'];
data = LoadDump(DumpFolder, '', 10);
data = Downsample(data, 10);

%%
figure(1);
subplot(3,2,1); plot(data.time, data.raw_accelerometer(:,1), data.time, data.mti_accelerometer(:,1)); title('Accelerometer X');  xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); legend('MPU-9250', 'MTI-200');
subplot(3,2,3); plot(data.time, data.raw_accelerometer(:,2), data.time, data.mti_accelerometer(:,2)); title('Accelerometer Y');  xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); legend('MPU-9250', 'MTI-200');
subplot(3,2,5); plot(data.time, data.raw_accelerometer(:,3), data.time, data.mti_accelerometer(:,3)); title('Accelerometer Z'); xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); legend('MPU-9250', 'MTI-200');

subplot(3,2,2); plot(data.time, data.raw_gyroscope(:,1), data.time, data.mti_gyroscope(:,1)); title('Gyroscope X'); xlabel('Time [s]'); ylabel('Angular velocity [rad/s]'); legend('MPU-9250', 'MTI-200');
subplot(3,2,4); plot(data.time, data.raw_gyroscope(:,2), data.time, data.mti_gyroscope(:,2)); title('Gyroscope Y'); xlabel('Time [s]'); ylabel('Angular velocity [rad/s]'); legend('MPU-9250', 'MTI-200');
subplot(3,2,6); plot(data.time, data.raw_gyroscope(:,3), data.time, data.mti_gyroscope(:,3)); title('Gyroscope Z'); xlabel('Time [s]'); ylabel('Angular velocity [rad/s]'); legend('MPU-9250', 'MTI-200');

%matlab2tikz('cov.tex')

out = [data.time, data.raw_accelerometer, data.raw_gyroscope, data.mti_accelerometer, data.mti_gyroscope];
headers = {'time', ...
    'raw_accelerometer_x', 'raw_accelerometer_y', 'raw_accelerometer_z', ...
    'raw_gyroscope_x', 'raw_gyroscope_y', 'raw_gyroscope_z', ...
    'mti_accelerometer_x', 'mti_accelerometer_y', 'mti_accelerometer_z', ...
    'mti_gyroscope_x', 'mti_gyroscope_y', 'mti_gyroscope_z'};
csvwrite_with_headers('out.csv', out, headers);