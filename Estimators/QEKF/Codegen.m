clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Codegen'));
load('CoderConfig_ARM_CortexM.mat');

% Define input sizes and types
X = coder.typeof(single(0), [10 1], 0); % fixed size of floats
P_prev = coder.typeof(single(0), [10 10], 0);
Gyroscope = coder.typeof(single(0), [3 1], 0);
Accelerometer = coder.typeof(single(0), [3 1], 0);
Heading = coder.typeof(single(0));
UseHeadingForCorrection = coder.typeof(false);
SamplePeriod = coder.typeof(single(0));
SensorDriven = coder.typeof(false);
BiasEstimationEnabled = coder.typeof(false);
YawBiasEstimationEnabled = coder.typeof(false);
NormalizeAccelerometer = coder.typeof(false);
cov_gyro = coder.typeof(single(0), [3 3], 0);
cov_acc = coder.typeof(single(0), [3 3], 0);
GyroscopeTrustFactor = coder.typeof(single(0));
sigma2_omega = coder.typeof(single(0));
sigma2_heading = coder.typeof(single(0));
sigma2_bias = coder.typeof(single(0));
AccelerometerVibrationDetectionEnabled = coder.typeof(false);
AccelerometerVibrationNormLPFtau = coder.typeof(single(0));
AccelerometerVibrationCovarianceVaryFactor = coder.typeof(single(0));
MaxVaryFactor = coder.typeof(single(0));
g = coder.typeof(single(0));

% Convert QEKF to C++ code
codegen QEKF -config config -nargout 2 -args { X, P_prev, Gyroscope, Accelerometer, Heading, UseHeadingForCorrection, SamplePeriod, SensorDriven, BiasEstimationEnabled, YawBiasEstimationEnabled, NormalizeAccelerometer, cov_gyro, cov_acc, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias, AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau, AccelerometerVibrationCovarianceVaryFactor, MaxVaryFactor, g }