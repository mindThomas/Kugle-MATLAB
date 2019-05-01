clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '../../Codegen'));
load('CoderConfig_ARM_CortexM.mat');

% Define input sizes and types
X = coder.typeof(single(0), [7 1], 0); % fixed size of floats
P_prev = coder.typeof(single(0), [7 7], 0);
EncoderDiffMeas = coder.typeof(single(0), [3 1], 0);
eta_encoder = coder.typeof(single(0));
Accelerometer = coder.typeof(single(0), [3 1], 0);
cov_acc = coder.typeof(single(0), [3 3], 0);
eta_accelerometer = coder.typeof(single(0));
eta_bias = coder.typeof(single(0));
qQEKF = coder.typeof(single(0), [4 1], 0);
cov_qQEKF = coder.typeof(single(0), [4 4], 0);
qdotQEKF = coder.typeof(single(0), [4 1], 0);
eta_acceleration = coder.typeof(single(0));
SamplePeriod = coder.typeof(single(0));
TicksPrRev = coder.typeof(single(0));
rk = coder.typeof(single(0));
rw = coder.typeof(single(0));
g = coder.typeof(single(0));

% Convert QEKF to C++ code
codegen VelocityEKF -config config -nargout 2 -args { X, P_prev, EncoderDiffMeas, eta_encoder, Accelerometer, cov_acc, eta_accelerometer, eta_bias, qQEKF, cov_qQEKF, qdotQEKF, eta_acceleration, SamplePeriod, TicksPrRev, rk,rw,g }