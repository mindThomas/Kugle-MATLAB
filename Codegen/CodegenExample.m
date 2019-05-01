% Example script to illustrate how to convert functions with MATLAB Coder
% Note that conversion with MATLAB coder requires the "ARM Cortex-M" code
% replacement library to be installed, found within the "Embedded Coder
% Support Package for ARM Cortex-M Processors"
% https://se.mathworks.com/matlabcentral/fileexchange/43095-embedded-coder-support-package-for-arm-cortex-m-processors
%load('CoderConfig_ARM_CortexM.mat');
load('config.mat');

% Define inputs
A = coder.typeof(single(0),[3 3],0); % fixed size of floats

% Try changing the flag below - it does not affect generated code
config.RowMajor = true;
%config.RowMajor = false;

% Convert desired function (in this case 'svd')
codegen svd -config config -nargout 3 -args { A }