To generate the C++ code for the MPC, [ACADO](http://acado.github.io/) is used.

The code are generated using the MATLAB interface, so ACADO needs to be installed and configured for use with MATLAB: http://acado.github.io/matlab_overview.html

The generated C++ code for the MPC is both used in the MPC simulation at [Kugle-Misc](https://github.com/mindThomas/Kugle-Misc/tree/master/src/MPC_Test), in the MPC ROS package at [Kugle-ROS](https://github.com/mindThomas/Kugle-ROS/tree/master/kugle_mpc), and it is compiled into a MEX file to be used in closed-loop Simulink simulations and other MATLAB simulations.