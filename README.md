# Kugle-MATLAB
MATLAB code and Simulink models for the model and controller development of the Kugle robot, including a non-linear Quaternion model, Sliding mode attitude controller and ACADO MPC for path following.

# Notes related to the report
When comparing this MATLAB code to the report, please note that the quaternion elements have been numbered from q1 to q4 instead of q0 to q3 as was done in the report.
This is to align the element with the indexing used by MATLAB, where the first element is at index 1.

# Required toolboxes
- Symbolic Toolbox
- Robotics System Toolbox    (used for eul2quat etc.)
- Simulink

# Other necessary MATLAB libraries
Peter Corkes Robotics Toolbox: http://petercorke.com/wordpress/toolboxes/robotics-toolbox
