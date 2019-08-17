# ROS bag
A ROS bag of a log captured while manually driving the Kugle V1 prototype using the Velocity LQR controller and a PS4 joystick can be downloaded from: https://www.dropbox.com/s/8ywmpacohq0nky7/kugle-v1_frb7_log.bag

Play the log by executing:
```bash
 roslaunch kugle_launch rosbag.launch bag:="${PWD}/kugle-v2_frb7_log.bag"
```

The corresponding raw measurements logged from the embedded firmware can be downloaded from: https://www.dropbox.com/s/6o4y08vinqji922/raw%2Broslog_joystick_driving.tar.gz
Use MATLAB to parse and visualize the logs with the [`LoadDump.m`](https://github.com/mindThomas/Kugle-MATLAB/blob/master/DataProcessing/functions/LoadDump.m) and [`VisualizeDump.m`](https://github.com/mindThomas/Kugle-MATLAB/blob/master/DataProcessing/functions/VisualizeDump.m) scripts.

# Validation measurements
A set of 28 test measurements recorded on the robot can be downloaded from: https://www.dropbox.com/s/489votquyz91vjj/MeasurementDataset.zip
This test set is used to test and validate the developed algorithms. All plots in the report are based on this test dataset.

## Measurement dataset
The measurement dataset contain the following 28 tests:

| Type | Test index | Test name | Description / notes |
|------|------|------|------|
|Model verification|**#1**|Kinematics verification|Robot moved around manually in Vicon
|QEKF Estimator|**#2**|MTI Sensor Covariance|Raw MTI IMU measurements at standstill
||**#3**|QEKF test without controller with MTI| Test QEKF by turning robot around<br>1. Hold robot upright for bias estimation for 15 seconds<br>2. Tilt robot 90 degree for z-axis bias estimation for 15 seconds<br>3. Hold robot upright for 15 seconds<br>4. Tilt roll +10 to +30 degrees<br>5. Tilt roll -10 to -30 degrees<br>6. Tilt pitch +10 to +30 degrees<br>7. Tilt pitch -10 to -30 degrees<br>8. Tilt forward right (roll and pitch positive)<br>9. Tilt forward left (roll negative, pitch positive)<br>10. Turn yaw +90 degrees while being upright<br>11. Arbitrary motion on all axis with roll and tilt less than 45 degrees
||**#4**|MPU-9250 Sensor Covariance|Raw MPU-9250 IMU measurements at standstill
||**#5**|QEKF test without controller with MPU-9250|Test QEKF by turning robot around. This test is similar to **#3** but is carried out with the MPU-9250 IMU
||**#6**|QEKF complete orientation test|Ensure that QEKF works for all type of rotations, including the Euler-angle singularities. Two people are carrying the ballbot to turn it upside down.<br>1. Start by holding robot upright for bias estimation for 15 seconds<br>2. Now start to turn the robot around such that roll and pitch rolls over 90 degrees, resulting in the robot being upside down for some time<br>3. Include all types of motion, also yaw.
||**#7**|QEKF test with balance controller|Robot is moved around manually and disturbed with sudden push. Sliding mode controller configured with qdot manifold and agressive gains.
||**#8**|QEKF test with velocity controller|Robot is moved around with joystick. Sliding mode controller configured with qdot manifold and non-agressive gains.
|Velocity Estimator|**#9**|Velocity estimator test with controller|Robot is moved around manually. Balance controller configured with qdot manifold and non-agressive gains.
|Balance Controller|**#10**|Sliding mode controller, upright, agressive|Upright balance test = Zero reference = unit quaternion reference. Sliding mode controller configured qdot sliding manifold and agressive gains.<br>1. Calibrate IMU just before test<br>2. Let the controller hold the balance<br>2. Excert some disturbances
||**#11**|Sliding mode, upright, non-agressive|Same as **#10** but with non-agressive sliding mode gains.
||**#12**|Balance LQR|Redo **#10** but with Balance LQR controller
||**#13**|Heading independent mode|Test of Heading independent mode of Quaternion mode with Sliding mode controller using qdot sliding manifold and agressive gains.<br>1. Manually turn the robot around while being in quaternion mode<br>2. Note how the balance is kept but yaw is not actuated
||**#14**|Sliding mode, step test, qdot manifold|Reference step test with the Sliding mode controller using the qdot sliding manifold and non-agressive gains<br>1. RPY = 0, 0, 0 deg<br>2. RPY = 5, 0, 0 deg<br>3. RPY = 0, 0, 0 deg<br>4. RPY = 0, 5, 0 deg<br>5. RPY = 0, 0, 0 deg<br>6. RPY = -5, 5, 0 deg<br>7. RPY = 0, 0, 0 deg<br>8. RPY = 0, 0, 45 deg<br>9. RPY = 0, 0, 0 deg<br>
||**#15**|Sliding mode, step test, angular velocity manifold|Redo test **#14** but with angular velocity sliding manifold and non-agressive gains scaled with a factor of x2.0
||**#16**|Sliding mode, sine wave response, roll|±3 deg, 0.5 Hz sine wave on roll using qdot manifold and agressive gains
||**#17**|Sliding mode, sine wave response, increasing roll, qdot manifold|±2 deg sine wave on roll with increasing frequency (starting from 0.5 Hz), using qdot manifold and agressive gains _(wheel slip and motor error detector disabled)_
||**#18**|Sliding mode, sine wave response, increasing roll, angular velocity manifold|±2 deg sine wave on roll with increasing frequency (starting from 0.5 Hz), using angular velocity sliding manifold and non-agressive gains scaled with a factor of x2.0 _(wheel slip and motor error detector disabled)_
||**#19**|Sliding mode, rotating inclination, agressive|±3 deg inclination which is then rotated with increasing angular velocity around inertial z-axis starting from 0 rad/s, using qdot sliding manifold and **agressive gains** (wheel slip and motor error detector disabled)
||**#20**|Sliding mode, rotating inclination, non-agressive|±3 deg inclination which is then rotated with increasing angular velocity around inertial z-axis starting from 0 rad/s, using qdot sliding manifold and **non-agressive gains** (wheel slip and motor error detector disabled)
||**#21**|Sliding mode gains, epsilon|Small yaw epsilon to start stable oscillation/limitcycle<br>1. Start with non-agressive gains<br>2. Then lower yaw epsilon
||**#22**|Sliding mode gains, K|K gain tested on yaw/z-axis<br>1. Start with agressive gains and manually turn the robot 10-20 degrees around yaw and wait for it to come back<br>2. Increase K gain and manually turn again and wait for it to come back<br>3. Lower K gain and manually turn again and wait for it to come back (response time to come back should now be much slower)
||**#23**|Sliding mode gains, eta|eta gain tested on yaw/z-axis<br>1. Start with agressive gains and manually turn the robot 10-20 degrees around yaw and wait for it to come back<br>2. Increase eta gain and manually turn again and wait for it to come back<br>3. Lower eta gain and manually turn again and wait for it to come back
||**#24**|Heading independent quaternion control|Heading independent mode enabled. Sliding mode controller with qdot sliding manifold and agressive gains. Zero reference = upright
|Velocity Controller|**#25**|Velocity control, agressive|Velocity controller with zero velocity reference and sliding mode balance controller using qdot sliding manifold and **agressive** gains
||**#26**|Velocity LQR, non-agressive|Velocity controller with zero velocity reference and sliding mode balance controller using qdot sliding manifold and **non-agressive** gains<br>1. Start controller and let the position stabilize<br>2. Drag detection enabled, such that position reference is updated if the robot velocity is pushed above a certain threshold – thus when the robot is deliberately dragged to another position which is then kept
||**#27**|Velocity LQR, station keeping|Same as **#26** but with drag velocity threshold so high that the robot always tries to stay/come back to the initial position.
||**#28**|Joystick velocity control|Velocity reference input through joystick control with Velocity LQR controller, setting angular velocity reference and quaternion reference to sliding mode controller using qdot sliding manifold and non-agressive gains<br>1. Drive forward<br>2. Drive backward<br>3. Drive right<br>4. Drive left<br>5. Turn yaw left and right<br>6. Drive forward while turning counter-clockwise

_Note that all test includes a synchronization "twist" around the yaw axis in the beginning, whereafter the controllers and estimators are reset._
