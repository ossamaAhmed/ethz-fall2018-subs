Errorcode, same for all 3 conigurations:


Traceback (most recent call last):
  File "/home/tim/duckietown/Software/catkin_ws/src/05-teleop/calibration/src/kinematic_calibration.py", line 946, in <module>
    calib=calib()
  File "/home/tim/duckietown/Software/catkin_ws/src/05-teleop/calibration/src/kinematic_calibration.py", line 70, in __init__
    self.fit_=self.nonlinear_model_fit()
  File "/home/tim/duckietown/Software/catkin_ws/src/05-teleop/calibration/src/kinematic_calibration.py", line 783, in nonlinear_model_fit
    s_init_sine = [x_sine_meas[start],y_sine_meas[start], yaw_sine_meas[start]]
IndexError: index 0 is out of bounds for axis 0 with size 0
================================================================================REQUIRED process [kinematic_calibration-2] has died!
process has died [pid 13894, exit code 1, cmd /home/tim/duckietown/Software/catkin_ws/src/05-teleop/calibration/src/kinematic_calibration.py __name:=kinematic_calibration __log:=/home/tim/.ros/log/08713356-d93e-11e8-9896-b8e8564156d0/kinematic_calibration-2.log].
log file: /home/tim/.ros/log/08713356-d93e-11e8-9896-b8e8564156d0/kinematic_calibration-2*.log
Initiating shutdown!
================================================================================
[kinematic_calibration-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete

