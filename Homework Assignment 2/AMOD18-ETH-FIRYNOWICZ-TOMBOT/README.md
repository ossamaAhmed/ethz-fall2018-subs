# Comments

Every time I did the system identification procedure my robot did not go straight but turned right, every time exactly the same way. I looked at the code and it seems that in publish_control.py manual trim configuration is not taken into account and raw motor commands are published, as if trim was set to 0. In my case, with trim 0, the robot turns right so much, that the calibration checkerboard is visible for very short time, which makes calibration results useless. I can't see how to make it work without including manually set trim parameter in system identification procedure.
