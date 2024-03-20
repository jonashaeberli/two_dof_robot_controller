from robot import robot
import numpy as np
import time


pick_and_place = robot()

pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=300, y= 50)
pick_and_place.hardware.move_to(pos.get("upper_arm_angle"), pos.get("lower_arm_angle"))