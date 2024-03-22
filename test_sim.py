from robot import robot
import numpy as np
import time

PnP = robot(run_sim = True)
PnP.controller.setup_controller()
angles = PnP.kinematics.kinematics(kinematics_type="inverse", x=250, y=100)
print (np.rad2deg(angles.get("upper_arm_angle")), np.rad2deg(angles.get("lower_arm_angle")))
while True:
    PnP.sim.move(angles.get("upper_arm_angle"), angles.get("lower_arm_angle"))
    time.sleep(0.05)
