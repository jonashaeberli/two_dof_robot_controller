from two_dof_pick_and_place import two_dof_pick_and_place
import numpy as np


robot = two_dof_pick_and_place(250, 300, 100, np.deg2rad(10), 0, 0)
test = robot.kinematics(kinematics_type="inverse", x=300, y=20)
if test != "out_of_reach":
    print(np.rad2deg(test.get("upper_arm_angle")), np.rad2deg(test.get("lower_arm_angle")))
else:
    print("Error cought: out_of_reach")
