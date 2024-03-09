from two_dof_pick_and_place import two_dof_pick_and_place
import numpy as np


robot = two_dof_pick_and_place()
test = robot.kinematics(kinematics_type="inverse", x=250, y=60)
if test != "out_of_reach":
    print(np.rad2deg(test.get("upper_arm_angle")), np.rad2deg(test.get("lower_arm_angle")))
    print(robot.draw_mechanism_with_inverse_kinematic(250, 60))
    print(robot.kinematics(kinematics_type="forward", upper_arm_angle=test.get("upper_arm_angle"), lower_arm_angle=test.get("lower_arm_angle")))
else:
    print("Error cought: out_of_reach")
