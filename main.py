from robot import robot
import numpy as np
import time


pick_and_place = robot()
pick_and_place.hardware.setup()

input("Press Enter to go to start position...")

if pick_and_place.kinematics.check_position(200, 50):
    start_pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=200, y=50)
    pick_and_place.hardware.move_to(np.rad2deg(start_pos.get("upper_arm_angle")), np.rad2deg(start_pos.get("lower_arm_angle")))

input("Press Enter to start the pick and place routine...")

while(True):
    for i in range (200,300):
        if pick_and_place.kinematics.check_position(i, 50):
            pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=i, y=50)
            pick_and_place.hardware.move_to(np.rad2deg(pos.get("upper_arm_angle")), np.rad2deg(pos.get("lower_arm_angle")))
            time.sleep(0.01)
    
    for i in range (50, 150):
        if pick_and_place.kinematics.check_position(300, i):
            pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=300, y=i)
            pick_and_place.hardware.move_to(np.rad2deg(pos.get("upper_arm_angle")), np.rad2deg(pos.get("lower_arm_angle")))
            time.sleep(0.01)
    
    for i in range (300, 200, -1):
        if pick_and_place.kinematics.check_position(i, 150):
            pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=i, y=150)
            pick_and_place.hardware.move_to(np.rad2deg(pos.get("upper_arm_angle")), np.rad2deg(pos.get("lower_arm_angle")))
            time.sleep(0.01)
    
    for i in range (150, 50, -1):
        if pick_and_place.kinematics.check_position(200, i):
            pos = pick_and_place.kinematics.kinematics(kinematics_type="inverse", x=200, y=i)
            pick_and_place.hardware.move_to(np.rad2deg(pos.get("upper_arm_angle")), np.rad2deg(pos.get("lower_arm_angle")))
            time.sleep(0.01)

