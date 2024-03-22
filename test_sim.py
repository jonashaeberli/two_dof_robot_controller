from robot import robot
import numpy as np
import time

PnP = robot(run_sim = True)
PnP.controller.setup_controller()

debt_time = 0.0
for i in range(300,1000,2):
    i = i / 2
    angles = PnP.kinematics.kinematics(kinematics_type="inverse", x=i, y=100)
    start_time = time.time()  # start timing
    PnP.sim.move(angles.get("upper_arm_angle"), angles.get("lower_arm_angle"))

    end_time = time.time()  # end timing
    elapsed_time = end_time - start_time  # calculate elapsed time

    debt_time += elapsed_time - 0.005  # update debt time
    sleep_time = max(0.005 - elapsed_time, -debt_time)  # calculate sleep time, can't be more than debt time
    if sleep_time > 0:  # if there's time left, sleep
        time.sleep(sleep_time)
        debt_time += sleep_time - 0.005  # update debt time

    print(debt_time)  # print debt time to monitor it
