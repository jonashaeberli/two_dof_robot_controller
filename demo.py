import argparse
from robot import robot
import time


parser = argparse.ArgumentParser()
parser.add_argument('--vel', type=int, default=50, help='Velocity')
parser.add_argument('--accel', type=int, default=20, help='Acceleration')
args = parser.parse_args()


pnp = robot(run_sim=True)
pnp.controller.set_move_params(args.vel, args.accel)
if pnp.controller.setup_controller():
    pnp.controller.check_zero_park()
    pnp.controller.clear_trajectory()
    print(len(pnp.controller.trajectory))
    #Create a Trajectory
    pnp.controller.moveL(150, 80)
    pnp.controller.moveL(500, 80)
    pnp.controller.moveL(500, 40)
    pnp.controller.set_gripper("close")
    pnp.controller.pause_trajectory(1)
    pnp.controller.moveL(500, 80)
    pnp.controller.moveL(150, 80)
    pnp.controller.moveL(150, 40)
    pnp.controller.set_gripper("open")
    pnp.controller.pause_trajectory(1)
    pnp.controller.moveL(150, 80)
    if pnp.controller.validate_trajectory():
        print("Trajectory is valid")
        input("Press Enter to execute pnp trajectory")
        pnp.controller.execute_trajectory()
        input("Press Enter to close")