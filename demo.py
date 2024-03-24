import argparse
from robot import robot
import time


parser = argparse.ArgumentParser()
parser.add_argument('--vel', type=int, default=1000, help='Velocity')
parser.add_argument('--accel', type=int, default=3500, help='Acceleration')
parser.add_argument('--calib', type=str, default='True', help='Run calibration')
args = parser.parse_args()

args.calib = args.calib.lower() in ['true', '1', 't', 'y', 'yes']

pnp = robot()
pnp.controller.set_move_params(args.vel, args.accel)
if pnp.controller.setup_controller():
    if args.calib:
        pnp.controller.check_zero_park()
    else:
        pnp.controller.park()
    pnp.controller.clear_trajectory()
    print(len(pnp.controller.trajectory))
    #First pick and place operation
    pnp.controller.moveL(150, 80, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.moveL(500, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(500, 50, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("close")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(500, 80, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.moveL(150, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(150, 50, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("open")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(150, 80, max_linear_velocity=1000, max_linear_acceleration=8000)

    #Second pick and place operation
    pnp.controller.moveL(475, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(475, 50, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("close")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(475, 80, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.moveL(175, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(175, 50, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("open")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(175, 80, max_linear_velocity=1000, max_linear_acceleration=8000)

    #Third pick and place operation
    pnp.controller.moveL(150, 120, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.moveL(150, 50, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("close")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(150, 120, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.moveL(500, 120, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(500, 50, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("open")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(500, 80, max_linear_velocity=1000, max_linear_acceleration=18500)

    #Fourth pick and place operation
    pnp.controller.moveL(175, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(175, 50, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("close")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(175, 80, max_linear_velocity=1000, max_linear_acceleration=8000)
    pnp.controller.moveL(475, 80, max_linear_velocity=1300, max_linear_acceleration=17000)
    pnp.controller.moveL(475, 50, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.set_gripper("open")
    pnp.controller.pause_trajectory(0.2)
    pnp.controller.moveL(475, 80, max_linear_velocity=1000, max_linear_acceleration=18500)
    pnp.controller.moveL(150, 80, max_linear_velocity=1300, max_linear_acceleration=17000)

    if pnp.controller.validate_trajectory():
        print("Trajectory is valid")
        print(pnp.controller.calculate_max_velocity_and_acceleration())
        input("Press Enter to execute pnp trajectory")
        pnp.controller.execute_trajectory()
        input("Press Enter to close")
