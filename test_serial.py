from robot import robot
import time

pnp = robot()
if pnp.controller.setup_controller():
    pnp.controller.check_zero_park()
    input("Press Enter to start the gripper test")
    while True:
        pnp.hardware.gripper("open")
        time.sleep(3)
        pnp.hardware.gripper("close")
        time.sleep(3)