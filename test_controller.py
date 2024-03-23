from robot import robot
import time

pnp = robot(run_sim = True)
if pnp.controller.setup_controller():
    pnp.controller.check_zero_park()
time.sleep(5)