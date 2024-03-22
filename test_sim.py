from robot import robot

PnP = robot(run_sim = True)
PnP.controller.setup_controller()
for i in range(1000):
    PnP.sim.move(0, i/10)