from sim import sim
import numpy as np
import time

simulation = sim()

for i in range(1000):
    simulation.move(np.deg2rad(-5), np.deg2rad(i/10))
    time.sleep(0.005)
