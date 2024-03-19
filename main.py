from robot import robot
import numpy as np
import time


pick_and_place = robot()
pick_and_place.hardware.move_dummy(110, 0)
input("Press Enter to move...")
pick_and_place.hardware.move_to(110, 0)

pick_and_place.hardware.move_dummy(90, 0)
input("Press Enter to move...")
pick_and_place.hardware.move_to(90, 0)