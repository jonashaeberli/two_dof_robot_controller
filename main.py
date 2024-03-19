from robot import robot
import numpy as np
import time


pick_and_place = robot()
pick_and_place.hardware.setup()

pick_and_place.hardware.move_dummy(110, 5)
input("Press Enter to move...")
pick_and_place.hardware.move_to(110, 5)

pick_and_place.hardware.move_dummy(90, -5)
input("Press Enter to move...")
pick_and_place.hardware.move_to(90, -5)
