from kinematics import kinematics
from hardware import hardware
from planner import planner

class robot:
    def __init__(self):
        self.kinematics = kinematics()
        self.hardware = hardware()
        self.planner = planner()