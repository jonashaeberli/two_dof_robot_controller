from kinematics import kinematics
from hardware import hardware

class robot:
    def __init__(self):
        self.kinematics = kinematics()
        self.hardware = hardware()