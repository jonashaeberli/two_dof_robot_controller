from kinematics import kinematics
from hardware import hardware
from planner import planner
from sim import sim

class robot:
    def __init__(self):
        self.kinematics = kinematics()
        self.hardware = hardware()
        self.planner = planner()
        self.sim = sim()

        self.hardware.handover(self.kinematics, self.planner)
        self.planner.handover(self.hardware, self.kinematics) # TODO: To run in simulation we should use the sim instrad of the hardware maybe add method or input to select sim or hardware

        self.initialized = True

    
    def is_initialized(self):
        return self.initialized