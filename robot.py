from kinematics import kinematics
from hardware import hardware
from controller import controller
from sim import sim

class robot:
    def __init__(self, run_sim = False):
        self.kinematics = kinematics()
        self.hardware = hardware()
        self.controller = controller()
        self.sim = sim()

        self.hardware.handover(self.kinematics, self.controller)
        self.controller.handover(self.hardware if run_sim is False else self.sim, self.kinematics) # TODO: To run in simulation we should use the sim instrad of the hardware maybe add method or input to select sim or hardware

        self.initialized = True

    
    def is_initialized(self):
        return self.initialized